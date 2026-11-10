// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "kid_rl/policy_runner.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "kid_rl/hurdle_vision.hpp"
#include "onnxruntime_cxx_api.h"  // NOLINT(build/include_subdir)

namespace kid_rl
{
namespace
{

bool ends_with(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::vector<std::string> active_joint_names(const std::string & metadata)
{
  std::vector<std::string> names;
  std::stringstream stream(metadata);
  std::string name;
  while (std::getline(stream, name, ',')) {
    // These linkage joints are passive in the MJCF and are deliberately absent
    // from both the action vector and the ros2_control joint list.
    if (!ends_with(name, "_roll_cap") && !ends_with(name, "_roll_actual")) {
      names.push_back(std::move(name));
    }
  }
  return names;
}

/// 이름 목록과 값 목록을 짝지어, 이름 쪽 필터를 통과한 값만 돌려준다.
/// 값만으로는 어느 항목이 수동 링키지인지 알 수 없으므로 이름이 함께 필요하다.
/// 두 목록의 길이가 다르면 빈 벡터를 돌려준다(호출자가 개수로 판단한다).
std::vector<double> active_joint_values(
  const std::string & names_metadata, const std::string & values_metadata)
{
  std::vector<std::string> names;
  std::stringstream name_stream(names_metadata);
  std::string name;
  while (std::getline(name_stream, name, ',')) {
    names.push_back(name);
  }

  std::vector<double> values;
  std::stringstream value_stream(values_metadata);
  std::string text;
  std::size_t index = 0;
  while (std::getline(value_stream, text, ',')) {
    if (index >= names.size()) {return {};}
    const std::string & current = names[index++];
    if (ends_with(current, "_roll_cap") || ends_with(current, "_roll_actual")) {continue;}
    try {
      values.push_back(std::stod(text));
    } catch (const std::exception &) {
      return {};
    }
  }
  if (index != names.size()) {return {};}
  return values;
}

bool has_shape(
  const std::vector<std::int64_t> & shape, std::size_t expected_width)
{
  return shape.size() == 2 && shape[0] == 1 &&
         shape[1] == static_cast<std::int64_t>(expected_width);
}

}  // namespace

struct PolicyRunner::Impl
{
  Ort::Env environment{ORT_LOGGING_LEVEL_WARNING, "kid_rl_policy"};
  Ort::SessionOptions session_options;
  std::unique_ptr<Ort::Session> session;
  Ort::AllocatorWithDefaultOptions allocator;
  Ort::MemoryInfo memory_info =
    Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);
  Ort::RunOptions run_options{nullptr};
  Ort::Value input_tensor{nullptr};
  Ort::Value output_tensor{nullptr};
  std::string input_name;
  std::string output_name;
  std::array<const char *, 1> input_names{};
  std::array<const char *, 1> output_names{};
  std::array<std::int64_t, 2> input_shape{};
  std::array<std::int64_t, 2> output_shape{};
  std::vector<float> input;
  std::vector<float> output;
  bool loaded{false};
};

PolicyRunner::PolicyRunner()
: impl_(std::make_unique<Impl>())
{
}

PolicyRunner::~PolicyRunner() = default;

bool PolicyRunner::load(
  const std::string & model_path,
  const std::vector<std::string> & base_observation_names,
  std::size_t base_observation_size,
  bool & uses_gait_phase,
  bool & uses_hurdle_vision,
  std::size_t stacked_observation_size,
  std::size_t & history_length,
  std::size_t expected_action_size,
  const std::vector<std::string> & expected_joint_names,
  const std::vector<double> & expected_default_positions,
  double expected_action_scale,
  std::string & error)
{
  reset();
  error.clear();
  uses_gait_phase = false;
  uses_hurdle_vision = false;
  history_length = 1;

  try {
    impl_->session_options.SetIntraOpNumThreads(1);
    impl_->session_options.SetInterOpNumThreads(1);
    impl_->session_options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    impl_->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    impl_->session = std::make_unique<Ort::Session>(
      impl_->environment, model_path.c_str(), impl_->session_options);

    if (impl_->session->GetInputCount() != 1 || impl_->session->GetOutputCount() != 1) {
      error = "policy must have exactly one input and one output";
      reset();
      return false;
    }

    // Keep TypeInfo alive while reading the nested tensor metadata. The C++
    // wrapper returned by GetTensorTypeAndShapeInfo() borrows that metadata.
    const Ort::TypeInfo input_type = impl_->session->GetInputTypeInfo(0);
    const Ort::TypeInfo output_type = impl_->session->GetOutputTypeInfo(0);
    const auto input_info = input_type.GetTensorTypeAndShapeInfo();
    const auto output_info = output_type.GetTensorTypeAndShapeInfo();
    if (input_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      output_info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
    {
      error = "policy input and output must both be float tensors";
      reset();
      return false;
    }
    const auto model_metadata = impl_->session->GetModelMetadata();

    // 관측 계약. 메타데이터가 없던 옛 모델은 위상 없는 기본 순서로 본다.
    auto observation_names = model_metadata.LookupCustomMetadataMapAllocated(
      "observation_names", impl_->allocator);
    if (observation_names) {
      std::vector<std::string> names;
      std::stringstream stream(observation_names.get());
      for (std::string name; std::getline(stream, name, ',');) {
        names.push_back(name);
      }
      if (!names.empty() && names.back() == "hurdle_vision") {
        uses_hurdle_vision = true;
        names.pop_back();
      }
      std::vector<std::string> with_phase = base_observation_names;
      with_phase.push_back("gait_phase");
      if (names == with_phase) {
        uses_gait_phase = true;
      } else if (names != base_observation_names) {
        error = "policy observation_names '" + std::string(observation_names.get()) +
          "' does not match the controller's observation layout";
        reset();
        return false;
      }
    }
    const std::size_t single_frame_size = base_observation_size + (uses_gait_phase ? 2 : 0) +
      (uses_hurdle_vision ? kHurdleVisionSize : 0);
    // 쌓은 프레임 수는 입력 폭에서 구한다. 나누어떨어지지 않으면 1 로 두고 아래에서 거부된다.
    const auto input_shape = input_info.GetShape();
    if (input_shape.size() == 2 && stacked_observation_size > 0 &&
      input_shape[1] > static_cast<std::int64_t>(single_frame_size))
    {
      const std::size_t extra = static_cast<std::size_t>(input_shape[1]) - single_frame_size;
      if (extra % stacked_observation_size == 0) {
        history_length = 1 + extra / stacked_observation_size;
      }
    }
    const std::size_t expected_observation_size =
      single_frame_size + (history_length - 1) * stacked_observation_size;

    if (!has_shape(input_info.GetShape(), expected_observation_size) ||
      !has_shape(output_info.GetShape(), expected_action_size))
    {
      error = "policy tensor shape does not match the observation/action sizes (expected [1," +
        std::to_string(expected_observation_size) + "] -> [1," +
        std::to_string(expected_action_size) + "])";
      reset();
      return false;
    }

    auto joint_names = model_metadata.LookupCustomMetadataMapAllocated(
      "joint_names", impl_->allocator);
    if (!joint_names) {
      error = "policy metadata has no joint_names entry";
      reset();
      return false;
    }
    if (active_joint_names(joint_names.get()) != expected_joint_names) {
      error = "policy joint_names order does not match the controller joint order";
      reset();
      return false;
    }

    // default_joint_pos: 관측(현재각 - 기본각), 액션 오프셋, 기동 목표, 진입
    // 판정에 모두 쓰인다. 어긋나면 네 군데가 한꺼번에 조용히 틀어진다.
    // 메타데이터는 1e-3 로 반올림되어 있으므로 여유를 둔다.
    auto default_pos = model_metadata.LookupCustomMetadataMapAllocated(
      "default_joint_pos", impl_->allocator);
    if (default_pos) {
      const std::vector<double> model_defaults =
        active_joint_values(joint_names.get(), default_pos.get());
      if (model_defaults.size() != expected_default_positions.size()) {
        error = "policy default_joint_pos count does not match the controller joint count";
        reset();
        return false;
      }
      for (std::size_t i = 0; i < model_defaults.size(); ++i) {
        if (std::abs(model_defaults[i] - expected_default_positions[i]) > 2e-3) {
          error = "policy default_joint_pos disagrees with default_positions for joint '" +
            expected_joint_names[i] + "' (policy " + std::to_string(model_defaults[i]) +
            ", config " + std::to_string(expected_default_positions[i]) + ")";
          reset();
          return false;
        }
      }
    }

    // action_scale: target = default + action_scale * action 이므로, 설정이 두 배면
    // 로봇이 정책이 의도한 변위의 두 배를 명령한다. 2026-09-21 실기에서 학습값
    // 0.5 에 배포값 1.0 이 들어간 채로 돌아 정책이 발산했다.
    auto action_scale = model_metadata.LookupCustomMetadataMapAllocated(
      "action_scale", impl_->allocator);
    if (action_scale) {
      try {
        const double model_scale = std::stod(action_scale.get());
        if (std::abs(model_scale - expected_action_scale) > 1e-6) {
          error = "policy action_scale is " + std::to_string(model_scale) +
            " but the controller is configured for " + std::to_string(expected_action_scale);
          reset();
          return false;
        }
      } catch (const std::exception &) {
        error = "policy action_scale metadata is not a number";
        reset();
        return false;
      }
    }

    auto input_name = impl_->session->GetInputNameAllocated(0, impl_->allocator);
    auto output_name = impl_->session->GetOutputNameAllocated(0, impl_->allocator);
    impl_->input_name = input_name.get();
    impl_->output_name = output_name.get();
    impl_->input_names[0] = impl_->input_name.c_str();
    impl_->output_names[0] = impl_->output_name.c_str();

    impl_->input.assign(expected_observation_size, 0.0F);
    impl_->output.assign(expected_action_size, 0.0F);
    impl_->input_shape = {1, static_cast<std::int64_t>(expected_observation_size)};
    impl_->output_shape = {1, static_cast<std::int64_t>(expected_action_size)};
    impl_->input_tensor = Ort::Value::CreateTensor<float>(
      impl_->memory_info, impl_->input.data(), impl_->input.size(),
      impl_->input_shape.data(), impl_->input_shape.size());
    impl_->output_tensor = Ort::Value::CreateTensor<float>(
      impl_->memory_info, impl_->output.data(), impl_->output.size(),
      impl_->output_shape.data(), impl_->output_shape.size());
    impl_->loaded = true;
    return true;
  } catch (const Ort::Exception & exception) {
    error = exception.what();
  } catch (const std::exception & exception) {
    error = exception.what();
  }

  reset();
  return false;
}

void PolicyRunner::reset() noexcept
{
  impl_->loaded = false;
  impl_->input_tensor = Ort::Value{nullptr};
  impl_->output_tensor = Ort::Value{nullptr};
  impl_->session.reset();
  impl_->input.clear();
  impl_->output.clear();
  impl_->input_name.clear();
  impl_->output_name.clear();
  impl_->input_names[0] = nullptr;
  impl_->output_names[0] = nullptr;
}

bool PolicyRunner::run(
  const std::vector<double> & observation, std::vector<double> & actions) noexcept
{
  if (!impl_->loaded || observation.size() != impl_->input.size() ||
    actions.size() != impl_->output.size())
  {
    return false;
  }

  try {
    for (std::size_t i = 0; i < observation.size(); ++i) {
      impl_->input[i] = static_cast<float>(observation[i]);
    }
    impl_->session->Run(
      impl_->run_options,
      impl_->input_names.data(), &impl_->input_tensor, 1,
      impl_->output_names.data(), &impl_->output_tensor, 1);
    for (std::size_t i = 0; i < actions.size(); ++i) {
      actions[i] = static_cast<double>(impl_->output[i]);
    }
    return true;
  } catch (...) {
    return false;
  }
}

bool PolicyRunner::is_loaded() const noexcept
{
  return impl_->loaded;
}

}  // namespace kid_rl
