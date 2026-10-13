// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "kid_rl/csv_logger.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>
#include <vector>

namespace kid_rl
{
namespace
{

void append_double(std::string & row, double value)
{
  char text[32];
  // %.6g 면 각도(rad)도 전류값도 사람이 읽을 만한 자리수로 나오고, 0 은 그냥 0 이다.
  std::snprintf(text, sizeof(text), "%.6g", value);
  row += text;
  row += ',';
}

}  // namespace

CsvLogger::~CsvLogger()
{
  stop();
}

std::string CsvLogger::default_path()
{
  const char * home = std::getenv("HOME");
  const std::string directory = std::string(home != nullptr ? home : ".") + "/kid_rl_logs";

  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm parts{};
  localtime_r(&now, &parts);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &parts);

  return directory + "/kid_rl_" + stamp + ".csv";
}

bool CsvLogger::start(
  const std::string & path,
  const std::vector<std::string> & joints,
  const std::vector<std::string> & mode_names,
  std::size_t buffer_samples,
  bool rx_missing_columns,
  bool bus_timing_columns,
  const std::vector<std::size_t> & telemetry_joints,
  std::string & error)
{
  stop();

  if (joints.size() > kMaxJoints) {
    error = "too many joints for the CSV logger";
    return false;
  }

  path_ = path.empty() ? default_path() : path;
  std::error_code code;
  const std::filesystem::path file_path(path_);
  if (file_path.has_parent_path()) {
    std::filesystem::create_directories(file_path.parent_path(), code);
    if (code) {
      error = "could not create " + file_path.parent_path().string() + ": " + code.message();
      return false;
    }
  }

  file_.open(path_, std::ios::out | std::ios::trunc);
  if (!file_.is_open()) {
    error = "could not open " + path_ + " for writing";
    return false;
  }

  // 열 구성: 먼저 로봇 전체 상태, 그다음 관절별 묶음. 관절 묶음을 이름으로
  // 끝에 몰아 두면 스프레드시트에서 한 관절씩 보기 쉽다.
  std::string header =
    "t_s,dt_s,mode,imu_valid,imu_age_ms,command_fresh,"
    "gravity_x,gravity_y,gravity_z,gyro_x,gyro_y,gyro_z,cmd_vx,cmd_vy,cmd_wz,gait_phase,"
    "policy_t_s,gyro_raw_x,gyro_raw_y,gyro_raw_z";
  if (rx_missing_columns) {
    // rx_missing_ids 는 공백으로 구분한 ID 목록 ("7 13"). 한 열에 넣어야
    // 스프레드시트에서 틱별로 한눈에 보인다.
    header += ",rx_missing_count,rx_missing_ids";
  }
  if (bus_timing_columns) {
    // read_ms: 이번 틱 SyncRead 시간 / joint_age_*: 정책이 쓰는 관절값의 나이 범위
    // prev_write_*: 직전 틱 SyncWrite (write() 는 update() 뒤에 돌기 때문)
    header += ",read_ms,read_ok,joint_age_min_ms,joint_age_max_ms,"
      "prev_write_sent,prev_write_ms,prev_write_drain_ms";
  }
  for (const auto & joint : joints) {
    header += ",pos_" + joint;
    header += ",vel_" + joint;
    header += ",target_" + joint;
    // 추종 오차. 서보가 목표를 못 따라가는지 한 열만 보면 알 수 있다.
    header += ",err_" + joint;
    header += ",action_" + joint;
    header += ",hwerr_" + joint;
  }
  // 전압·PWM 은 맨 끝에 붙인다. 기존 열의 위치가 바뀌지 않아 예전 분석 스크립트가 그대로 돈다.
  for (const std::size_t index : telemetry_joints) {
    if (index >= joints.size()) {
      error = "telemetry joint index out of range";
      return false;
    }
    header += ",vin_" + joints[index];
    header += ",duty_" + joints[index];
  }
  file_ << header << '\n';
  file_.flush();

  capacity_ = std::max<std::size_t>(buffer_samples, 64);
  buffer_.assign(capacity_, Sample{});
  head_.store(0, std::memory_order_relaxed);
  tail_.store(0, std::memory_order_relaxed);
  dropped_.store(0, std::memory_order_relaxed);
  mode_names_ = mode_names;
  rx_missing_columns_ = rx_missing_columns;
  bus_timing_columns_ = bus_timing_columns;
  telemetry_count_ = telemetry_joints.size();
  row_.reserve(4096);

  running_.store(true, std::memory_order_release);
  thread_ = std::thread(&CsvLogger::writer_loop, this);
  return true;
}

void CsvLogger::stop()
{
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    if (file_.is_open()) {file_.close();}
    return;
  }
  if (thread_.joinable()) {thread_.join();}
  drain();   // 스레드가 멈춘 뒤 남은 것까지 비운다.
  if (file_.is_open()) {
    file_.flush();
    file_.close();
  }
}

void CsvLogger::log(const Sample & sample)
{
  if (!running_.load(std::memory_order_acquire)) {return;}

  const std::uint64_t head = head_.load(std::memory_order_relaxed);
  if (head - tail_.load(std::memory_order_acquire) >= capacity_) {
    // 버퍼가 가득 찼다. 제어 루프를 막느니 이번 샘플을 버린다.
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  buffer_[head % capacity_] = sample;
  head_.store(head + 1, std::memory_order_release);
}

std::size_t CsvLogger::drain()
{
  std::size_t written = 0;
  std::uint64_t tail = tail_.load(std::memory_order_relaxed);
  const std::uint64_t head = head_.load(std::memory_order_acquire);

  while (tail < head) {
    const Sample & sample = buffer_[tail % capacity_];

    row_.clear();
    append_double(row_, sample.t_s);
    append_double(row_, sample.dt_s);
    if (sample.mode >= 0 && static_cast<std::size_t>(sample.mode) < mode_names_.size()) {
      row_ += mode_names_[static_cast<std::size_t>(sample.mode)];
    } else {
      row_ += std::to_string(sample.mode);
    }
    row_ += ',';
    row_ += sample.imu_valid != 0 ? "1," : "0,";
    append_double(row_, sample.imu_age_ms);
    row_ += sample.command_fresh != 0 ? "1," : "0,";
    for (const double value : sample.gravity) {append_double(row_, value);}
    for (const double value : sample.gyro) {append_double(row_, value);}
    for (const double value : sample.command) {append_double(row_, value);}
    append_double(row_, sample.gait_phase);
    append_double(row_, sample.policy_t_s);
    for (const double value : sample.gyro_raw) {append_double(row_, value);}
    if (rx_missing_columns_) {
      row_ += std::to_string(sample.rx_missing_count);
      row_ += ',';
      for (std::size_t i = 0; i < sample.rx_missing_count && i < kMaxJoints; ++i) {
        if (i > 0) {row_ += ' ';}
        row_ += std::to_string(sample.rx_missing_ids[i]);
      }
      row_ += ',';
    }
    if (bus_timing_columns_) {
      append_double(row_, sample.read_ms);
      row_ += sample.read_ok != 0 ? "1," : "0,";
      append_double(row_, sample.joint_age_min_ms);
      append_double(row_, sample.joint_age_max_ms);
      row_ += sample.prev_write_sent != 0 ? "1," : "0,";
      append_double(row_, sample.prev_write_ms);
      append_double(row_, sample.prev_write_drain_ms);
    }
    for (std::size_t i = 0; i < sample.joint_count; ++i) {
      append_double(row_, sample.position[i]);
      append_double(row_, sample.velocity[i]);
      append_double(row_, sample.target[i]);
      append_double(row_, sample.target[i] - sample.position[i]);
      append_double(row_, sample.action[i]);
      append_double(row_, sample.hardware_state[i]);
    }
    for (std::size_t i = 0; i < telemetry_count_ && i < kMaxJoints; ++i) {
      append_double(row_, sample.motor_volts[i]);
      append_double(row_, sample.motor_duty[i]);
    }
    if (!row_.empty() && row_.back() == ',') {row_.pop_back();}

    file_ << row_ << '\n';
    ++tail;
    ++written;
  }

  tail_.store(tail, std::memory_order_release);
  return written;
}

void CsvLogger::writer_loop()
{
  while (running_.load(std::memory_order_acquire)) {
    if (drain() > 0) {file_.flush();}
    // 100 Hz 기준 한 번에 최대 10틱 정도만 밀린다. 제어 스레드와 경쟁하지 않도록
    // 주기를 느슨하게 둔다.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace kid_rl
