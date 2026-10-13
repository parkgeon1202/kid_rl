// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__IMU_HPP_
#define KID_RL__IMU_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>
#include <thread>

#include "kid_rl/input_types.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace kid_rl
{

class Imu
{
public:
  enum class Format {EULER_GYRO, QUATERNION_GYRO};

  struct Config
  {
    std::string port;
    int baudrate{115200};
    Format format{Format::EULER_GYRO};
    double angle_scale{0.017453292519943295};  // degrees -> radians
    double gyro_scale{0.017453292519943295};  // degrees/s -> radians/s
    // 센서 축 -> ROS 몸통 축 매핑. "x,y,z" 는 그대로 쓰는 것이고,
    // "-y,x,z" 처럼 부호와 순서를 바꿔 장착 방향을 반영한다.
    // 기본값은 항등이다. 실기에서 확인한 값을 controllers.yaml 의
    // imu.axis_map 에 적을 것.
    std::array<std::size_t, 3> axis_order{0, 1, 2};
    std::array<double, 3> axis_sign{1.0, 1.0, 1.0};
    // 자이로 전용 매핑. 이 EBIMU 는 자세와 각속도를 서로 다른 좌표 규약으로
    // 내보낸다(실측으로 확인, controllers.yaml 의 imu.gyro_axis_map 주석 참고).
    // 설정하지 않으면 위 axis_order/axis_sign 과 같은 값을 쓴다.
    std::array<std::size_t, 3> gyro_axis_order{0, 1, 2};
    std::array<double, 3> gyro_axis_sign{1.0, 1.0, 1.0};
  };

  Imu();
  ~Imu();

  Imu(const Imu &) = delete;
  Imu & operator=(const Imu &) = delete;

  bool start(const Config & config, std::string & error);
  void stop();
  ImuSample read_from_rt();
  bool running() const {return running_.load();}

  static bool parse_format(const std::string & text, Format & format);

  /// "x,y,z" / "-y,x,z" 같은 표기를 축 순서와 부호로 바꾼다.
  /// 세 축이 정확히 한 번씩 나와야 하고, 아니면 false 를 반환한다.
  static bool parse_axis_map(
    const std::string & text, std::array<std::size_t, 3> & order, std::array<double, 3> & sign);

private:
  bool open_serial(std::string & error);
  void read_loop();
  void consume_bytes(const char * data, std::size_t size);
  void parse_line(const std::string & line);

  Config config_;
  void report(const char * reason, const std::string & line);

  int fd_{-1};
  std::string line_buffer_;
  // 진단용. 이게 없으면 IMU 가 invalid 일 때 원인(헤더 없음/필드 부족/개행 없음)을
  // 알 방법이 로그에 전혀 없다. 2초에 한 번만 출력한다.
  std::size_t bytes_received_{0};
  std::size_t accepted_lines_{0};
  std::size_t rejected_lines_{0};
  std::int64_t last_report_ns_{0};
  std::atomic<bool> running_{false};
  std::thread thread_;
  realtime_tools::RealtimeBuffer<ImuSample> buffer_;
};

}  // namespace kid_rl

#endif  // KID_RL__IMU_HPP_
