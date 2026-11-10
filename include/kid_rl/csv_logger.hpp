// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#ifndef KID_RL__CSV_LOGGER_HPP_
#define KID_RL__CSV_LOGGER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace kid_rl
{

/// 제어 루프가 본 값을 CSV 파일로 남긴다.
///
/// 설계에서 지켜야 하는 것은 하나다: update() 안에서는 절대 파일을 건드리지
/// 않는다. 디스크 쓰기는 멈칫하는 시간이 예측 불가라 100 Hz 주기를 놓친다.
/// 그래서 update() 는 미리 할당해 둔 링 버퍼에 값을 복사만 하고(할당도 락도
/// 없다), 별도 스레드가 그 버퍼를 비우며 파일에 쓴다.
///
/// 생산자 하나(제어 스레드) / 소비자 하나(쓰기 스레드) 구조라 락이 필요 없다.
/// 버퍼가 가득 차면 새 샘플을 버리고 개수만 센다. 로깅 때문에 제어가 밀리는
/// 것보다 기록이 조금 비는 편이 낫기 때문이다.
class CsvLogger
{
public:
  static constexpr std::size_t kMaxJoints = 32;

  /// 한 제어 틱의 기록. 고정 크기 POD 라 복사가 예측 가능하다.
  struct Sample
  {
    double t_s{0.0};             ///< 로깅 시작 기준 경과 시간
    double dt_s{0.0};            ///< 이번 틱의 주기 (지터 확인용)
    int mode{0};                 ///< 컨트롤러 모드 (이름은 start() 에서 받는다)
    int imu_valid{0};
    /// 마지막 IMU 샘플의 나이 [ms]. imu_valid 는 imu_timeout_s 를 넘겨야 0 이 되므로
    /// 그보다 짧은 끊김은 이 값으로만 보인다. 샘플이 한 번도 없으면 -1.
    double imu_age_ms{-1.0};
    int command_fresh{0};
    /// /hurdle/pose. hurdle_fresh: 이번 틱 관측 프레임이 [1, x, y, z] 인가(지난 정책 스텝 뒤 새로 받음).
    int hurdle_fresh{0};
    double hurdle_age_ms{-1.0};       ///< 마지막 수신의 나이 [ms]. 받은 적 없으면 -1
    double hurdle_period_ms{-1.0};    ///< 마지막 수신과 그 직전 수신의 간격 [ms]. 모르면 -1
    double hurdle_position[3]{};      ///< base_link x, y, z [m]. fresh 가 아니면 0
    double gravity[3]{};         ///< 몸통 좌표계 투영 중력
    double gyro[3]{};            ///< 몸통 좌표계 각속도 [rad/s], 관측에 들어간 값 (필터 후)
    double gyro_raw[3]{};        ///< 필터 전 IMU 각속도 [rad/s]
    double command[3]{};         ///< vx, vy, wz
    double gait_phase{-1.0};     ///< 관측에 넣은 보행 위상 [0,1), 안 쓰면 -1
    /// 첫 추론(= 학습의 에피소드 시작, 위상 0) 이후 흐른 실제 시간 [s]. 추론 전·정책 밖은 -1.
    /// sim 로그(scripts/mjlab_sim_csv.py)의 같은 열과 맞춰 두 실행을 정렬하는 기준이다.
    double policy_t_s{-1.0};
    /// 이번 틱 SyncRead 에서 응답이 빠진 서보 ID (관절 순서). RxMissingMonitor 가 채운다.
    std::size_t rx_missing_count{0};
    int rx_missing_ids[kMaxJoints]{};
    /// 버스 타이밍 (BusTimingMonitor 가 채운다). 모두 ms.
    double read_ms{0.0};            ///< 이번 틱 SyncRead 트랜잭션에 걸린 시간
    int read_ok{0};                 ///< 이번 틱 SyncRead 성공 여부
    double joint_age_min_ms{-1.0};  ///< 지금 쓰는 관절값의 나이 하한 (now - 성공 읽기 끝)
    double joint_age_max_ms{-1.0};  ///< 지금 쓰는 관절값의 나이 상한 (now - 성공 읽기 시작)
    int prev_write_sent{0};         ///< 직전 틱에 SyncWrite 를 보냈는가
    double prev_write_ms{0.0};      ///< 직전 틱 SyncWrite 호출 시간 (커널에 넘기기까지)
    double prev_write_drain_ms{-1.0};  ///< 직전 틱 송신 큐가 빌 때까지 (측정 끄면 -1)
    std::size_t joint_count{0};
    double position[kMaxJoints]{};
    double velocity[kMaxJoints]{};
    double target[kMaxJoints]{};
    double action[kMaxJoints]{};
    double hardware_state[kMaxJoints]{};  ///< 0 이 아니면 그 서보가 고장 보고 중
    /// MotorTelemetryMonitor 가 켜졌을 때만 채워진다. 관절 순서가 아니라 start() 에 넘긴
    /// telemetry_joints 순서다.
    double motor_volts[kMaxJoints]{};     ///< Present Input Voltage [V]
    double motor_duty[kMaxJoints]{};      ///< Present PWM / 885, 부호 있음 (-1..1)
  };

  CsvLogger() = default;
  ~CsvLogger();
  CsvLogger(const CsvLogger &) = delete;
  CsvLogger & operator=(const CsvLogger &) = delete;

  /// 파일을 열고 헤더를 쓴 뒤 쓰기 스레드를 띄운다.
  /// @param path 빈 문자열이면 default_path() 를 쓴다.
  /// @param joints 열 이름에 쓰일 관절 이름.
  /// @param mode_names 모드 번호를 사람이 읽는 이름으로 바꾸는 표.
  /// @param rx_missing_columns true 면 rx_missing_count/rx_missing_ids 열을 쓴다
  ///        (RxMissingMonitor 가 켜졌을 때).
  /// @param telemetry_joints 전압·PWM 을 기록할 관절의 joints 안 위치. 비어 있으면 열을
  ///        만들지 않는다. 열은 행의 맨 끝에 vin_<관절>, duty_<관절> 순서로 붙는다.
  bool start(
    const std::string & path,
    const std::vector<std::string> & joints,
    const std::vector<std::string> & mode_names,
    std::size_t buffer_samples,
    bool rx_missing_columns,
    bool bus_timing_columns,
    const std::vector<std::size_t> & telemetry_joints,
    std::string & error);

  /// 남은 샘플을 모두 쓰고 파일을 닫는다.
  void stop();

  bool running() const {return running_.load(std::memory_order_acquire);}
  /// 버퍼가 가득 차서 버린 샘플 수. 0 이 아니면 버퍼를 키우거나 decimation 을 올린다.
  std::size_t dropped() const {return dropped_.load(std::memory_order_relaxed);}
  const std::string & path() const {return path_;}

  /// 제어 스레드에서 호출한다. 할당도 락도 파일 접근도 하지 않는다.
  void log(const Sample & sample);

  /// ~/kid_rl_logs/kid_rl_YYYYmmdd_HHMMSS.csv
  static std::string default_path();

private:
  void writer_loop();
  std::size_t drain();

  std::vector<Sample> buffer_;
  std::size_t capacity_{0};
  std::atomic<std::uint64_t> head_{0};
  std::atomic<std::uint64_t> tail_{0};
  std::atomic<std::size_t> dropped_{0};
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::ofstream file_;
  std::string path_;
  std::vector<std::string> mode_names_;
  bool rx_missing_columns_{false};
  bool bus_timing_columns_{false};
  std::size_t telemetry_count_{0};
  std::string row_;
};

}  // namespace kid_rl

#endif  // KID_RL__CSV_LOGGER_HPP_
