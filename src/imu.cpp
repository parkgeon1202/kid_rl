// Copyright 2026 kid_rl authors
// SPDX-License-Identifier: Apache-2.0

#include "kid_rl/imu.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace kid_rl
{
namespace
{

constexpr std::size_t kMaximumLineLength = 512;

speed_t baud_constant(int baudrate)
{
  switch (baudrate) {
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: return 0;
  }
}

bool finite_values(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

// 센서 좌표계 벡터를 ROS 몸통 좌표계로 옮긴다. 각속도와 projected gravity 는
// 둘 다 방향을 가진 3차원 벡터라 같은 변환을 적용한다.
//
// 예전에는 [y, x, -z] 가 하드코딩되어 있었다. 그 변환은 실기에서 틀렸을 뿐 아니라
// x 와 y 를 맞바꾸면서 z 부호만 뒤집는 거울상이라 물리적인 장착으로는 나올 수 없는
// 조합이었다. 이제 imu.axis_map 설정으로 바꾼다.
std::array<double, 3> apply_axis_map(
  const std::array<double, 3> & v,
  const std::array<std::size_t, 3> & order,
  const std::array<double, 3> & sign)
{
  return {sign[0] * v[order[0]], sign[1] * v[order[1]], sign[2] * v[order[2]]};
}

std::vector<double> parse_csv(const std::string & line)
{
  if (line.empty() || line.front() != '*') {return {};}
  std::vector<double> values;
  const char * cursor = line.c_str() + 1;
  while (*cursor != '\0' && *cursor != '\r' && *cursor != '\n') {
    char * end = nullptr;
    errno = 0;
    const double value = std::strtod(cursor, &end);
    // EBIMU 는 줄 끝에 2자리 16진수 체크섬을 붙인다 ("...,az,3A").
    // strtod 로는 읽을 수 없으므로 여기서 멈추고, 줄을 통째로 버리는 대신
    // 지금까지 읽은 값으로 호출자가 개수를 보고 판단하게 한다.
    if (end == cursor || errno == ERANGE) {break;}
    values.push_back(value);
    cursor = end;
    if (*cursor == ',') {
      ++cursor;
    } else {
      break;
    }
  }
  return values;
}

bool quaternion_to_projected_gravity(
  double w, double x, double y, double z, std::array<double, 3> & gravity)
{
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (!std::isfinite(norm) || norm <= 1e-9) {return false;}
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;
  gravity = {
    -2.0 * (x * z - w * y),
    -2.0 * (y * z + w * x),
    -(1.0 - 2.0 * (x * x + y * y))};
  return true;
}

}  // namespace

Imu::Imu()
{
  buffer_.writeFromNonRT(ImuSample{});
}

Imu::~Imu()
{
  stop();
}

bool Imu::parse_axis_map(
  const std::string & text, std::array<std::size_t, 3> & order, std::array<double, 3> & sign)
{
  std::array<std::size_t, 3> parsed_order{};
  std::array<double, 3> parsed_sign{};
  std::array<bool, 3> seen{false, false, false};

  std::size_t index = 0;
  std::size_t cursor = 0;
  while (index < 3) {
    const std::size_t comma = text.find(',', cursor);
    std::string token = text.substr(
      cursor, comma == std::string::npos ? std::string::npos : comma - cursor);
    token.erase(
      std::remove_if(
        token.begin(), token.end(),
        [](unsigned char c) {return std::isspace(c) != 0;}), token.end());
    if (token.empty()) {return false;}

    double axis_sign = 1.0;
    if (token.front() == '-') {
      axis_sign = -1.0;
      token.erase(token.begin());
    } else if (token.front() == '+') {
      token.erase(token.begin());
    }
    if (token.size() != 1) {return false;}

    std::size_t axis = 0;
    switch (token[0]) {
      case 'x': case 'X': axis = 0; break;
      case 'y': case 'Y': axis = 1; break;
      case 'z': case 'Z': axis = 2; break;
      default: return false;
    }
    // 같은 축을 두 번 쓰면 회전이 아니라 찌그러진 변환이 된다.
    if (seen[axis]) {return false;}
    seen[axis] = true;

    parsed_order[index] = axis;
    parsed_sign[index] = axis_sign;
    ++index;
    if (comma == std::string::npos) {break;}
    cursor = comma + 1;
  }

  if (index != 3 || !(seen[0] && seen[1] && seen[2])) {return false;}
  order = parsed_order;
  sign = parsed_sign;
  return true;
}

bool Imu::parse_format(const std::string & text, Format & format)
{
  if (text == "euler_gyro") {
    format = Format::EULER_GYRO;
    return true;
  }
  if (text == "quaternion_gyro") {
    format = Format::QUATERNION_GYRO;
    return true;
  }
  return false;
}

bool Imu::start(const Config & config, std::string & error)
{
  stop();
  config_ = config;
  line_buffer_.clear();
  buffer_.writeFromNonRT(ImuSample{});
  if (!open_serial(error)) {return false;}
  running_.store(true);
  thread_ = std::thread(&Imu::read_loop, this);
  return true;
}

void Imu::stop()
{
  running_.store(false);
  if (thread_.joinable()) {thread_.join();}
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  buffer_.writeFromNonRT(ImuSample{});
}

ImuSample Imu::read_from_rt()
{
  const auto * sample = buffer_.readFromRT();
  return sample ? *sample : ImuSample{};
}

bool Imu::open_serial(std::string & error)
{
  const speed_t speed = baud_constant(config_.baudrate);
  if (speed == 0) {
    error = "unsupported baudrate " + std::to_string(config_.baudrate);
    return false;
  }

  fd_ = open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error = "could not open " + config_.port + ": " + std::strerror(errno);
    return false;
  }

  termios options{};
  if (tcgetattr(fd_, &options) < 0) {
    error = "tcgetattr failed: " + std::string(std::strerror(errno));
    close(fd_);
    fd_ = -1;
    return false;
  }
  cfmakeraw(&options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~PARENB;
  options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);
  tcflush(fd_, TCIFLUSH);
  if (tcsetattr(fd_, TCSANOW, &options) < 0) {
    error = "tcsetattr failed: " + std::string(std::strerror(errno));
    close(fd_);
    fd_ = -1;
    return false;
  }
  return true;
}

void Imu::consume_bytes(const char * data, std::size_t size)
{
  for (std::size_t i = 0; i < size; ++i) {
    const char byte = data[i];
    // EBIMU 는 패킷 끝을 0x0D(CR) 로 보낸다(ebimu/src/e2box_imu_node/
    // e2box_imu_node.cpp:135). 예전에는 '\n' 에서만 파싱했기 때문에 CR 만 오는
    // 장치에서는 parse_line 이 한 번도 불리지 않았고, 샘플이 영원히 invalid 였다.
    // CRLF 를 보내는 장치에서는 두 번째 구분자에서 빈 줄이 되어 그냥 무시된다.
    if (byte == '\n' || byte == '\r') {
      if (!line_buffer_.empty()) {parse_line(line_buffer_);}
      line_buffer_.clear();
    } else {
      if (byte == '*') {line_buffer_.clear();}
      if (line_buffer_.size() < kMaximumLineLength) {
        line_buffer_.push_back(byte);
      } else {
        line_buffer_.clear();
      }
    }
  }
}

void Imu::report(const char * reason, const std::string & line)
{
  const std::int64_t now = steady_time_nanoseconds();
  if (last_report_ns_ != 0 && now - last_report_ns_ < 2000000000LL) {return;}
  last_report_ns_ = now;

  std::string shown;
  for (const char c : line.substr(0, 96)) {
    if (c >= 32 && c < 127) {
      shown.push_back(c);
    } else {
      char escaped[8];
      std::snprintf(escaped, sizeof(escaped), "\\x%02x", static_cast<unsigned char>(c));
      shown += escaped;
    }
  }
  std::fprintf(
    stderr,
    "[kid_rl imu] %s | bytes=%zu accepted=%zu rejected=%zu | \"%s\"\n",
    reason, bytes_received_, accepted_lines_, rejected_lines_, shown.c_str());
}

void Imu::parse_line(const std::string & line)
{
  const std::vector<double> values = parse_csv(line);
  const std::size_t required = config_.format == Format::EULER_GYRO ? 6U : 7U;
  if (values.empty()) {
    ++rejected_lines_;
    report("line rejected: must start with '*' and hold comma-separated numbers", line);
    return;
  }
  if (values.size() < required) {
    ++rejected_lines_;
    report(
      config_.format == Format::EULER_GYRO ?
      "too few fields: euler_gyro needs roll,pitch,yaw,gx,gy,gz (>=6)" :
      "too few fields: quaternion_gyro needs 4 quaternion + 3 gyro (>=7)", line);
    return;
  }
  if (!finite_values(values)) {
    ++rejected_lines_;
    report("non-finite value in line", line);
    return;
  }

  ImuSample sample;
  if (config_.format == Format::EULER_GYRO) {
    const double roll = values[0] * config_.angle_scale;
    const double pitch = values[1] * config_.angle_scale;
    const double yaw = values[2] * config_.angle_scale;
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
    if (!quaternion_to_projected_gravity(
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy,
      sample.projected_gravity)) {return;}
    sample.angular_velocity = {
      values[3] * config_.gyro_scale,
      values[4] * config_.gyro_scale,
      values[5] * config_.gyro_scale};
  } else {
    // EBIMU 의 쿼터니언 출력 순서는 *qz,qy,qx,qw 다. x 와 z 가 뒤집혀 있고 w 가
    // 마지막이다(장치 설정: <soc1> ASCII, <sof2> Quaternion, <sog1> Gyro).
    // 같은 장치를 읽는 ebimu 패키지도 이 순서를 되돌려 받는다:
    //   x = m_dQuaternion[2], y = [1], z = [0], w = [3]
    //   (ebimu/src/e2box_imu_node/e2box_imu_node.cpp:245-248)
    // 순서를 그대로 (w,x,y,z) 로 읽으면 파싱은 성공하지만 자세가 조용히 틀어진다.
    if (!quaternion_to_projected_gravity(
        values[3], values[2], values[1], values[0], sample.projected_gravity))
    {
      ++rejected_lines_;
      report("quaternion norm is zero or non-finite", line);
      return;
    }
    sample.angular_velocity = {
      values[4] * config_.gyro_scale,
      values[5] * config_.gyro_scale,
      values[6] * config_.gyro_scale};
  }
  // 정책의 observation은 ROS 몸통 좌표계를 기준으로 학습되었으므로, 센서 고유
  // 좌표계의 각속도와 중력 방향을 그대로 넣지 않고 장착 방향만큼 변환한다.
  const std::array<double, 3> raw_gravity = sample.projected_gravity;
  const std::array<double, 3> raw_gyro = sample.angular_velocity;
  // 자세와 각속도에 서로 다른 매핑을 쓴다. 보통은 같아야 하지만, 이 장치는
  // 두 신호의 좌표 규약이 달라서 따로 둬야 물리가 맞는다.
  sample.angular_velocity =
    apply_axis_map(sample.angular_velocity, config_.gyro_axis_order, config_.gyro_axis_sign);
  sample.projected_gravity =
    apply_axis_map(sample.projected_gravity, config_.axis_order, config_.axis_sign);
  sample.stamp_ns = steady_time_nanoseconds();
  sample.valid = true;
  buffer_.writeFromNonRT(sample);

  // 장착 방향을 눈으로 확인할 수 있게 변환 전/후를 주기적으로 찍는다.
  // 로봇을 똑바로 세운 상태라면 body 쪽 중력이 [0, 0, -1] 에 가까워야 한다.
  // sensor 쪽이 [0,0,-1] 인데 body 쪽이 [0,0,+1] 로 나오면 imu_to_ros_body 의
  // z 부호가 틀린 것이다(그 상태면 컨트롤러가 "뒤집혔다"고 보고 정책을 막는다).
  ++accepted_lines_;
  if (accepted_lines_ == 1) {last_report_ns_ = 0;}
  char summary[200];
  std::snprintf(
    summary, sizeof(summary),
    "gravity sensor=[%+.3f %+.3f %+.3f] body=[%+.3f %+.3f %+.3f] | "
    "gyro sensor=[%+.3f %+.3f %+.3f] rad/s",
    raw_gravity[0], raw_gravity[1], raw_gravity[2],
    sample.projected_gravity[0], sample.projected_gravity[1], sample.projected_gravity[2],
    raw_gyro[0], raw_gyro[1], raw_gyro[2]);
  report(summary, line);
}

void Imu::read_loop()
{
  std::array<char, 256> bytes{};
  pollfd descriptor{fd_, POLLIN, 0};
  while (running_.load()) {
    const int result = poll(&descriptor, 1, 100);
    if (result < 0) {
      if (errno == EINTR) {continue;}
      break;
    }
    if (result == 0) {continue;}
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))) {break;}
    if (!(descriptor.revents & POLLIN)) {continue;}

    const ssize_t count = read(fd_, bytes.data(), bytes.size());
    if (count < 0) {
      if (errno == EAGAIN || errno == EINTR) {continue;}
      break;
    }
    if (count > 0) {
      bytes_received_ += static_cast<std::size_t>(count);
      consume_bytes(bytes.data(), static_cast<std::size_t>(count));
      if (accepted_lines_ == 0 && bytes_received_ > 256) {
        // 바이트는 들어오는데 유효 샘플이 하나도 없는 상태. 버퍼 내용을 그대로
        // 보여주면 구분자/헤더/필드 수 중 무엇이 문제인지 바로 드러난다.
        report("receiving bytes but no valid sample yet; buffer now", line_buffer_);
      }
    }
  }
  running_.store(false);
  buffer_.writeFromNonRT(ImuSample{});
}

}  // namespace kid_rl
