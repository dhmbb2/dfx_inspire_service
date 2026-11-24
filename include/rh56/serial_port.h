// Raw 8N1 serial port for RS485 hand links: non-blocking I/O with deadlines, input flush, and
// low-latency mode for FTDI adapters.
#pragma once

#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

namespace rh56 {

using Clock = std::chrono::steady_clock;

class SerialPort {
 public:
  explicit SerialPort(const std::string& path, speed_t baud = B115200) : path_(path) {
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("cannot open " + path + ": " + std::strerror(errno));
    termios t{};
    if (tcgetattr(fd_, &t) != 0) fail("tcgetattr");
    cfmakeraw(&t);
    t.c_cflag &= ~(CSIZE | CSTOPB | PARENB | CRTSCTS);
    t.c_cflag |= CS8 | CLOCAL | CREAD;
    t.c_iflag &= ~(IXON | IXOFF | IXANY);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 0;
    cfsetispeed(&t, baud);
    cfsetospeed(&t, baud);
    if (tcsetattr(fd_, TCSANOW, &t) != 0) fail("tcsetattr");
    tcflush(fd_, TCIOFLUSH);
  }
  ~SerialPort() {
    if (fd_ >= 0) ::close(fd_);
  }
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  // Ask the kernel driver for low-latency mode. For ftdi_sio this sets the adapter's latency
  // timer to 1 ms (the default 16 ms delays every reply). Returns false where unsupported (a pty).
  bool set_low_latency() {
    serial_struct ss{};
    if (ioctl(fd_, TIOCGSERIAL, &ss) != 0) return false;
    ss.flags |= ASYNC_LOW_LATENCY;
    return ioctl(fd_, TIOCSSERIAL, &ss) == 0;
  }

  // FTDI latency timer in ms read from sysfs, or -1 when the device has none.
  int latency_timer_ms() const {
    char real[PATH_MAX];
    if (!realpath(path_.c_str(), real)) return -1;
    std::string name(real);
    name = name.substr(name.rfind('/') + 1);
    std::ifstream f("/sys/bus/usb-serial/devices/" + name + "/latency_timer");
    int v = -1;
    return (f >> v) ? v : -1;
  }

  // Drop unread input: late or stray bytes must not be taken for the next reply.
  void flush_input() { tcflush(fd_, TCIFLUSH); }

  bool write_all(const uint8_t* data, size_t n, Clock::time_point deadline) {
    size_t off = 0;
    while (off < n) {
      ssize_t w = ::write(fd_, data + off, n - off);
      if (w > 0) {
        off += size_t(w);
        continue;
      }
      if (w < 0 && errno != EAGAIN && errno != EINTR) return false;
      if (!wait(POLLOUT, deadline)) return false;
    }
    return true;
  }

  // Read whatever arrives first, waiting until the deadline. Returns 0 on timeout or error.
  size_t read_some(uint8_t* buf, size_t cap, Clock::time_point deadline) {
    while (true) {
      ssize_t r = ::read(fd_, buf, cap);
      if (r > 0) return size_t(r);
      if (r < 0 && errno != EAGAIN && errno != EINTR) return 0;
      if (!wait(POLLIN, deadline)) return 0;
    }
  }

  const std::string& path() const { return path_; }

 private:
  bool wait(short events, Clock::time_point deadline) {
    auto left = std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now()).count();
    if (left <= 0) return false;
    pollfd p{fd_, events, 0};
    int r = ::poll(&p, 1, int(left));
    return r > 0 && (p.revents & events);
  }
  [[noreturn]] void fail(const char* what) {
    std::string msg = std::string(what) + " failed on " + path_ + ": " + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error(msg);
  }

  std::string path_;
  int fd_ = -1;
};

}  // namespace rh56
