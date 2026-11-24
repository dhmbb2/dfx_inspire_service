// Inspire RH56 serial protocol (user manual V1.0.9): register reads and writes with full-frame
// receive, checksum and address checks, and resynchronization on late or stray frames.
//
//   request  EB 90 | id | len | cmd | addr_lo addr_hi | payload       | checksum
//   reply    90 EB | id | len | cmd | addr_lo addr_hi | data / status | checksum
//   len = 3 + payload length; checksum = low byte of the sum from id to the last payload byte.
//   cmd 0x11 reads (payload: number of bytes to read), 0x12 writes (reply payload 0x01 = accepted).
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <vector>

#include "rh56/serial_port.h"

namespace rh56 {

namespace reg {
constexpr uint16_t HAND_ID = 1000;
constexpr uint16_t CLEAR_ERROR = 1004;
constexpr uint16_t FORCE_CALIBRATION = 1009;  // GESTURE_FORCE_CLB; write 1 with the hand unloaded
constexpr uint16_t ANGLE_SET = 1486;          // 6 x int16, 0..1000 (1000 = open), -1 = leave as is
constexpr uint16_t FORCE_SET = 1498;          // 6 x int16, force threshold [g], 0..1000
constexpr uint16_t SPEED_SET = 1522;          // 6 x int16, 0..1000
constexpr uint16_t ANGLE_ACT = 1546;          // 6 x int16
constexpr uint16_t FORCE_ACT = 1582;          // 6 x int16 [g]; followed directly by the next four
constexpr uint16_t CURRENT = 1594;            // 6 x int16 [mA]
constexpr uint16_t ERROR = 1606;              // 6 x uint8, fault bits
constexpr uint16_t STATUS = 1612;             // 6 x uint8, motion state
constexpr uint16_t TEMP = 1618;               // 6 x uint8 [deg C]
}  // namespace reg

constexpr uint8_t CMD_READ = 0x11;
constexpr uint8_t CMD_WRITE = 0x12;

enum class Result { Ok, Timeout, BadFrame, Rejected, IoError };

inline const char* to_string(Result r) {
  switch (r) {
    case Result::Ok: return "ok";
    case Result::Timeout: return "timeout";
    case Result::BadFrame: return "bad frame";
    case Result::Rejected: return "rejected";
    case Result::IoError: return "io error";
  }
  return "?";
}

struct Stats {
  uint64_t ok = 0, timeout = 0, bad_frame = 0, rejected = 0, io_error = 0;
  void add(Result r) {
    switch (r) {
      case Result::Ok: ++ok; break;
      case Result::Timeout: ++timeout; break;
      case Result::BadFrame: ++bad_frame; break;
      case Result::Rejected: ++rejected; break;
      case Result::IoError: ++io_error; break;
    }
  }
  uint64_t failed() const { return timeout + bad_frame + rejected + io_error; }
};

class Hand {
 public:
  Hand(SerialPort& port, uint8_t id, std::chrono::milliseconds timeout) : port_(port), id_(id), timeout_(timeout) {}

  uint8_t id() const { return id_; }
  SerialPort& port() { return port_; }

  Result read(uint16_t addr, uint8_t* out, uint8_t n) {
    std::vector<uint8_t> req = {0xEB, 0x90, id_, 0x04, CMD_READ, uint8_t(addr & 0xFF), uint8_t(addr >> 8), n, 0};
    return transact(req, CMD_READ, addr, out, n);
  }

  Result write(uint16_t addr, const uint8_t* data, uint8_t n) {
    std::vector<uint8_t> req = {0xEB, 0x90, id_, uint8_t(3 + n), CMD_WRITE, uint8_t(addr & 0xFF), uint8_t(addr >> 8)};
    req.insert(req.end(), data, data + n);
    req.push_back(0);
    return transact(req, CMD_WRITE, addr, nullptr, 0);
  }

  Result read_i16(uint16_t addr, int16_t* out, int count) {
    uint8_t raw[64];
    Result r = read(addr, raw, uint8_t(2 * count));
    if (r == Result::Ok)
      for (int i = 0; i < count; ++i) out[i] = int16_t(uint16_t(raw[2 * i]) | uint16_t(raw[2 * i + 1]) << 8);
    return r;
  }

  Result write_i16(uint16_t addr, const int16_t* v, int count) {
    uint8_t raw[64];
    for (int i = 0; i < count; ++i) {
      raw[2 * i] = uint8_t(uint16_t(v[i]) & 0xFF);
      raw[2 * i + 1] = uint8_t(uint16_t(v[i]) >> 8);
    }
    return write(addr, raw, uint8_t(2 * count));
  }

 private:
  Result transact(std::vector<uint8_t>& req, uint8_t cmd, uint16_t addr, uint8_t* out, uint8_t n_out) {
    uint8_t cs = 0;
    for (size_t k = 2; k + 1 < req.size(); ++k) cs += req[k];
    req.back() = cs;

    port_.flush_input();
    const auto deadline = Clock::now() + timeout_;
    if (!port_.write_all(req.data(), req.size(), deadline)) return Result::IoError;

    std::array<uint8_t, 256> buf;
    size_t have = 0;
    bool saw_other = false;  // got bytes, but not the reply we wanted
    while (true) {
      size_t i = 0;
      while (true) {
        while (i + 1 < have && !(buf[i] == 0x90 && buf[i + 1] == 0xEB)) ++i;
        if (i + 4 > have) break;                    // need the id and length bytes
        const size_t total = 5 + size_t(buf[i + 3]);
        if (buf[i + 3] < 3 || total > buf.size()) {  // implausible length: not a header
          ++i;
          saw_other = true;
          continue;
        }
        if (i + total > have) break;                // frame not complete yet
        uint8_t sum = 0;
        for (size_t k = i + 2; k + 1 < i + total; ++k) sum += buf[k];
        if (sum != buf[i + total - 1]) {            // corrupt: resync one byte later
          ++i;
          saw_other = true;
          continue;
        }
        const bool mine = buf[i + 2] == id_ && buf[i + 4] == cmd && buf[i + 5] == (addr & 0xFF) && buf[i + 6] == (addr >> 8);
        if (mine) {
          const uint8_t* payload = &buf[i + 7];
          const size_t plen = total - 8;
          if (cmd == CMD_READ) {
            if (plen != n_out) return Result::BadFrame;
            std::memcpy(out, payload, n_out);
            return Result::Ok;
          }
          return (plen >= 1 && payload[0] == 0x01) ? Result::Ok : Result::Rejected;
        }
        saw_other = true;                           // a valid frame for something else, e.g. a late ack
        i += total;
      }
      if (i > 0) {
        std::memmove(buf.data(), buf.data() + i, have - i);
        have -= i;
      }
      if (have == buf.size()) have = 0;             // garbage filled the buffer: start over
      const size_t r = port_.read_some(buf.data() + have, buf.size() - have, deadline);
      if (r == 0) return saw_other || have ? Result::BadFrame : Result::Timeout;
      have += r;
    }
  }

  SerialPort& port_;
  uint8_t id_;
  std::chrono::milliseconds timeout_;
};

}  // namespace rh56
