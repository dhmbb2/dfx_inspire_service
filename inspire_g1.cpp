// Inspire RH56DFX driver for the Unitree G1: rt/inspire/cmd -> RS485 -> hands -> rt/inspire/state.
//
// Each hand has its own serial port and thread. Every exchange with a hand is a complete,
// checksummed frame with a deadline, so a late reply cannot shift later reads, and position
// feedback keeps flowing while commands are being written. Hands that do not answer at startup
// are skipped (--hands auto). The DDS messages keep the original layout; see README.md.
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <boost/program_options.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rh56/protocol.h"

namespace po = boost::program_options;
using rh56::Clock;
using rh56::Result;
namespace reg = rh56::reg;

namespace {

std::atomic<bool> g_stop{false};

void say(const char* fmt, ...) {
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  const int ms = int(std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000);
  char ts[16];
  std::strftime(ts, sizeof ts, "%H:%M:%S", std::localtime(&t));
  std::printf("[%s.%03d] ", ts, ms);
  va_list ap;
  va_start(ap, fmt);
  std::vprintf(fmt, ap);
  va_end(ap);
  std::printf("\n");
  std::fflush(stdout);
}

using Six = std::array<int16_t, 6>;
constexpr int16_t KEEP = -1;  // on the wire: leave this finger's target as it is

struct Snapshot {
  bool present = false;
  std::array<float, 6> q{};                // ANGLE_ACT / 1000, 1 = open
  std::array<float, 6> force_n{};          // FORCE_ACT [N]
  std::array<float, 6> current_ma{};       // CURRENT [mA]
  std::array<uint8_t, 6> error{}, status{}, temp{};
  uint32_t lost = 0;                       // failed position reads, cumulative
  bool have_q = false;
  Clock::time_point q_time{};
  rh56::Stats reads, writes, status_reads;
  Result last_error = Result::Ok;
  uint64_t cycles = 0;
};

class HandWorker {
 public:
  HandWorker(std::string side, std::string port, int id) : side_(std::move(side)), path_(std::move(port)), id_(uint8_t(id)) {}

  const std::string& side() const { return side_; }

  // Open the port and check that the hand answers.
  bool open(int timeout_ms) {
    try {
      port_ = std::make_unique<rh56::SerialPort>(path_);
    } catch (const std::exception& e) {
      say("%s hand: %s", side_.c_str(), e.what());
      return false;
    }
    const bool low = port_->set_low_latency();
    const int lat = port_->latency_timer_ms();
    hand_ = std::make_unique<rh56::Hand>(*port_, id_, std::chrono::milliseconds(timeout_ms));
    Six a{};
    Result r = Result::Timeout;
    for (int k = 0; k < 3 && r != Result::Ok; ++k) r = hand_->read_i16(reg::ANGLE_ACT, a.data(), 6);
    const std::string lat_s = lat >= 0 ? std::to_string(lat) + " ms" : "n/a";
    say("%s hand: %s, ID %d, low-latency mode %s, adapter latency timer %s: %s", side_.c_str(), path_.c_str(), id_,
        low ? "on" : "not available", lat_s.c_str(), r == Result::Ok ? "answers" : rh56::to_string(r));
    if (r != Result::Ok) return false;
    if (lat > 4)
      say("%s hand: warning: latency timer %d ms; replies may need up to that long (raise --timeout-ms if reads time out)",
          side_.c_str(), lat);
    speed_known_ = hand_->read_i16(reg::SPEED_SET, speed_.data(), 6) == Result::Ok;
    force_known_ = hand_->read_i16(reg::FORCE_SET, force_.data(), 6) == Result::Ok;
    std::lock_guard<std::mutex> l(snap_mutex_);
    snap_.present = true;
    set_q(a);
    return true;
  }

  rh56::Hand& hand() { return *hand_; }
  Six speed() const { return speed_; }
  Six force() const { return force_; }

  Result write_all_fingers(uint16_t addr, int16_t value) {
    Six v;
    v.fill(value);
    const Result r = hand_->write_i16(addr, v.data(), 6);
    if (r == Result::Ok) {
      (addr == reg::SPEED_SET ? speed_ : force_) = v;
      (addr == reg::SPEED_SET ? speed_known_ : force_known_) = true;
    }
    return r;
  }

  // From the DDS callback. speed/force entries of 0 mean "keep the current setting".
  void set_command(const Six& angle, const Six& speed, const Six& force) {
    std::lock_guard<std::mutex> l(cmd_mutex_);
    cmd_angle_ = angle;
    cmd_speed_ = speed;
    cmd_force_ = force;
    has_cmd_ = true;
  }

  Snapshot snapshot() {
    std::lock_guard<std::mutex> l(snap_mutex_);
    return snap_;
  }

  void run(int rate_hz, int status_every) {
    const auto period = std::chrono::nanoseconds(1000000000LL / rate_hz);
    auto next = Clock::now();
    uint64_t cycle = 0;
    while (!g_stop) {
      Six angle{}, speed_req{}, force_req{};
      bool has = false;
      {
        std::lock_guard<std::mutex> l(cmd_mutex_);
        has = has_cmd_;
        angle = cmd_angle_;
        speed_req = cmd_speed_;
        force_req = cmd_force_;
      }
      if (has) {
        write_if_changed(reg::SPEED_SET, speed_req, speed_, speed_known_);
        write_if_changed(reg::FORCE_SET, force_req, force_, force_known_);
        if (!angle_ok_ || angle != angle_written_) {
          const Result r = hand_->write_i16(reg::ANGLE_SET, angle.data(), 6);
          angle_ok_ = r == Result::Ok;
          if (angle_ok_) angle_written_ = angle;
          record_write(r);
        }
      }
      Six a{};
      const Result r = hand_->read_i16(reg::ANGLE_ACT, a.data(), 6);
      {
        std::lock_guard<std::mutex> l(snap_mutex_);
        snap_.reads.add(r);
        if (r == Result::Ok) {
          set_q(a);
        } else {
          ++snap_.lost;
          snap_.last_error = r;
        }
      }
      if (status_every > 0 && cycle % uint64_t(status_every) == 0) read_status();
      ++cycle;
      {
        std::lock_guard<std::mutex> l(snap_mutex_);
        snap_.cycles = cycle;
      }
      next += period;
      const auto now = Clock::now();
      if (next < now)
        next = now;  // overrun: do not try to catch up
      else
        std::this_thread::sleep_until(next);
    }
  }

 private:
  static Six merge(const Six& current, const Six& request) {
    Six out = current;
    for (int i = 0; i < 6; ++i)
      if (request[i] > 0) out[i] = request[i];
    return out;
  }

  // Per-finger requests (0 = keep) are merged into the register's current contents, which must be
  // known first: writing a partly unknown array would set the other fingers to 0.
  void write_if_changed(uint16_t addr, const Six& request, Six& current, bool& known) {
    if (std::all_of(request.begin(), request.end(), [](int16_t v) { return v <= 0; })) return;
    if (!known) known = hand_->read_i16(addr, current.data(), 6) == Result::Ok;
    if (!known) return;
    const Six wanted = merge(current, request);
    if (wanted == current) return;
    const Result r = hand_->write_i16(addr, wanted.data(), 6);
    if (r == Result::Ok) current = wanted;
    record_write(r);
  }

  void record_write(Result r) {
    std::lock_guard<std::mutex> l(snap_mutex_);
    snap_.writes.add(r);
    if (r != Result::Ok) snap_.last_error = r;
  }

  void set_q(const Six& a) {  // caller holds snap_mutex_
    for (int i = 0; i < 6; ++i) snap_.q[i] = float(a[i]) / 1000.0f;
    snap_.have_q = true;
    snap_.q_time = Clock::now();
  }

  // FORCE_ACT, CURRENT, ERROR, STATUS and TEMP are contiguous: one 42-byte read.
  void read_status() {
    uint8_t raw[42];
    const Result r = hand_->read(reg::FORCE_ACT, raw, 42);
    std::lock_guard<std::mutex> l(snap_mutex_);
    snap_.status_reads.add(r);
    if (r != Result::Ok) return;
    auto i16 = [&](int k) { return int16_t(uint16_t(raw[2 * k]) | uint16_t(raw[2 * k + 1]) << 8); };
    for (int i = 0; i < 6; ++i) {
      snap_.force_n[i] = float(i16(i)) * 9.80665f / 1000.0f;
      snap_.current_ma[i] = float(i16(6 + i));
      snap_.error[i] = raw[24 + i];
      snap_.status[i] = raw[30 + i];
      snap_.temp[i] = raw[36 + i];
    }
  }

  std::string side_, path_;
  uint8_t id_;
  std::unique_ptr<rh56::SerialPort> port_;
  std::unique_ptr<rh56::Hand> hand_;
  Six speed_{}, force_{};  // last known register contents
  bool speed_known_ = false, force_known_ = false;
  Six angle_written_{};
  bool angle_ok_ = false;

  std::mutex cmd_mutex_;
  Six cmd_angle_{}, cmd_speed_{}, cmd_force_{};
  bool has_cmd_ = false;

  std::mutex snap_mutex_;
  Snapshot snap_;
};

void fill_states(unitree_go::msg::dds_::MotorStates_& msg, int offset, HandWorker* w, uint32_t& absent_lost) {
  const auto now = Clock::now();
  if (!w) {
    ++absent_lost;  // no hand: every message counts as a failed read
    for (int i = 0; i < 6; ++i) {
      auto& s = msg.states()[offset + i];
      s = unitree_go::msg::dds_::MotorState_();
      s.lost() = absent_lost;
      s.reserve()[1] = UINT32_MAX;
    }
    return;
  }
  const Snapshot snap = w->snapshot();
  const double age_ms = snap.have_q ? std::chrono::duration<double, std::milli>(now - snap.q_time).count() : 1e12;
  for (int i = 0; i < 6; ++i) {
    auto& s = msg.states()[offset + i];
    s.q() = snap.q[i];
    s.tau_est() = snap.force_n[i];
    s.dq_raw() = snap.current_ma[i];
    s.temperature() = snap.temp[i];
    s.mode() = snap.status[i];
    s.lost() = snap.lost;
    s.reserve()[0] = snap.error[i];
    s.reserve()[1] = uint32_t(std::min(age_ms, double(UINT32_MAX)));
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string network, ns, hands, left_port, right_port;
  int left_id, right_id, rate, status_every, timeout_ms, speed, force;
  po::options_description desc("Inspire RH56DFX driver for Unitree G1 (rt/<namespace>/cmd -> hands -> rt/<namespace>/state)");
  desc.add_options()
    ("help,h", "show this help")
    ("network", po::value(&network)->default_value(""), "DDS network interface, e.g. eth0")
    ("namespace", po::value(&ns)->default_value("inspire"), "DDS topic namespace")
    ("hands", po::value(&hands)->default_value("auto"), "auto (use the hands that answer), left, right or both")
    ("left-port", po::value(&left_port)->default_value("/dev/ttyUSB2"), "left hand serial port")
    ("right-port", po::value(&right_port)->default_value("/dev/ttyUSB1"), "right hand serial port")
    ("left-id", po::value(&left_id)->default_value(1), "left hand ID")
    ("right-id", po::value(&right_id)->default_value(1), "right hand ID")
    ("rate", po::value(&rate)->default_value(100), "loop rate per hand and state publish rate [Hz]")
    ("status-every", po::value(&status_every)->default_value(5), "read force/current/error/status/temperature every N loops (0 = never)")
    ("timeout-ms", po::value(&timeout_ms)->default_value(25), "deadline for one exchange with a hand [ms]")
    ("speed", po::value(&speed)->default_value(-1), "set all finger speeds at startup, 1..1000 (-1 = leave)")
    ("force", po::value(&force)->default_value(-1), "set all finger force thresholds at startup, 1..1000 g (-1 = leave)")
    ("clear-errors", "clear the hands' fault flags at startup")
    ("calibrate-force", "calibrate the force sensors (hands unloaded) and exit; the fingers may move");
  po::variables_map vm;
  try {
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n\n", e.what());
    std::cerr << desc << std::endl;
    return 2;
  }
  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 0;
  }
  if (hands != "auto" && hands != "left" && hands != "right" && hands != "both") {
    std::fprintf(stderr, "--hands must be auto, left, right or both\n");
    return 2;
  }
  if (rate < 1 || rate > 500 || timeout_ms < 1) {
    std::fprintf(stderr, "--rate must be 1..500 and --timeout-ms positive\n");
    return 2;
  }

  say("Inspire RH56DFX driver for G1: hands %s, %d Hz, exchange timeout %d ms", hands.c_str(), rate, timeout_ms);
  std::unique_ptr<HandWorker> right, left;
  const bool want_right = hands != "left", want_left = hands != "right";
  if (want_right) {
    right = std::make_unique<HandWorker>("right", right_port, right_id);
    if (!right->open(timeout_ms)) {
      if (hands == "right" || hands == "both") return 1;
      say("right hand: skipped");
      right.reset();
    }
  }
  if (want_left) {
    left = std::make_unique<HandWorker>("left", left_port, left_id);
    if (!left->open(timeout_ms)) {
      if (hands == "left" || hands == "both") return 1;
      say("left hand: skipped");
      left.reset();
    }
  }
  if (!right && !left) {
    say("no hand answers; check power, cables and --left-port/--right-port");
    return 1;
  }
  std::vector<HandWorker*> workers;
  for (HandWorker* w : {right.get(), left.get()})
    if (w) workers.push_back(w);

  for (HandWorker* w : workers) {
    if (vm.count("clear-errors")) {
      const uint8_t one = 1;
      say("%s hand: clear errors: %s", w->side().c_str(), rh56::to_string(w->hand().write(reg::CLEAR_ERROR, &one, 1)));
    }
    if (speed >= 0)
      say("%s hand: speed %d: %s", w->side().c_str(), speed,
          rh56::to_string(w->write_all_fingers(reg::SPEED_SET, int16_t(std::clamp(speed, 1, 1000)))));
    if (force >= 0)
      say("%s hand: force threshold %d g: %s", w->side().c_str(), force,
          rh56::to_string(w->write_all_fingers(reg::FORCE_SET, int16_t(std::clamp(force, 1, 1000)))));
    const Six s = w->speed(), f = w->force();
    say("%s hand: speed %d %d %d %d %d %d, force threshold %d %d %d %d %d %d g", w->side().c_str(), s[0], s[1], s[2], s[3],
        s[4], s[5], f[0], f[1], f[2], f[3], f[4], f[5]);
  }

  if (vm.count("calibrate-force")) {
    for (HandWorker* w : workers) {
      Six before{}, after{};
      w->hand().read_i16(reg::FORCE_ACT, before.data(), 6);
      const uint8_t one = 1;
      const Result r = w->hand().write(reg::FORCE_CALIBRATION, &one, 1);
      say("%s hand: force calibration started: %s; waiting 10 s", w->side().c_str(), rh56::to_string(r));
      std::this_thread::sleep_for(std::chrono::seconds(10));
      w->hand().read_i16(reg::FORCE_ACT, after.data(), 6);
      say("%s hand: force [g] before %d %d %d %d %d %d, after %d %d %d %d %d %d", w->side().c_str(), before[0], before[1],
          before[2], before[3], before[4], before[5], after[0], after[1], after[2], after[3], after[4], after[5]);
    }
    return 0;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, network);
  auto pub = std::make_shared<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorStates_>>("rt/" + ns + "/state");
  pub->InitChannel();

  std::atomic<uint64_t> n_cmd{0}, n_rejected{0};
  auto sub = std::make_shared<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorCmds_>>("rt/" + ns + "/cmd");
  sub->InitChannel([&](const void* p) {
    const auto& m = *static_cast<const unitree_go::msg::dds_::MotorCmds_*>(p);
    if (m.cmds().size() < 12) {
      ++n_rejected;
      return;
    }
    for (const auto& c : m.cmds())
      if (!std::isfinite(c.q()) || !std::isfinite(c.dq()) || !std::isfinite(c.tau())) {
        ++n_rejected;
        return;
      }
    ++n_cmd;
    const std::array<std::pair<HandWorker*, int>, 2> sides{{{right.get(), 0}, {left.get(), 6}}};
    for (const auto& [w, off] : sides) {
      if (!w) continue;
      Six angle{}, spd{}, frc{};
      for (int i = 0; i < 6; ++i) {
        const auto& c = m.cmds()[off + i];
        angle[i] = c.q() < 0.0f ? KEEP : int16_t(std::lround(std::min(c.q(), 1.0f) * 1000.0f));
        spd[i] = c.dq() > 0.0f ? int16_t(std::clamp(std::lround(c.dq() * 1000.0f), 1L, 1000L)) : 0;
        frc[i] = c.tau() > 0.0f ? int16_t(std::clamp(std::lround(c.tau() * 1000.0f), 1L, 1000L)) : 0;
      }
      w->set_command(angle, spd, frc);
    }
  });

  std::signal(SIGINT, [](int) { g_stop = true; });
  std::signal(SIGTERM, [](int) { g_stop = true; });

  std::vector<std::thread> threads;
  for (HandWorker* w : workers) threads.emplace_back([w, rate, status_every] { w->run(rate, status_every); });
  threads.emplace_back([&] {
    unitree_go::msg::dds_::MotorStates_ msg;
    msg.states().resize(12);
    uint32_t absent_right = 0, absent_left = 0;
    const auto period = std::chrono::nanoseconds(1000000000LL / rate);
    auto next = Clock::now();
    while (!g_stop) {
      fill_states(msg, 0, right.get(), absent_right);
      fill_states(msg, 6, left.get(), absent_left);
      pub->Write(msg);
      next += period;
      std::this_thread::sleep_until(next);
    }
  });

  say("running; commands on rt/%s/cmd, state on rt/%s/state. Ctrl+C stops; the hands keep their last position.",
      ns.c_str(), ns.c_str());
  std::vector<Snapshot> prev(workers.size());
  for (size_t k = 0; k < workers.size(); ++k) prev[k] = workers[k]->snapshot();
  uint64_t prev_cmd = 0, prev_rej = 0;
  auto t_prev = Clock::now();
  while (!g_stop) {
    for (int k = 0; k < 50 && !g_stop; ++k) std::this_thread::sleep_for(std::chrono::milliseconds(100));
    const double dt = std::chrono::duration<double>(Clock::now() - t_prev).count();
    t_prev = Clock::now();
    const uint64_t c = n_cmd, rj = n_rejected;
    for (size_t k = 0; k < workers.size(); ++k) {
      const Snapshot s = workers[k]->snapshot();
      const Snapshot& p = prev[k];
      say("%s hand: %.0f Hz | position reads %llu ok, %llu failed | writes %llu ok, %llu failed | status reads %llu failed%s%s",
          workers[k]->side().c_str(), double(s.cycles - p.cycles) / dt, (unsigned long long)(s.reads.ok - p.reads.ok),
          (unsigned long long)(s.reads.failed() - p.reads.failed()), (unsigned long long)(s.writes.ok - p.writes.ok),
          (unsigned long long)(s.writes.failed() - p.writes.failed()),
          (unsigned long long)(s.status_reads.failed() - p.status_reads.failed()),
          s.last_error == Result::Ok ? "" : " | last error: ", s.last_error == Result::Ok ? "" : rh56::to_string(s.last_error));
      prev[k] = s;
    }
    if (c != prev_cmd || rj != prev_rej)
      say("commands: %.0f Hz%s", double(c - prev_cmd) / dt,
          rj != prev_rej ? (" | rejected " + std::to_string(rj - prev_rej) + " (fewer than 12 entries or non-finite values)").c_str() : "");
    prev_cmd = c;
    prev_rej = rj;
  }
  for (auto& t : threads) t.join();
  say("stopped; the hands keep their last position");
  std::_Exit(0);  // skip DDS teardown, which can block at exit
}
