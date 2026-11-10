// Copyright 2026 ARTS contributors
// SPDX-License-Identifier: Apache-2.0
//
// disc_drive: the da Vinci X instrument adapter's Dynamixel disc servos (one serial bus per
// arm), velocity-commanded. A port of collaborator A's working Humble setup, with A's values:
// XC330-M288, protocol 2.0, IDs 1-5, velocity mode, velocity PI 500 / 1600, profile velocity 0
// (baud now 2 Mbps; A's was 1 Mbps), positions zeroed at start-up (A's offsets: the discs'
// counts when the node starts; or tool_homing's home counts, zero_counts), torque off on exit.
//
// Departures from A's node, each deliberate:
//  - Operating Mode (address 11) lives in EEPROM. A writes it on every start; this node reads
//    it first and writes only if it differs AND allow_mode_write is true, otherwise it refuses
//    to start and says which mode the servo is in. Same end state, no write when not needed,
//    and no EEPROM write the operator did not ask for.
//  - Every write is checked (A checked the I gain but not the P gain).
//  - Position AND velocity come back in one sync read (addresses 128-135); A bulk-read
//    position only.
//  - A command older than command_timeout zeroes the velocities (A had the same 0.5 s
//    staleness on its jog input).
//  - The serial port comes from config/disc_buses.yaml by the FTDI adapter's serial number
//    (/dev/serial/by-id), not /dev/ttyUSBn, whose order changes between plugs.
//
//
// Instrument engagement and homing: collaborator A's scheme as is (engage_sequence.hpp), one
// service per moving step so the operator gives a go before each. Its mode switches are
// EEPROM writes (A's scheme makes them on every engagement; PI decision 2026-11-02); each is
// logged, and one that would not change the mode is skipped.
//
// Topics and services (in the arm's namespace, decision 18):
//   tool_actuator_target  sensor_msgs/JointState  in: velocity [rad/s] per disc, by name
//                         (disc_1 .. disc_5) or, with no names, in ID order; obeyed only in
//                         the velocity phases (before engagement, and once homed)
//   tool_actuator_states  sensor_msgs/JointState  out: disc_1..disc_5 position [rad] since
//                         start-up (since homing, once homed), velocity [rad/s], effort =
//                         present current [mA, raw servo units]
//   disc_drive/engage_state  std_msgs/String (latched): the engagement phase
//   disc_drive/{go_to_start, spin, find_limits, return_to_start, stop}  std_srvs/Trigger
//   disc_drive/set_zero   std_srvs/Trigger: the joint zero at the discs' present counts, in a
//                         velocity phase. For MANUAL engagement (collaborator A's practice:
//                         jog each disc by gamepad until it drops into the instrument, align,
//                         then zero), since the automatic spin does not work on the adapter.
//
// Instrument joints (when instrument_joints is set, from config/instruments/<file>.yaml by the
// launch): collaborator A's teleop path, value for value. Disc angles from joint angles,
// [disc_a, disc_b, ...] = coupling * [roll, wrist_pitch, wrist_yaw, jaw] (disc angles since the
// zero, A's MotorCntToJang / JvelToMotorVel); joint velocity targets slowed near the limits by
// v <= 6 (limit - q) (A's LimitVelocityNearJointLimit, gain 6 /s), applied to roll, pitch,
// yaw, the jaw opening and, as A does, to each jaw blade (yaw +- jaw/2) with the yaw limits.
//   tool_joint_states   sensor_msgs/JointState  out: <arm>_tool_<joint> position [rad]
//   tool_joint_target   sensor_msgs/JointState  in: velocity [rad/s] by those names
//   tool_jaw_target     std_msgs/Float64        in: the unitless jaw command, -10 .. 90 (0 = just
//                       closed, 90 = fully open, below 0 = grip, -10 = full grip; clamped);
//                       the jaw follows it at the file's jaw_gain and overrides a jaw velocity
//                       from tool_joint_target while fresh
// INTERIM topics: decision 18 puts these joints in <arm>/joint_states, which waits for the
// ros2_control component. A disc target and a joint target: the newer fresh one is obeyed.
// Joint targets are kept PER JOINT, each zeroed when older than command_timeout, so two
// sources can share the instrument: the arm's controller (roll, pitch, yaw) and the jaw
// follower (jaw) each send only their own joints (decision 17, 2026-11-03 amendment).

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include "arts_dv_tool_driver/engage_sequence.hpp"
#include "arts_dv_tool_driver/instrument_joints.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace
{
// XC330-M288 control table (collaborator A's dxl_interface.hpp; the servo's e-manual).
constexpr uint16_t kAddrOperatingMode = 11;   // EEPROM
constexpr uint16_t kAddrTorqueEnable = 64;
constexpr uint16_t kAddrVelocityIGain = 76;
constexpr uint16_t kAddrVelocityPGain = 78;
constexpr uint16_t kAddrGoalCurrent = 102;
constexpr uint16_t kAddrGoalVelocity = 104;
constexpr uint16_t kAddrProfileVelocity = 112;
constexpr uint16_t kAddrGoalPosition = 116;
// One sync read: present current (2 bytes) at 126, velocity (4) at 128, position (4) at 132.
constexpr uint16_t kAddrPresentCurrent = 126;
constexpr uint16_t kLenPresentCurVelPos = 10;
constexpr uint8_t kVelocityMode = arts_dv_tool_driver::kModeVelocity;
constexpr double kProtocol = 2.0;
constexpr double kCountsPerRev = 4096.0;
constexpr double kVelocityUnitRpm = 0.229;
constexpr double kTwoPi = 2.0 * M_PI;

}  // namespace

class DiscDrive : public rclcpp::Node, public arts_dv_tool_driver::DiscBus
{
public:
  DiscDrive()
  : Node("disc_drive")
  {
    port_name_ = declare_parameter<std::string>("port", "");
    // 2 Mbps with Return Delay Time 0 on every servo since 2026-11-04 (bus probe: ~985 Hz).
    baud_ = declare_parameter<int>("baud_rate", 2000000);
    // The joint zero from tool_homing (its home_<serial>.yaml; launch zero_from_homing:=true)
    // instead of the discs' counts at start-up. Refused if any disc is more than
    // max_zero_drift ticks from it: the counts survive mode changes and process restarts while
    // the servos stay powered (measured 2026-11-04), but a power cycle drops multi-turn counts
    // back into one turn, a whole multiple of 4096 away.
    zero_counts_ = declare_parameter<std::vector<int64_t>>("zero_counts", std::vector<int64_t>{});
    max_zero_drift_ = declare_parameter<int>("max_zero_drift", 300);
    // Grip cap, dVRK's jaw ftmax by motor current (raw Present Current units): a jaw closing
    // target is held while a jaw disc draws >= the cap. Full grip (jaw command -10, or a jaw
    // velocity from tool_joint_target) = jaw_grip_cap, 1000 (PI, 2026-11-05; collaborator A runs
    // uncapped, i.e. up to the servos' Current Limit, read as 2352 on all ten servos 2026-11-05;
    // to be tuned after experiments); the command scales it (-5 -> 500), never below
    // jaw_touch_cap, 150 (the adapter designer's jaw move/hold current when homing), so a free
    // jaw still closes. The direct disc jog (tool_actuator_target) is NOT capped.
    jaw_grip_cap_ = declare_parameter<double>("jaw_grip_cap", 1000.0);
    jaw_touch_cap_ = declare_parameter<double>("jaw_touch_cap", 150.0);
    jaw_gain_ = declare_parameter<double>("instrument_jaw_gain", 10.0);
    const auto ids = declare_parameter<std::vector<int64_t>>("ids", {1, 2, 3, 4, 5});
    // The UR's 500 Hz. At 1 Mbps with the factory Return Delay Time the bus capped this at 333 Hz
    // (2026-11-03); at 2 Mbps, delay 0, a bus cycle takes ~1 ms (probe 2026-11-04), so 500 Hz is
    // paced, not flat out.
    rate_hz_ = declare_parameter<double>("rate_hz", 500.0);
    timeout_s_ = declare_parameter<double>("command_timeout", 0.5);
    max_rpm_ = declare_parameter<double>("max_rpm", 30.0);
    // Instrument-joint targets: a separate cap near the XC330-M288's no-load speed (81 rpm at
    // 5 V), applied by scaling ALL discs together so the joint direction is kept. The 30 rpm
    // max_rpm (direct disc jog) cut a fast jaw to ~6 rad/s on 2026-11-02, and clipping each
    // disc separately bends a coupled command. Collaborator A sends the coupling's velocities
    // uncapped.
    max_rpm_instrument_ = declare_parameter<double>("max_rpm_instrument", 75.0);
    vel_p_ = declare_parameter<int>("velocity_p_gain", 500);
    vel_i_ = declare_parameter<int>("velocity_i_gain", 1600);
    allow_mode_write_ = declare_parameter<bool>("allow_mode_write", false);
    // Protocol 2.0 Fast Sync Read (servo firmware >= 45; ours are 53): every servo answers in
    // ONE combined packet instead of one packet each. With the plain sync read a bus cycle
    // costs a fixed 5 ms (200 Hz whatever rate_hz asks, measured 2026-11-03).
    fast_sync_read_ = declare_parameter<bool>("fast_sync_read", true);
    // A's effective homing-loop period (see engage_sequence.hpp, TIMING), measured on the bus.
    engage_period_s_ = declare_parameter<double>("engage_tick_period", 0.0);
    for (auto id : ids) {
      ids_.push_back(static_cast<uint8_t>(id));
      names_.push_back("disc_" + std::to_string(id));
    }
    instrument_ = declare_parameter<std::string>("instrument", "");
    jnames_ = declare_parameter<std::vector<std::string>>(
      "instrument_joints", std::vector<std::string>{});
    const auto jdiscs = declare_parameter<std::vector<int64_t>>(
      "instrument_discs", std::vector<int64_t>{});
    const auto coupling = declare_parameter<std::vector<double>>(
      "instrument_coupling", std::vector<double>{});
    jmin_ = declare_parameter<std::vector<double>>("instrument_min", std::vector<double>{});
    jmax_ = declare_parameter<std::vector<double>>("instrument_max", std::vector<double>{});
    goal_rad_s_.assign(ids_.size(), 0.0);
    const std::size_t n = jnames_.size();
    if (n > 0) {
      if (jdiscs.size() != n) {throw std::runtime_error("instrument: joints and discs disagree");}
      inst_ = std::make_unique<arts_dv_tool_driver::InstrumentJoints>(
        jnames_, coupling, jmin_, jmax_);
      if (inst_->jawIndex() >= 0 && jmax_[inst_->jawIndex()] > 0.0) {
        jaw_current_cap_ = jaw_grip_cap_;
        RCLCPP_INFO(get_logger(), "jaw: command -10..90, grip cap %.0f at -10 (touch %.0f), gain %.1f /s",
          jaw_grip_cap_, jaw_touch_cap_, jaw_gain_);
        jawsub_ = create_subscription<std_msgs::msg::Float64>(
          "tool_jaw_target", 10,
          [this](std_msgs::msg::Float64::SharedPtr m) {
            std::lock_guard<std::mutex> lock(mutex_);
            jaw_cmd_ = m->data;
            jaw_cmd_stamp_ = now();
            have_jaw_cmd_ = true;
            last_target_ = now();
            have_target_ = true;
            joint_source_ = true;
          });
      }
      for (std::size_t r = 0; r < n; ++r) {
        bool found = false;
        for (std::size_t i = 0; i < ids_.size(); ++i) {
          if (ids_[i] == jdiscs[r]) {jdisc_index_.push_back(i); found = true;}
        }
        if (!found) {throw std::runtime_error("instrument disc id not on this bus");}
      }
      std::string prefix = get_namespace();
      prefix = prefix.size() > 1 ? prefix.substr(1) + "_" : "";
      for (const auto & j : jnames_) {jfull_.push_back(prefix + "tool_" + j);}
      jvel_cmd_.assign(n, 0.0);
      jvel_stamp_.assign(n, rclcpp::Time(0, 0, get_clock()->get_clock_type()));
      jpub_ = create_publisher<sensor_msgs::msg::JointState>("tool_joint_states", 10);
      jsub_ = create_subscription<sensor_msgs::msg::JointState>(
        "tool_joint_target", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr m) {onJointTarget(*m);});
      RCLCPP_INFO(get_logger(), "instrument %s: joints %zu, coupling loaded", instrument_.c_str(),
                  n);
    }
    offsets_.assign(ids_.size(), 0);

    seq_ = std::make_unique<arts_dv_tool_driver::EngageSequence>(
      *this, engage_period_s_, [this](const std::string & m) {RCLCPP_INFO(get_logger(), "engage %s", m.c_str());});
    pub_ = create_publisher<sensor_msgs::msg::JointState>("tool_actuator_states", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "~/engage_state", rclcpp::QoS(1).transient_local());
    using Trigger = std_srvs::srv::Trigger;
    auto service = [this](const char * name, std::function<std::string()> request) {
        services_.push_back(create_service<Trigger>(
          std::string("~/") + name,
          [this, name, request](const Trigger::Request::SharedPtr,
          Trigger::Response::SharedPtr res) {
            std::string why;
            {
              std::lock_guard<std::mutex> lock(bus_mutex_);
              // stop is always accepted; the moving steps need engageReady().
              why = (std::string(name) == "stop" || engageReady()) ? request() : engageRefusal();
            }
            res->success = why.empty();
            res->message = why.empty() ? std::string(name) + " accepted" : why;
            if (!why.empty()) {RCLCPP_WARN(get_logger(), "%s", why.c_str());}
          }));
      };
    service("go_to_start", [this] {return seq_->goToStart(seconds());});
    service("spin", [this] {return seq_->spin(last_counts_);});
    service("find_limits", [this] {return seq_->findLimits(last_counts_, seconds());});
    service("return_to_start", [this] {return seq_->returnToStart();});
    // set_zero needs no engagement readiness (no motion, no mode write), only a velocity phase.
    services_.push_back(create_service<Trigger>(
      "~/set_zero",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        if (!running_ || !seq_->acceptsVelocity()) {
          res->success = false;
          res->message = std::string("set_zero: refused in phase ") +
          arts_dv_tool_driver::phaseName(seq_->phase()) + " (needs a velocity phase)";
          RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
          return;
        }
        offsets_.assign(last_counts_.begin(), last_counts_.begin() + ids_.size());
        std::string c;
        for (auto v : offsets_) {c += (c.empty() ? "" : ", ") + std::to_string(v);}
        res->success = true;
        res->message = "zero set at counts [" + c + "]";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      }));
    service("stop", [this] {
        if (seq_->acceptsVelocity()) {sendVelocities(std::vector<double>(ids_.size(), 0.0));}
        seq_->stop();
        return std::string();
      });
    sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "tool_actuator_target", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr m) {onTarget(*m);});
  }

  ~DiscDrive() override {shutdown();}

  bool start()
  {
    if (port_name_.empty()) {
      RCLCPP_ERROR(get_logger(), "parameter 'port' is empty (config/disc_buses.yaml)");
      return false;
    }
    port_ = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packet_ = dynamixel::PacketHandler::getPacketHandler(kProtocol);
    if (!port_->openPort() || !port_->setBaudRate(baud_)) {
      RCLCPP_ERROR(get_logger(), "cannot open %s at %d baud", port_name_.c_str(), baud_);
      return false;
    }
    opened_ = true;
    RCLCPP_INFO(get_logger(), "bus %s at %d baud, ids %s", port_name_.c_str(), baud_,
                idList().c_str());

    for (std::size_t i = 0; i < ids_.size(); ++i) {
      const uint8_t id = ids_[i];
      if (!write1(id, kAddrTorqueEnable, 0, "torque off")) {return false;}
      uint8_t mode = 255;
      uint8_t err = 0;
      if (packet_->read1ByteTxRx(port_, id, kAddrOperatingMode, &mode, &err) != COMM_SUCCESS) {
        RCLCPP_ERROR(get_logger(), "id %u: cannot read the operating mode", id);
        return false;
      }
      modes_.push_back(mode);
      if (mode != kVelocityMode) {
        if (!allow_mode_write_) {
          RCLCPP_ERROR(
            get_logger(), "id %u is in operating mode %u, not velocity (%u). Setting it is an "
            "EEPROM write (collaborator A's node does it on every start). Re-run with "
            "allow_mode_write:=true to write it.", id, mode, kVelocityMode);
          return false;
        }
        if (!write1(id, kAddrOperatingMode, kVelocityMode, "operating mode (EEPROM)")) {
          return false;
        }
        RCLCPP_WARN(get_logger(), "id %u: operating mode %u -> velocity (EEPROM written)", id,
                    mode);
        modes_.back() = kVelocityMode;
      }
      if (!write2(id, kAddrVelocityPGain, static_cast<uint16_t>(vel_p_), "velocity P gain") ||
        !write2(id, kAddrVelocityIGain, static_cast<uint16_t>(vel_i_), "velocity I gain") ||
        !write4(id, kAddrProfileVelocity, 0, "profile velocity"))
      {
        return false;
      }
    }

    reader_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_, packet_, kAddrPresentCurrent, kLenPresentCurVelPos);
    fast_reader_ = std::make_unique<dynamixel::GroupFastSyncRead>(
      port_, packet_, kAddrPresentCurrent, kLenPresentCurVelPos);
    writer_ = std::make_unique<dynamixel::GroupSyncWrite>(port_, packet_, kAddrGoalVelocity, 4);
    pos_writer_ = std::make_unique<dynamixel::GroupSyncWrite>(
      port_, packet_, kAddrGoalPosition, 4);
    for (auto id : ids_) {
      if (!reader_->addParam(id) || !fast_reader_->addParam(id)) {
        RCLCPP_ERROR(get_logger(), "id %u: sync read setup failed", id);
        return false;
      }
    }
    std::vector<int32_t> counts, vel;
    std::vector<int16_t> cur;
    if (!readAll(counts, vel, cur)) {
      RCLCPP_ERROR(get_logger(), "first read failed");
      return false;
    }
    offsets_ = counts;  // A's zero: the discs where they are now
    std::string zero_from = "the current disc positions";
    if (!zero_counts_.empty()) {
      if (zero_counts_.size() != ids_.size()) {
        RCLCPP_ERROR(get_logger(), "zero_counts has %zu values for %zu discs", zero_counts_.size(),
                     ids_.size());
        return false;
      }
      std::string d;
      for (std::size_t i = 0; i < ids_.size(); ++i) {
        const int64_t drift = counts[i] - zero_counts_[i];
        d += (d.empty() ? "" : ", ") + std::to_string(drift);
        if (std::abs(drift) > max_zero_drift_) {
          RCLCPP_ERROR(get_logger(), "disc %u is %ld ticks from its homed count %ld (max %d): "
                       "power cycle or moved since homing; home again", ids_[i],
                       static_cast<long>(drift), static_cast<long>(zero_counts_[i]),
                       max_zero_drift_);
          return false;
        }
        offsets_[i] = static_cast<int32_t>(zero_counts_[i]);
      }
      zero_from = "the homed counts (now off by [" + d + "] ticks)";
    }
    rememberCounts(counts);
    for (auto id : ids_) {
      if (!write1(id, kAddrTorqueEnable, 1, "torque on")) {return false;}
    }
    torque_on_ = true;
    RCLCPP_INFO(get_logger(), "torque on, velocity mode, zero at %s", zero_from.c_str());
    publishPhase();
    running_ = true;
    thread_ = std::thread([this] {loop();});
    return true;
  }

  void shutdown()
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (opened_) {
      if (torque_on_) {
        if (seq_->acceptsVelocity()) {sendVelocities(std::vector<double>(ids_.size(), 0.0));}
        for (auto id : ids_) {
          uint8_t err = 0;
          packet_->write1ByteTxRx(port_, id, kAddrTorqueEnable, 0, &err);
        }
        torque_on_ = false;
        RCLCPP_INFO(get_logger(), "velocities zeroed, torque off");
      }
      port_->closePort();
      opened_ = false;
    }
  }

  bool faulted() const {return fault_;}

  // DiscBus, for EngageSequence. Called with bus_mutex_ held.
  bool torqueAll(bool on) override
  {
    bool ok = true;
    for (std::size_t i = 0; i < ids_.size(); ++i) {ok = torque(static_cast<int>(i), on) && ok;}
    return ok;
  }
  bool torque(int disc, bool on) override
  {
    return write1(ids_[disc], kAddrTorqueEnable, on ? 1 : 0, on ? "torque on" : "torque off");
  }
  bool operatingMode(uint8_t mode) override
  {
    for (std::size_t i = 0; i < ids_.size(); ++i) {
      if (modes_[i] == mode) {continue;}  // no EEPROM write that changes nothing
      if (!write1(ids_[i], kAddrOperatingMode, mode, "operating mode (EEPROM)")) {return false;}
      ++eeprom_writes_;
      RCLCPP_WARN(get_logger(), "id %u: operating mode %u -> %u (EEPROM write %d this run)",
                  ids_[i], modes_[i], mode, eeprom_writes_);
      modes_[i] = mode;
    }
    return true;
  }
  bool profileVelocity(uint32_t units) override
  {
    for (auto id : ids_) {
      if (!write4(id, kAddrProfileVelocity, units, "profile velocity")) {return false;}
    }
    return true;
  }
  bool goalCurrent(int disc, uint16_t units) override
  {
    return write2(ids_[disc], kAddrGoalCurrent, units, "goal current");
  }
  bool goalPositions(const arts_dv_tool_driver::Counts & counts) override
  {
    pos_writer_->clearParam();
    for (std::size_t i = 0; i < ids_.size(); ++i) {
      const int32_t u = counts[i];
      uint8_t b[4] = {
        static_cast<uint8_t>(u & 0xff), static_cast<uint8_t>((u >> 8) & 0xff),
        static_cast<uint8_t>((u >> 16) & 0xff), static_cast<uint8_t>((u >> 24) & 0xff)};
      pos_writer_->addParam(ids_[i], b);
    }
    if (pos_writer_->txPacket() != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "goal position sync write failed");
      return false;
    }
    return true;
  }
  bool zeroVelocities() override {return sendVelocities(std::vector<double>(ids_.size(), 0.0));}
  void pause(int ms) override {std::this_thread::sleep_for(std::chrono::milliseconds(ms));}

private:
  double seconds() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
           .count();
  }

  // The engagement needs servo IDs 1-5 in order (A's indexing) and a measured tick period.
  bool engageReady() const {return engageRefusal().empty();}
  std::string engageRefusal() const
  {
    for (std::size_t i = 0; i < ids_.size(); ++i) {
      if (ids_.size() != arts_dv_tool_driver::kDiscs || ids_[i] != static_cast<uint8_t>(i + 1)) {
        return "engagement needs ids 1-5 in order";
      }
    }
    if (!(engage_period_s_ > 0.0)) {
      return "engage_tick_period is not set: it must be A's measured homing-loop period "
             "(engage_sequence.hpp, TIMING)";
    }
    if (!running_) {return "the bus loop is not running";}
    return "";
  }

  void rememberCounts(const std::vector<int32_t> & counts)
  {
    for (std::size_t i = 0; i < ids_.size() && i < arts_dv_tool_driver::kDiscs; ++i) {
      last_counts_[i] = counts[i];
    }
  }

  void publishPhase()
  {
    std_msgs::msg::String m;
    m.data = arts_dv_tool_driver::phaseName(seq_->phase());
    state_pub_->publish(m);
  }

  std::string idList() const
  {
    std::string s;
    for (auto id : ids_) {s += (s.empty() ? "" : ",") + std::to_string(id);}
    return s;
  }

  bool check(int comm, uint8_t err, uint8_t id, const char * what)
  {
    if (comm != COMM_SUCCESS) {
      RCLCPP_ERROR(get_logger(), "id %u: %s: %s", id, what, packet_->getTxRxResult(comm));
      return false;
    }
    if (err != 0) {
      RCLCPP_ERROR(get_logger(), "id %u: %s: %s", id, what, packet_->getRxPacketError(err));
      return false;
    }
    return true;
  }
  bool write1(uint8_t id, uint16_t addr, uint8_t v, const char * what)
  {
    uint8_t err = 0;
    return check(packet_->write1ByteTxRx(port_, id, addr, v, &err), err, id, what);
  }
  bool write2(uint8_t id, uint16_t addr, uint16_t v, const char * what)
  {
    uint8_t err = 0;
    return check(packet_->write2ByteTxRx(port_, id, addr, v, &err), err, id, what);
  }
  bool write4(uint8_t id, uint16_t addr, uint32_t v, const char * what)
  {
    uint8_t err = 0;
    return check(packet_->write4ByteTxRx(port_, id, addr, v, &err), err, id, what);
  }

  bool readAll(
    std::vector<int32_t> & counts, std::vector<int32_t> & vel, std::vector<int16_t> & cur)
  {
    // The SDK's methods are not virtual, so the reader is chosen here, not by a base pointer.
    dynamixel::GroupSyncRead & r = fast_sync_read_ ? *fast_reader_ : *reader_;
    const int comm = fast_sync_read_ ? fast_reader_->txRxPacket() : reader_->txRxPacket();
    if (comm != COMM_SUCCESS) {return false;}
    counts.resize(ids_.size());
    vel.resize(ids_.size());
    cur.resize(ids_.size());
    for (std::size_t i = 0; i < ids_.size(); ++i) {
      if (!r.isAvailable(ids_[i], kAddrPresentCurrent, kLenPresentCurVelPos)) {
        return false;
      }
      cur[i] = static_cast<int16_t>(r.getData(ids_[i], kAddrPresentCurrent, 2));
      vel[i] = static_cast<int32_t>(r.getData(ids_[i], kAddrPresentCurrent + 2, 4));
      counts[i] = static_cast<int32_t>(r.getData(ids_[i], kAddrPresentCurrent + 6, 4));
    }
    return true;
  }

  bool sendVelocities(const std::vector<double> & rad_s, double max_rpm = 0.0)
  {
    writer_->clearParam();
    const double limit = (max_rpm > 0.0 ? max_rpm : max_rpm_) / kVelocityUnitRpm;
    for (std::size_t i = 0; i < ids_.size(); ++i) {
      double units = rad_s[i] * 60.0 / kTwoPi / kVelocityUnitRpm;
      units = std::max(-limit, std::min(limit, units));
      const int32_t u = static_cast<int32_t>(std::lround(units));
      uint8_t b[4] = {
        static_cast<uint8_t>(u & 0xff), static_cast<uint8_t>((u >> 8) & 0xff),
        static_cast<uint8_t>((u >> 16) & 0xff), static_cast<uint8_t>((u >> 24) & 0xff)};
      writer_->addParam(ids_[i], b);
    }
    return writer_->txPacket() == COMM_SUCCESS;
  }

  void onTarget(const sensor_msgs::msg::JointState & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Last writer wins. While the arm's servo controller streams tool_joint_target (wrist in
    // the controller, 500 Hz) a disc jog (gamepad, ~50 Hz) is overwritten at once: say so
    // rather than fail silently (2026-11-06). Not overridden on purpose: the controller holds its
    // last tip pose, so after a jog and set_zero it would drive the wrist straight back. Zero
    // with only disc_drive running, then start the arm (or deactivate servo_controller first).
    if (joint_source_ && have_target_ && (now() - last_target_).seconds() < timeout_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "disc jog IGNORED: tool_joint_target is streaming (the arm's servo controller drives the "
        "instrument). Engage and zero before starting the arm, or deactivate servo_controller.");
    }
    if (m.name.empty()) {
      for (std::size_t i = 0; i < ids_.size() && i < m.velocity.size(); ++i) {
        goal_rad_s_[i] = m.velocity[i];
      }
    } else {
      for (std::size_t k = 0; k < m.name.size() && k < m.velocity.size(); ++k) {
        for (std::size_t i = 0; i < names_.size(); ++i) {
          if (names_[i] == m.name[k]) {goal_rad_s_[i] = m.velocity[k];}
        }
      }
    }
    last_target_ = now();
    have_target_ = true;
    joint_source_ = false;
  }

  void onJointTarget(const sensor_msgs::msg::JointState & m)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t k = 0; k < m.name.size() && k < m.velocity.size(); ++k) {
      for (std::size_t j = 0; j < jfull_.size(); ++j) {
        if (jfull_[j] == m.name[k] || jnames_[j] == m.name[k]) {
          jvel_cmd_[j] = m.velocity[k];
          jvel_stamp_[j] = now();
        }
      }
    }
    last_target_ = now();
    have_target_ = true;
    joint_source_ = true;
  }

  // Joint velocities -> disc velocities (all discs; those not in the instrument at 0).
  std::vector<double> discFromJoint(
    const Eigen::VectorXd & q, const std::vector<double> & qd, const std::vector<int16_t> & cur,
    double jaw_cap)
  {
    Eigen::VectorXd jd = Eigen::Map<const Eigen::VectorXd>(qd.data(), static_cast<Eigen::Index>(qd.size()));
    std::vector<double> row_cur(jdisc_index_.size(), 0.0);
    for (std::size_t r = 0; r < jdisc_index_.size(); ++r) {
      if (jdisc_index_[r] < cur.size()) {row_cur[r] = cur[jdisc_index_[r]];}
    }
    if (inst_->capJawClosing(jd, row_cur, jaw_cap)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "jaw grip at the current cap (%.0f): closing held", jaw_cap);
    }
    const Eigen::VectorXd v = inst_->discVelocities(q, jd);
    std::vector<double> out(ids_.size(), 0.0);
    for (std::size_t r = 0; r < jdisc_index_.size(); ++r) {out[jdisc_index_[r]] = v[r];}
    return out;
  }

  void loop()
  {
    // The bus loop runs at rate_hz in the velocity phases and at exactly engage_tick_period
    // (A's loop period, one sequence tick per iteration) while engaging.
    const auto bus_period = std::chrono::duration<double>(1.0 / rate_hz_);
    const auto engage_period = std::chrono::duration<double>(engage_period_s_);
    auto next = std::chrono::steady_clock::now();
    std::vector<int32_t> counts, vel;
    std::vector<int16_t> cur;
    int failures = 0;
    auto phase = seq_->phase();
    while (running_ && rclcpp::ok()) {
      const auto ph = seq_->phase();
      const bool engaging = !seq_->acceptsVelocity() && ph != arts_dv_tool_driver::Phase::kStopped &&
        ph != arts_dv_tool_driver::Phase::kFailed;
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        engaging ? engage_period : bus_period);
      std::unique_lock<std::mutex> bus_lock(bus_mutex_);
      const bool ok = readAll(counts, vel, cur);
      if (ok) {
        failures = 0;
        rememberCounts(counts);
        const double t = seconds();
        if (engaging && engageReady()) {
          arts_dv_tool_driver::Currents c{};
          for (std::size_t i = 0; i < c.size(); ++i) {c[i] = cur[i];}
          seq_->tick(last_counts_, c, t);
        }
        if (seq_->phase() != phase) {
          phase = seq_->phase();
          if (phase == arts_dv_tool_driver::Phase::kHomed) {
            // The joint zero: the centred counts.
            const auto & h = seq_->homedCounts();
            offsets_.assign(h.begin(), h.end());
            RCLCPP_INFO(get_logger(), "zero set at the homed (centred) disc counts");
          }
          publishPhase();
        }
        sensor_msgs::msg::JointState s;
        s.header.stamp = now();
        s.name = names_;
        for (std::size_t i = 0; i < ids_.size(); ++i) {
          s.position.push_back((counts[i] - offsets_[i]) * kTwoPi / kCountsPerRev);
          s.velocity.push_back(vel[i] * kVelocityUnitRpm * kTwoPi / 60.0);
          s.effort.push_back(cur[i]);
        }
        pub_->publish(s);
        if (!jnames_.empty()) {
          Eigen::VectorXd d(jdisc_index_.size());
          for (std::size_t r = 0; r < jdisc_index_.size(); ++r) {d[r] = s.position[jdisc_index_[r]];}
          q_ = inst_->joints(d);
          sensor_msgs::msg::JointState js;
          js.header.stamp = s.header.stamp;
          js.name = jfull_;
          js.position.assign(q_.data(), q_.data() + q_.size());
          jpub_->publish(js);
        }
      } else if (++failures >= 10) {
        RCLCPP_ERROR(get_logger(), "10 consecutive bus reads failed: stopping (fault)");
        fault_ = true;
        running_ = false;
        break;
      }
      std::vector<double> goal(ids_.size(), 0.0);
      double goal_cap_rpm = max_rpm_;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (have_target_ && (now() - last_target_).seconds() < timeout_s_) {
          if (joint_source_ && q_.size()) {
            std::vector<double> qd = jvel_cmd_;  // each joint's own freshness
            const rclcpp::Time t = now();
            for (std::size_t j = 0; j < qd.size(); ++j) {
              if ((t - jvel_stamp_[j]).seconds() >= timeout_s_) {qd[j] = 0.0;}
            }
            double jaw_cap = jaw_current_cap_;
            const int ji = inst_->jawIndex();
            if (ji >= 0 && have_jaw_cmd_ && (t - jaw_cmd_stamp_).seconds() < timeout_s_) {
              const auto g = arts_dv_tool_driver::InstrumentJoints::jawFromCommand(
                jaw_cmd_, jmin_[ji], jmax_[ji], jaw_grip_cap_, jaw_touch_cap_);
              qd[ji] = jaw_gain_ * (g.target - q_[ji]);
              jaw_cap = g.cap;
            }
            goal = discFromJoint(q_, qd, cur, jaw_cap);
            // Scale all discs together to the instrument cap (keeps the joint direction).
            const double cap = max_rpm_instrument_ * kTwoPi / 60.0;
            double peak = 0.0;
            for (double v : goal) {peak = std::max(peak, std::abs(v));}
            if (peak > cap) {for (double & v : goal) {v *= cap / peak;}}
            goal_cap_rpm = max_rpm_instrument_;
          } else if (!joint_source_) {
            goal = goal_rad_s_;
          }
        }
      }
      if (seq_->acceptsVelocity() && !sendVelocities(goal, goal_cap_rpm)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "goal velocity write failed");
      }
      bus_lock.unlock();
      // No catch-up: after an overrun (e.g. a request's mode-switch pauses) start afresh, so
      // the engagement never sees ticks closer than its period.
      const auto now_tp = std::chrono::steady_clock::now();
      if (next < now_tp) {next = now_tp;}
      std::this_thread::sleep_until(next);
    }
  }

  std::string port_name_;
  int baud_{2000000};
  std::vector<int64_t> zero_counts_;
  int max_zero_drift_{300};
  double jaw_grip_cap_{1000.0};
  double jaw_touch_cap_{150.0};
  double jaw_gain_{10.0};
  double jaw_current_cap_{0.0};
  double jaw_cmd_{0.0};
  rclcpp::Time jaw_cmd_stamp_;
  bool have_jaw_cmd_{false};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr jawsub_;
  double rate_hz_{200.0};
  double timeout_s_{0.5};
  double max_rpm_{30.0};
  double max_rpm_instrument_{75.0};
  int vel_p_{500};
  int vel_i_{1600};
  bool allow_mode_write_{false};
  double engage_period_s_{0.0};
  std::string instrument_;
  std::vector<std::string> jnames_, jfull_;
  std::vector<std::size_t> jdisc_index_;
  std::vector<double> jmin_, jmax_, jvel_cmd_;
  std::vector<rclcpp::Time> jvel_stamp_;
  std::unique_ptr<arts_dv_tool_driver::InstrumentJoints> inst_;
  Eigen::VectorXd q_;
  bool joint_source_{false};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jpub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jsub_;
  std::vector<uint8_t> modes_;
  int eeprom_writes_{0};
  arts_dv_tool_driver::Counts last_counts_{};
  std::unique_ptr<arts_dv_tool_driver::EngageSequence> seq_;
  std::mutex bus_mutex_;
  std::vector<uint8_t> ids_;
  std::vector<std::string> names_;
  std::vector<int32_t> offsets_;
  dynamixel::PortHandler * port_{nullptr};
  dynamixel::PacketHandler * packet_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead> reader_;
  std::unique_ptr<dynamixel::GroupFastSyncRead> fast_reader_;
  bool fast_sync_read_{false};
  std::unique_ptr<dynamixel::GroupSyncWrite> writer_;
  std::unique_ptr<dynamixel::GroupSyncWrite> pos_writer_;
  bool opened_{false};
  bool torque_on_{false};
  std::atomic<bool> running_{false};
  std::atomic<bool> fault_{false};
  std::thread thread_;
  std::mutex mutex_;
  std::vector<double> goal_rad_s_;
  rclcpp::Time last_target_;
  bool have_target_{false};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DiscDrive>();
  if (!node->start()) {
    node->shutdown();
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  while (rclcpp::ok() && !node->faulted()) {
    ex.spin_some(std::chrono::milliseconds(20));
  }
  node->shutdown();
  const int rc = node->faulted() ? 2 : 0;
  rclcpp::shutdown();
  return rc;
}
