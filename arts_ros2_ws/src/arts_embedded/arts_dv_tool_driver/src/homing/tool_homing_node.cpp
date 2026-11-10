// Automatic instrument homing: roll and pitch to the middle of their mechanical limits, then the
// jaws (M5 centered, M2 closed onto it), from motor current alone. The adapter designer's node,
// ported 2026-11-04 with the algorithm and its calibration UNCHANGED (code intake tool_homing,
// NOTES.md lists every difference). ARTS changes, all outside the homing logic:
//   - the adapter is named by its FTDI serial (adapter:=FTBEO5G1), and so are its files
//     (config/homing/adapter_<serial>.txt) and its per-adapter keys in the tool files
//     (<serial>.m5_step_offset): the designer's psm1/psm2 followed the adapter, not the arm;
//   - baud_rate parameter, default 2 Mbps (the servos' setting since 2026-11-04);
//   - on COMPLETE it writes <log_dir>/home_<serial>.yaml (the five home counts) and publishes
//     them on ~/home_counts (latched), which disc_drive takes as its joint zero
//     (zero_from_homing:=true) in place of the visual zero. Starting a homing deletes the file
//     first, so an aborted homing never leaves an older zero behind.
#include "arts_dv_tool_driver/homing/tool_homing_node.hpp"

#include <sstream>

#include <map>
#include <climits>
#include <cstdlib>
#include <filesystem>

// Adapter config file, '#' starts a comment:
//   "M<id> <position>"  start position, one line per motor -- all five (M1..M5) exactly once
//   "<name> <value>"    optional per-adapter jaw parameter (names in kAdapterJawKeys), at most once each
//   "port <device>"     serial port of THIS adapter's USB converter, e.g. /dev/serial/by-id/usb-FTDI_..._FTBEQBRA-...
//                       (by-id paths follow the converter; /dev/ttyUSB<n> numbers change with plug-in order)
static const char* const kAdapterJawKeys[] = {"m5_step_offset", "m2_step_offset"};

static bool load_adapter_start_positions(const std::string& path, int out[5], std::map<std::string, double>& extra,
                                         std::string& port, std::string& err)
{
    port.clear();
    std::ifstream f(path);
    if (!f) { err = "cannot open file"; return false; }
    bool seen[5] = {false, false, false, false, false};
    std::string line;
    int lineno = 0;
    while (std::getline(f, line)) {
        ++lineno;
        line = line.substr(0, line.find('#'));
        std::istringstream ss(line);
        std::string key;
        if (!(ss >> key)) continue;                      // blank / comment line
        if (key == "port") {
            std::string dev, rest;
            if (!(ss >> dev) || (ss >> rest)) {
                err = "line " + std::to_string(lineno) + " is not 'port <device path>': \"" + line + "\"";
                return false;
            }
            if (!port.empty()) { err = "port given twice"; return false; }
            port = dev;
            continue;
        }
        bool is_jaw_key = false;
        for (const char* k : kAdapterJawKeys) is_jaw_key = is_jaw_key || key == k;
        if (is_jaw_key) {
            err = "line " + std::to_string(lineno) + ": " + key + " moved to the TOOL file (config/<tool>.txt, as "
                  "<adapter>." + key + ") -- step offsets depend on the tool as well as the adapter";
            return false;
        }
        if (false) {
            double dv;
            std::string rest;
            if (!(ss >> dv) || (ss >> rest)) {
                err = "line " + std::to_string(lineno) + " is not '" + key + " <number>': \"" + line + "\"";
                return false;
            }
            if (extra.count(key)) { err = key + " given twice"; return false; }
            if (dv < -500 || dv > 500) { err = key + " = " + std::to_string(dv) + " is outside -500..500"; return false; }
            extra[key] = dv;
            continue;
        }
        long value;
        std::string extra_tok;
        if (key.size() != 2 || (key[0] != 'M' && key[0] != 'm') || key[1] < '1' || key[1] > '5' ||
            !(ss >> value) || (ss >> extra_tok)) {
            err = "line " + std::to_string(lineno) + " is not 'M<1-5> <position>' or a known jaw setting: \"" + line + "\"";
            return false;
        }
        int i = key[1] - '1';
        if (seen[i]) { err = "M" + std::to_string(i + 1) + " given twice"; return false; }
        seen[i] = true;
        out[i] = static_cast<int>(value);
    }
    for (int i = 0; i < 5; ++i)
        if (!seen[i]) { err = "M" + std::to_string(i + 1) + " missing"; return false; }
    return true;
}


// Tool file: "<key> <value>" per line, '#' starts a comment. Keys: jaw.*, rp.*, step.*, <adapter>.<offset>.
bool ToolHomingNode::load_tool_file(const std::string& path, std::string& err)
{
    std::ifstream f(path);
    if (!f) { err = "cannot open file"; return false; }
    std::string line;
    int lineno = 0;
    while (std::getline(f, line)) {
        ++lineno;
        line = line.substr(0, line.find('#'));
        std::istringstream ss(line);
        std::string key, val, rest;
        if (!(ss >> key)) continue;
        if (!(ss >> val) || (ss >> rest)) {
            err = "line " + std::to_string(lineno) + " is not '<key> <value>': \"" + line + "\"";
            return false;
        }
        if (tool_cfg_.count(key)) { err = key + " given twice"; return false; }
        tool_cfg_[key] = val;
    }
    return true;
}

template <typename T> T ToolHomingNode::tool_value(const std::string& key, T def)
{
    auto it = tool_cfg_.find(key);
    if (it == tool_cfg_.end()) { tool_missing_.push_back(key); return def; }
    tool_used_.insert(key);
    const std::string& v = it->second;
    try {
        size_t used = 0;
        if constexpr (std::is_same_v<T, int>) {
            long x = std::stol(v, &used);
            if (used == v.size()) return static_cast<int>(x);
        } else if constexpr (std::is_same_v<T, double>) {
            double x = std::stod(v, &used);
            if (used == v.size()) return x;
        } else if constexpr (std::is_same_v<T, bool>) {
            if (v == "true" || v == "1") return true;
            if (v == "false" || v == "0") return false;
        } else {
            return v;
        }
    } catch (const std::exception&) {}
    tool_bad_.push_back(key + " = '" + v + "'");
    return def;
}

ToolHomingNode::ToolHomingNode() : Node("tool_homing_node"), current_state_(SetupState::JOY_TELEOP)
{
    this->declare_parameter("tool_serial_port", "/dev/ttyUSB0");
    std::string port_name = this->get_parameter("tool_serial_port").as_string();
    config_dir_ = this->declare_parameter("adapter_config_dir", std::string(ADAPTER_CONFIG_DIR));
    // Tool-specific values (jaw.*, rp.*, step.*, per-adapter step offsets) come from config/<tool>.txt.
    tool_name_ = this->declare_parameter("tool", std::string("forceps"));
    // calibrate:=true -- learn a tool: slow roll/pitch/jaw homing + verify pauses (joystick), then the measured
    // values are written into config/<tool>.txt after the operator confirms (A). Only these tool files:
    calibrate_ = this->declare_parameter("calibrate", false);
    // slow:=true -- normal homing at calibration speed (no fast sections; M5 back to the middle at the sweep
    // speed), without verify pauses or saving. The GUI uses it for tools still being tried out.
    slow_ = this->declare_parameter("slow", false) || calibrate_;
    // slow_jaws:=true -- only the jaws slow (roll/pitch use the tool file's speeds)
    slow_jaws_ = this->declare_parameter("slow_jaws", false) || slow_;
    if (calibrate_) {
        static const std::set<std::string> kCalTools = {"forceps", "scissors", "hook", "stapler", "needledriver"};
        if (!kCalTools.count(tool_name_)) {
            RCLCPP_ERROR(this->get_logger(), "Calibration is limited to forceps, scissors, hook, stapler, needledriver "
                         "(tool:=%s). Not moving any motor.", tool_name_.c_str());
            rclcpp::shutdown();
            return;
        }
    }
    {
        const std::string tool_path = config_dir_ + "/" + tool_name_ + ".txt";
        std::string err;
        if (!load_tool_file(tool_path, err)) {
            RCLCPP_ERROR(this->get_logger(), "Tool file %s: %s. Not moving any motor.", tool_path.c_str(), err.c_str());
            rclcpp::shutdown();
            return;
        }
        // homing.jaw_mode: "contact" = jaw sweep + close-onto-contact plan (forceps, scissors);
        // "limits" = one jaw with hard stops, homed like pitch (hook). Only that mode's keys are required.
        jaw_mode_ = tool_value("homing.jaw_mode", std::string("contact"));
        if (jaw_mode_ != "contact" && jaw_mode_ != "limits") {
            RCLCPP_ERROR(this->get_logger(), "Tool file %s: homing.jaw_mode must be 'contact' or 'limits', not '%s'",
                         tool_path.c_str(), jaw_mode_.c_str());
            config_error_ = true;
        }
        load_jaw_params();
        load_rp_params();
        if (jaw_mode_ == "limits") load_lim_params();
        std::vector<std::string> unknown;
        for (const auto& kv : tool_cfg_) {
            const std::string& k = kv.first;
            // "<adapter>.<name>" per-adapter jaw values (checked when that adapter is used)
            bool per_adapter = false;
            for (const char* sfx : {".m5_step_offset", ".m2_step_offset", ".m5_engaged_start", ".m2_contact_offset",
                                    ".pitch_limit1_angle", ".m5_center_angle", ".m2_center_angle"}) {
                const std::string sx = sfx;
                if (k.size() > sx.size() && k.compare(k.size() - sx.size(), sx.size(), sx) == 0 && k.rfind("jaw.", 0) != 0)
                    per_adapter = true;
            }
            if (!tool_used_.count(k) && !per_adapter) unknown.push_back(k);
        }
        auto join = [](const std::vector<std::string>& v) { std::string r; for (auto& x : v) r += (r.empty() ? "" : ", ") + x; return r; };
        if (!tool_missing_.empty())
            RCLCPP_ERROR(this->get_logger(), "Tool file %s is missing: %s", tool_path.c_str(), join(tool_missing_).c_str());
        if (!tool_bad_.empty())
            RCLCPP_ERROR(this->get_logger(), "Tool file %s has bad values: %s", tool_path.c_str(), join(tool_bad_).c_str());
        if (!unknown.empty())
            RCLCPP_ERROR(this->get_logger(), "Tool file %s has unknown keys (typo?): %s", tool_path.c_str(), join(unknown).c_str());
        if (!tool_missing_.empty() || !tool_bad_.empty() || !unknown.empty() || config_error_) {
            RCLCPP_ERROR(this->get_logger(), "Tool '%s' config is not usable. Not moving any motor.", tool_name_.c_str());
            rclcpp::shutdown();
            return;
        }
        if (slow_) {
            // slow everywhere: no fast sections (calibration: the ranges are what we are measuring)
            rp_.roll_expected_range = 0;
            rp_.pitch_expected_range = 0;
            lim_.expected_range = 0;
            if (!calibrate_)
                RCLCPP_WARN(this->get_logger(), "SLOW MODE: no fast sections (expected ranges ignored), M5 back to "
                            "the middle at the sweep speed.");
        } else if (slow_jaws_) {
            RCLCPP_WARN(this->get_logger(), "SLOW JAWS: M5 back to the middle at the sweep speed (roll/pitch normal).");
        }
        if (calibrate_) {
            // the operator confirms each contact jaw's true center (joystick sticks + A)
            if (jaw_mode_ == "contact") jp_.verify = true;
            RCLCPP_WARN(this->get_logger(), "CALIBRATION MODE for '%s': roll/pitch/jaw slow (expected ranges ignored)%s. "
                        "At the end: A = save the measured values to %s/%s.txt, B = discard.", tool_name_.c_str(),
                        jaw_mode_ == "contact" ? ", verify pause after each jaw (needs the joystick)" : "",
                        config_dir_.c_str(), tool_name_.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "Tool '%s' (%s): %zu values loaded, jaw mode '%s'", tool_name_.c_str(),
                    tool_path.c_str(), tool_cfg_.size(), jaw_mode_.c_str());
        if (jaw_mode_ == "limits")
            RCLCPP_INFO(this->get_logger(), "Jaw limit axis: M%d, first sweep %s, current %d, expected range %.0f%s",
                        lim_.motor, lim_.direction > 0 ? "+" : "-", lim_.sweep_current, lim_.expected_range,
                        lim_.expected_range > 0 ? "" : " (never fast)");
    }
  
    // Start positions (before the tool goes on). Per adapter: the drive pins must meet the right holes.
    // Read from <adapter_config_dir>/<adapter>_adapter.txt (e.g. config/psm1_adapter.txt).
    int starting_locations[5] = {0, 0, 0, 0, 0};
    {
        std::string adapter = this->declare_parameter("adapter", std::string(""));
        adapter_ = adapter;
        adapter_name_ = adapter;
        std::string path = config_dir_ + "/adapter_" + adapter + ".txt";
        std::string err;
        std::map<std::string, double> adapter_jaw;
        std::string adapter_port;
        if (!load_adapter_start_positions(path, starting_locations, adapter_jaw, adapter_port, err)) {
            RCLCPP_ERROR(this->get_logger(), "Adapter config %s: %s. Not moving any motor; fix the file and restart.",
                         path.c_str(), err.c_str());
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Adapter '%s' (%s) start positions: M1 %d, M2 %d, M3 %d, M4 %d, M5 %d",
                    adapter.c_str(), path.c_str(), starting_locations[0], starting_locations[1],
                    starting_locations[2], starting_locations[3], starting_locations[4]);
        // jaw values for THIS tool on THIS adapter ("<adapter>.<name>" in the tool file) override the tool-wide jaw.*:
        // step offsets, and (since the start positions differ per adapter) m5_engaged_start / m2_contact_offset
        bool have5 = tool_cfg_.count(adapter + ".m5_step_offset") > 0, have2 = tool_cfg_.count(adapter + ".m2_step_offset") > 0;
        bool have_es = tool_cfg_.count(adapter + ".m5_engaged_start") > 0;
        bool have_co = tool_cfg_.count(adapter + ".m2_contact_offset") > 0;
        if (have5) jp_.m5_step_offset = tool_value(adapter + ".m5_step_offset", jp_.m5_step_offset);
        if (have2) jp_.m2_step_offset = tool_value(adapter + ".m2_step_offset", jp_.m2_step_offset);
        if (have_es) jp_.m5_engaged_start = tool_value(adapter + ".m5_engaged_start", jp_.m5_engaged_start);
        if (have_co) jp_.m2_contact_offset = tool_value(adapter + ".m2_contact_offset", jp_.m2_contact_offset);
        if (tool_cfg_.count(adapter + ".pitch_limit1_angle")) {
            pitch_angle_expect_ = tool_value(adapter + ".pitch_limit1_angle", -1.0);
            if (pitch_angle_expect_ < 0 || pitch_angle_expect_ >= 4096) {
                RCLCPP_ERROR(this->get_logger(), "Tool file: %s.pitch_limit1_angle must be 0..4095.", adapter.c_str());
                tool_bad_.push_back(adapter + ".pitch_limit1_angle");
            }
        }
        if (tool_cfg_.count(adapter + ".m5_center_angle")) {
            m5_center_angle_ = tool_value(adapter + ".m5_center_angle", -1.0);
            if (m5_center_angle_ < 0 || m5_center_angle_ >= 4096) {
                RCLCPP_ERROR(this->get_logger(), "Tool file: %s.m5_center_angle must be 0..4095.", adapter.c_str());
                tool_bad_.push_back(adapter + ".m5_center_angle");
            }
        }
        if (tool_cfg_.count(adapter + ".m2_center_angle")) {
            m2_center_angle_ = tool_value(adapter + ".m2_center_angle", -1.0);
            if (m2_center_angle_ < 0 || m2_center_angle_ >= 4096) {
                RCLCPP_ERROR(this->get_logger(), "Tool file: %s.m2_center_angle must be 0..4095.", adapter.c_str());
                tool_bad_.push_back(adapter + ".m2_center_angle");
            }
        }
        if (jaw_mode_ == "contact")
            RCLCPP_INFO(this->get_logger(), "M2 center angle on '%s': %s", adapter.c_str(), m2_center_angle_ >= 0
                        ? ("after the contact M2 snaps to motor angle " + std::to_string(static_cast<int>(m2_center_angle_))).c_str()
                        : "none (M2 placed by the contact)");
        if (jaw_mode_ == "contact")
            RCLCPP_INFO(this->get_logger(), "M5 center angle on '%s': %s", adapter.c_str(), m5_center_angle_ >= 0
                        ? ("step mode snaps M5 to motor angle " + std::to_string(static_cast<int>(m5_center_angle_))).c_str()
                        : "none (step mode uses step - m5_step_offset)");
        if (rp_.pitch_angle_tol > 0)
            RCLCPP_INFO(this->get_logger(), "Pitch disc-angle check on '%s': %s", adapter.c_str(),
                        pitch_angle_expect_ >= 0 ? ("limit 1 at angle " + std::to_string(static_cast<int>(pitch_angle_expect_)) +
                        " +- " + std::to_string(static_cast<int>(rp_.pitch_angle_tol))).c_str()
                        : "no <adapter>.pitch_limit1_angle yet (calibration measures it): check OFF");
        if (!tool_bad_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Tool file: bad %s offset value. Not moving any motor.", adapter.c_str());
            rclcpp::shutdown();
            return;
        }
        if (jaw_mode_ == "contact")
        RCLCPP_INFO(this->get_logger(), "Tool '%s' on adapter '%s' jaw step offsets: M5 %.0f%s, M2 %.0f%s",
                    tool_name_.c_str(), adapter.c_str(),
                    jp_.m5_step_offset, have5 ? "" : " (tool default: no adapter entry)",
                    jp_.m2_step_offset, have2 ? "" : " (tool default: no adapter entry)");
        if (jaw_mode_ == "contact")
        RCLCPP_INFO(this->get_logger(), "Tool '%s' on adapter '%s': m5_engaged_start %.0f%s, m2_contact_offset %.0f%s",
                    tool_name_.c_str(), adapter.c_str(),
                    jp_.m5_engaged_start, have_es ? "" : " (tool default: no adapter entry)",
                    jp_.m2_contact_offset, have_co ? "" : " (tool default: no adapter entry)");
        // the adapter file's port wins: it names THIS adapter's converter (tool_serial_port may be stale)
        if (!adapter_port.empty()) {
            if (!std::filesystem::exists(adapter_port)) {
                RCLCPP_ERROR(this->get_logger(), "Adapter '%s' port %s is not connected. Not moving any motor.",
                             adapter.c_str(), adapter_port.c_str());
                rclcpp::shutdown();
                return;
            }
            if (adapter_port != port_name)
                RCLCPP_INFO(this->get_logger(), "Adapter '%s': using its port %s (not tool_serial_port %s)",
                            adapter.c_str(), adapter_port.c_str(), port_name.c_str());
            port_name = adapter_port;
        } else {
            RCLCPP_WARN(this->get_logger(), "Adapter '%s' has no 'port' line: using tool_serial_port %s -- this may be "
                        "the OTHER adapter if both are plugged in.", adapter.c_str(), port_name.c_str());
        }
        char resolved[PATH_MAX];
        RCLCPP_INFO(this->get_logger(), "Adapter '%s' serial port: %s%s%s", adapter.c_str(), port_name.c_str(),
                    realpath(port_name.c_str(), resolved) ? " -> " : "", realpath(port_name.c_str(), resolved) ? resolved : "");
    }
    dxl_interface_ = std::make_unique<arts_homing::DxlInterface>();
    // std::string port_name = "/dev/ttyUSB0"; 
    // Dynamixels reconfigured to 2 Mbps (4 Nov 2026; was 1 Mbps). -p baud_rate:=1000000 for a motor set not yet changed.
    const int baud_rate = this->declare_parameter("baud_rate", 2000000);
    RCLCPP_INFO(this->get_logger(), "Opening %s at %d baud", port_name.c_str(), baud_rate);

    if (!dxl_interface_->OpenPort(port_name, baud_rate, 2.0)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open port: %s", port_name.c_str());
        rclcpp::shutdown();
        return;
    }

    int motor_ids[5] = {1,2,3,4,5}; 
    if (!dxl_interface_->ConfigureDevices(5, motor_ids)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure device IDs.");
        return;
    }
    
    dxl_interface_->DisableTorque();
    dxl_interface_->SetOperatingMode(OPMODE_EXTPOS_CTRL);
    dxl_interface_->EnableTorque();
    RCLCPP_INFO(this->get_logger(), "Position control activate, torque enabled");
    
    dxl_interface_->SetProfileVelocity(150);
    
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "homing_status", rclcpp::QoS(1).transient_local().reliable());
    home_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(
        "~/home_counts", rclcpp::QoS(1).transient_local().reliable());
    publish_status("MOVING_TO_START");
    dxl_interface_->SetBulkPos(starting_locations);
    RCLCPP_INFO(this->get_logger(), "Moving to starting locations (waits until all motors arrive and stop)");
    {
        // Wait until every motor is within 20 ticks of its target and not moving, for 0.1 s.
        // Safety timeout 10 s: then report which motor did not arrive and continue.
        const auto t_start = std::chrono::steady_clock::now();
        int settled = 0;
        int32_t p[5] = {0}, v[5] = {0};
        int16_t c[5] = {0};
        bool timed_out = false;
        while (rclcpp::ok()) {
            bool arrived = false;
            if (dxl_interface_->SyncReadState(p, v, c)) {
                arrived = true;
                for (int i = 0; i < 5; ++i)
                    if (std::abs(p[i] - starting_locations[i]) > 20 || std::abs(v[i]) > 2) arrived = false;
            }
            settled = arrived ? settled + 1 : 0;
            if (settled >= 10) break;                       // 10 polls x 10 ms
            double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
            if (waited > 10.0) { timed_out = true; break; }
            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        double waited = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
        if (timed_out) {
            for (int i = 0; i < 5; ++i)
                if (std::abs(p[i] - starting_locations[i]) > 20)
                    RCLCPP_WARN(this->get_logger(), "Motor %d did not reach its start position: at %d, target %d",
                                i + 1, p[i], starting_locations[i]);
            RCLCPP_WARN(this->get_logger(), "Start move timed out after %.1f s; continuing.", waited);
        } else {
            RCLCPP_INFO(this->get_logger(), "Start positions reached in %.1f s", waited);
        }
    }
    dxl_interface_->GetBulkPos(starting_locations_);
    
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ToolHomingNode::joy_callback, this, _1));
    gui_a_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "gui_a", 10, [this](const std_msgs::msg::Empty::SharedPtr) {
            if (current_state_ == SetupState::ESTOP) return;
            RCLCPP_INFO(this->get_logger(), "GUI: A");
            handle_a_press();
        });
    // The GUI can stand in for the joystick: its heartbeat satisfies the jaw deadman (stops if the GUI dies),
    // and its STOP button does what joystick Y does.
    gui_heartbeat_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "gui_heartbeat", 10, [this](const std_msgs::msg::Empty::SharedPtr) { last_gui_heartbeat_ = now_s(); });
    gui_estop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "gui_estop", 10, [this](const std_msgs::msg::Empty::SharedPtr) { emergency_stop("GUI STOP"); });
        
    current_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("motor_currents",10);
    event_pub_ = this->create_publisher<std_msgs::msg::Int8>("motor_events", 10);
    pos_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("motor_positions", 10);
    limit_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("motor_limits", 10);
        
    control_hz_ = this->declare_parameter("control_hz", 333.0);
    if (control_hz_ < 20.0 || control_hz_ > 500.0) control_hz_ = 333.0;
    timer_ = this->create_wall_timer(
        std::chrono::microseconds(static_cast<int64_t>(1e6 / control_hz_)),
        std::bind(&ToolHomingNode::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Motor I/O loop requested at %.0f Hz (homing logic runs at 50 Hz on averaged data)",
                control_hz_);
    
    current_state_ = SetupState::WAIT_FOR_TOOL;
    publish_status("WAIT_FOR_TOOL");
    RCLCPP_INFO(this->get_logger(), "Put the Xi tool on the adapter, and press the 'A' button on the joy controller");
}

void ToolHomingNode::publish_status(const std::string& s)
{
    if (!status_pub_) return;
    std_msgs::msg::String msg;
    msg.data = s;
    status_pub_->publish(msg);
}

ToolHomingNode::~ToolHomingNode()
{
    if (!dxl_interface_) return;   // stopped before the port was opened (bad config / port not connected)
    dxl_interface_->DisableTorque();
    dxl_interface_->ClosePort();
}

void ToolHomingNode::publish_limit_state(int axis_id, int state) 
{
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(axis_id);
    msg.data.push_back(state);
    limit_pub_->publish(msg);
}

void ToolHomingNode::handle_a_press()
{
    if (current_state_ == SetupState::JAW_VERIFY) { verify_confirm_ = true; return; }
    if (current_state_ == SetupState::CAL_REVIEW) { cal_save(); return; }
    // Jaw homing: A during an automatic jaw sweep/close = stop it now (handled in the timer).
    if (current_state_ == SetupState::JAW_SWEEP || current_state_ == SetupState::JAW_CLOSE) {
        operator_stop_ = true;
        return;
    }
    if (current_state_ != SetupState::WAIT_FOR_TOOL) return;
    // Jaw homing needs a live joystick (deadman, jaw.joystick_timeout). Check it BEFORE starting, so a missing
    // controller doesn't abort the run halfway (the GUI's Start works without touching the joystick).
    if (!joy_fresh()) {
        RCLCPP_ERROR(this->get_logger(), "No joystick and no GUI heartbeat: jaw homing needs one of them as a "
                     "deadman. Run the GUI (or connect the controller), then press Start/A again.");
        publish_status("NO_JOYSTICK");
        publish_status("WAIT_FOR_TOOL");
        return;
    }
    // Combined roll engagement + homing: sweep Motor 1 directly to its first limit.
    RCLCPP_INFO(this->get_logger(), "A pressed: combined roll engagement and homing...");
    {
        std::error_code ec;   // no older zero may survive a homing that does not complete
        std::filesystem::remove(home_file(), ec);
    }
    publish_status("HOMING");
    t_homing_start_ = now_s();
    soft_limit_axes_.clear();
    push_limit_axes_.clear();
    limit_since_ = -1;
    center_ok_ = 0;
    roll_slowed_ = false;
    current_state_ = SetupState::LIMITS;
    joint_limit_motor_ = 0;  // Motor 1 (roll)
    direction_ = 1;          // Same positive direction as the old engagement spin
    homing_step_ = 0;
    stall_count_ = 0;
    phase_initialized_ = false;
    motors_locked_in_ = 0;

    dxl_interface_->DisableTorque();
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    dxl_interface_->SetOperatingMode(OPMODE_POSCURR_CTRL);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    dxl_interface_->EnableTorque();

    dxl_interface_->SetProfileVelocity(40);  // Old engagement speed
    for (int i = 1; i <= 5; i++) {
        dxl_interface_->SetGoalCurrent(i, 100);  // Existing homing current
        starting_locations_[i-1] = dxl_interface_->ReadPosition(i);
    }
    prev_pos_ = dxl_interface_->ReadPosition(1);
}

void ToolHomingNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->buttons.empty()) return;
    last_joy_time_ = now_s();   // jaw homing deadman: automatic jaw motion needs a live joystick

    bool button_a_pressed = (msg->buttons[0] == 1);
    bool button_b_pressed = (msg->buttons[1] == 1);
    bool button_x_pressed = (msg->buttons[2] == 1);
    bool button_y_pressed = (msg->buttons[3] == 1);

    if (button_y_pressed && !prev_button_y_) emergency_stop("joystick Y");
    
    if (current_state_ == SetupState::ESTOP) {
        prev_button_y_ = button_y_pressed;
        prev_button_a_ = button_a_pressed;
        prev_button_b_ = button_b_pressed;
        prev_button_x_ = button_x_pressed;
        return; 
    }

    // Verify mode: remember the jaw sticks; A (sticks released) confirms the true center.
    if (static_cast<int>(msg->axes.size()) > std::max(jp_.m2_axis, jp_.m5_axis)) {
        axis_m2_ = msg->axes[jp_.m2_axis];
        axis_m5_ = msg->axes[jp_.m5_axis];
    }
    // A (joystick or GUI): verify confirm / stop an automatic jaw move / start homing
    if (button_a_pressed && !prev_button_a_) handle_a_press();
    if (button_b_pressed && !prev_button_b_ && current_state_ == SetupState::CAL_REVIEW) cal_discard();

    if (current_state_ == SetupState::JOY_TELEOP || current_state_ == SetupState::KINEMATICS)
    {
        if (button_x_pressed && !prev_button_x_)
        {
            RCLCPP_INFO(this->get_logger(), "X button pressed. Returning tool to home position...");
            current_state_ = SetupState::GO_START_POS;
            
            dxl_interface_->DisableTorque();
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            dxl_interface_->SetOperatingMode(OPMODE_POSCURR_CTRL);
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            dxl_interface_->EnableTorque();
            
            dxl_interface_->SetProfileVelocity(30); 
            
            for (int i = 1; i <= 5; i++) {
                dxl_interface_->SetGoalCurrent(i, 200); 
            }
            
            dxl_interface_->SetBulkPos(starting_locations_);
        }
    
        if (button_b_pressed && !prev_button_b_)
        {
            if (current_state_ == SetupState::JOY_TELEOP) {
                current_state_ = SetupState::KINEMATICS;
                RCLCPP_INFO(this->get_logger(), "Mode switched to tool tip kinematics (coupled motors).");
            } else {
                current_state_ = SetupState::JOY_TELEOP;
                RCLCPP_INFO(this->get_logger(), "Mode switched to joystick teleop (individual motors).");
            }
        }

        if (current_state_ == SetupState::JOY_TELEOP || current_state_ == SetupState::KINEMATICS)
        {
            int target_velocities[5] = {0, 0, 0, 0, 0};
            float max_speed = 100.0;

            if (current_state_ == SetupState::JOY_TELEOP)
            {
                float axis_inputs[5] = {
                    msg->axes[0], 
                    msg->axes[1], 
                    msg->axes[3], 
                    msg->axes[4], 
                    msg->axes[6]  
                };

                for (int i = 0; i < 5; ++i) {
                    target_velocities[i] = axis_inputs[i] * max_speed; 
                }
                target_velocities[2] = 0;      
            }
            else if (current_state_ == SetupState::KINEMATICS)
            {
                float joy_roll  = (msg->axes[0])*20.0/180.0*M_PI;     
                float wrist_bend = (msg->axes[3])*20.0/180.0*M_PI;    
                float grasper_bend = (msg->axes[6])*20.0/180.0*M_PI;  
                float grasper_open = (msg->axes[7])*20.0/180.0*M_PI;  

                Eigen::Matrix4f couplingMat;
                couplingMat(0,0) = 0.90;     couplingMat(0,1) = 0.00;         couplingMat(0,2) = 0.00;         couplingMat(0,3) = 0.00;         
                couplingMat(1,0) = 0.00;     couplingMat(1,1) = 0.59;         couplingMat(1,2) = 0.85;         couplingMat(1,3) = -0.85;         
                couplingMat(2,0) = 0.00;     couplingMat(2,1) = 0.94;         couplingMat(2,2) = 0.00;         couplingMat(2,3) = 0.00;         
                couplingMat(3,0) = 0.00;     couplingMat(3,1) = -0.59;        couplingMat(3,2) = -0.85;        couplingMat(3,3) = -0.85;         

                Eigen::Vector4f jVel;
                jVel[0] = joy_roll;         jVel[1] = wrist_bend;       jVel[2] = grasper_bend;     jVel[3] = grasper_open;

                Eigen::Vector4f diskVel;
                diskVel = couplingMat * jVel;

                target_velocities[0] = -diskVel[0]*RAD_PER_SEC_TO_RPM*RPM_TO_CMD;
                target_velocities[1] = -diskVel[1]*RAD_PER_SEC_TO_RPM*RPM_TO_CMD;
                target_velocities[2] = 0;              
                target_velocities[3] = -diskVel[2]*RAD_PER_SEC_TO_RPM*RPM_TO_CMD;
                target_velocities[4] = -diskVel[3]*RAD_PER_SEC_TO_RPM*RPM_TO_CMD;
            }

            dxl_interface_->SetBulkVel(target_velocities);
        }
    }
    
    prev_button_a_ = button_a_pressed;
    prev_button_b_ = button_b_pressed;
    prev_button_x_ = button_x_pressed;
    prev_button_y_ = button_y_pressed;
}
    	
void ToolHomingNode::timer_callback()
{
    if (current_state_ == SetupState::ESTOP) {
        return; 
    }
    const double now = now_s();
    const bool rp_state = current_state_ == SetupState::LIMITS;
    const bool jaw_state = current_state_ == SetupState::JAW_SWEEP || current_state_ == SetupState::JAW_GOTO
                           || current_state_ == SetupState::JAW_CLOSE;

    // ---- fast I/O: ONE sync read of all motors every tick (333 Hz when the bus allows) ----
    bool ok = read_raw();
    if (!ok) {
        ++read_fail_;
        if (read_fail_ >= 3 && rp_state) {
            RCLCPP_ERROR(get_logger(), "3 motor reads failed during roll/pitch homing: stopping (ESTOP).");
            current_state_ = SetupState::ESTOP;
            publish_status("ESTOP");
            dxl_interface_->DisableTorque();
            return;
        }
        if (read_fail_ >= 3 && jaw_state) { jaw_abort("3 motor reads failed in a row"); return; }
    } else {
        read_fail_ = 0;
        if (io_t0_ == 0) io_t0_ = now;
        io_count_ += 1;
        if (now - io_t0_ >= 5.0) {     // report the real I/O rate every 5 s while homing
            if (rp_state || jaw_state)
                RCLCPP_INFO(get_logger(), "Motor I/O rate: %.0f Hz", io_count_ / (now - io_t0_));
            io_t0_ = now; io_count_ = 0;
        }
    }
    // (goals are written by the homing logic whenever they change; re-sending identical goals every
    //  tick would cost ~2 ms of bus time and cap the loop near 250 Hz)

    // ---- homing logic at 50 Hz on the averaged samples (what the detectors were calibrated on) ----
    if (rp_state) {
        if (now - rp_last_tick_ >= 0.019 && acc_n_ > 0) {
            rp_last_tick_ = now;
            take_average();
            rp_tick();
        }
    }
    else if (jaw_state) {
        if (now - last_jaw_tick_ >= 0.019 && acc_n_ > 0) {
            last_jaw_tick_ = now;
            take_average();
            jaw_tick();
        }
    }
    else if (current_state_ == SetupState::JAW_VERIFY) {
        if (now - last_jaw_tick_ >= 0.019 && acc_n_ > 0) {
            double dt = std::min(0.1, now - last_jaw_tick_);
            last_jaw_tick_ = now;
            take_average();
            jaw_tick_verify(dt);
        }
    }
    else if (current_state_ == SetupState::GO_START_POS)
    {
        bool all_reached = true;
        for (int i = 1; i <= 5; i++) {
            if (std::abs(raw_pos_[i] - starting_locations_[i-1]) > 25) all_reached = false;
        }

        if (all_reached) {
            RCLCPP_INFO(this->get_logger(), "Home position reached. Restoring JOY_TELEOP.");
            current_state_ = SetupState::JOY_TELEOP;

            dxl_interface_->DisableTorque();
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            dxl_interface_->SetOperatingMode(OPMODE_VELOCITY_CTRL); 
            rclcpp::sleep_for(std::chrono::milliseconds(50));
            dxl_interface_->EnableTorque();
            
            for (int i = 1; i <= 5; i++) {
                dxl_interface_->SetGoalCurrent(i, 1193); 
            }
        }
    }

    if (!raw_valid_) return;
    std_msgs::msg::Int16MultiArray current_msg;
    std_msgs::msg::Int32MultiArray pos_msg;
    for (int i = 1; i <= 5; i++) {
        current_msg.data.push_back(raw_cur_[i]);
        pos_msg.data.push_back(raw_pos_[i]);
    }
    current_pub_->publish(current_msg);
    pos_pub_->publish(pos_msg);
}
    

// =====================================================================================
//  JAW HOMING (current only). Ported from jaw_autohome_test.py, one jaw at a time:
//    0) M5 opening sweep -> stops by itself ('step' / 'engaged' / safety)
//    1) M5 back to the middle: step estimate, or (engaged start) start - m5_engaged_start
//    2) M2 opening sweep
//    3) M2 closes until it touches M5 and parks there (checked against its step estimate)
//    4) done. Any unexpected result -> JAW_ABORTED (motors hold).
// =====================================================================================
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

double ToolHomingNode::now_s() const
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Deadman for automatic jaw motion: a live joystick OR a live GUI heartbeat (either one is enough).
bool ToolHomingNode::joy_fresh() const
{
    const double t = now_s();
    return t - last_joy_time_ <= jp_.joystick_timeout || t - last_gui_heartbeat_ <= jp_.joystick_timeout;
}

// Jaw motors free (torque off) / held again. Re-enabling: goal := present position FIRST, so the jaw does not
// jump back to its old goal when torque comes on.
void ToolHomingNode::set_jaws_free(bool free)
{
    if (free) {
        bool ok = dxl_interface_->DisableSingleTorque(2) && dxl_interface_->DisableSingleTorque(5);
        if (ok) RCLCPP_INFO(get_logger(), "Jaw motors M2, M5 free (torque off) during pitch homing.");
        else RCLCPP_WARN(get_logger(), "Could not free the jaw motors; pitch homes with them held.");
        return;
    }
    int32_t p2 = dxl_interface_->ReadPosition(2), p5 = dxl_interface_->ReadPosition(5);
    starting_locations_[1] = p2;
    starting_locations_[4] = p5;
    dxl_interface_->SetBulkPos(starting_locations_);
    bool ok = dxl_interface_->EnableSingleTorque(2) && dxl_interface_->EnableSingleTorque(5);
    if (!ok) {
        RCLCPP_ERROR(get_logger(), "Could not re-enable torque on the jaw motors after pitch homing.");
        emergency_stop("jaw torque re-enable failed");
        return;
    }
    RCLCPP_INFO(get_logger(), "Jaw motors M2, M5 held again where pitch left them: M2 %d, M5 %d.", p2, p5);
}

void ToolHomingNode::emergency_stop(const char* source)
{
    if (current_state_ == SetupState::ESTOP) return;
    RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP TRIGGERED (%s)! Stopping all motors.", source);
    current_state_ = SetupState::ESTOP;
    publish_status("ESTOP");
    int zero_vel[5] = {0, 0, 0, 0, 0};
    dxl_interface_->SetBulkVel(zero_vel);
    dxl_interface_->DisableTorque();
}

void ToolHomingNode::load_jaw_params()
{
    // operator / UI settings are not tool-specific: plain ROS parameters with code defaults
    static const std::set<std::string> kNotTool = {"log_dir", "verify", "jog_speed", "m2_axis", "m5_axis",
                                                   "deadband", "joystick_timeout"};
    // jaw mode "limits" (hook): the contact plan never runs, so its jaw.* / step.* keys are NOT read from the
    // tool file (a jaw.* line there is then an unknown key -> refused). The code defaults below stay unused.
    const bool contact = jaw_mode_ == "contact";
    auto P = [this, contact](const std::string& n, auto def) {
        auto v = (kNotTool.count(n) || !contact) ? def : tool_value("jaw." + n, def);
        return this->declare_parameter("jaw." + n, v);
    };
    jp_.sweep_current = P("sweep_current", jp_.sweep_current);
    jp_.sweep_profile_velocity = P("sweep_profile_velocity", jp_.sweep_profile_velocity);
    jp_.move_profile_velocity = P("move_profile_velocity", jp_.move_profile_velocity);
    jp_.move_fast_profile = P("move_fast_profile", jp_.move_fast_profile);
    jp_.move_fast_current = P("move_fast_current", jp_.move_fast_current);
    jp_.move_slow_margin = P("move_slow_margin", jp_.move_slow_margin);
    jp_.move_current = P("move_current", jp_.move_current);
    jp_.dir_m2 = P("dir_m2", jp_.dir_m2);
    jp_.dir_m5 = P("dir_m5", jp_.dir_m5);
    jp_.max_sweep = P("max_sweep", jp_.max_sweep);
    jp_.max_close = P("max_close", jp_.max_close);
    jp_.stall_velocity = P("stall_velocity", jp_.stall_velocity);
    jp_.stall_time = P("stall_time", jp_.stall_time);
    jp_.m5_step_offset = P("m5_step_offset", jp_.m5_step_offset);
    jp_.m2_step_offset = P("m2_step_offset", jp_.m2_step_offset);
    jp_.m2_contact_offset = P("m2_contact_offset", jp_.m2_contact_offset);
    jp_.m2_apply_contact_offset = P("m2_apply_contact_offset", jp_.m2_apply_contact_offset);
    jp_.m5_center_snap_max = P("m5_center_snap_max", jp_.m5_center_snap_max);
    jp_.m2_center_snap_max = P("m2_center_snap_max", jp_.m2_center_snap_max);
    if (jp_.m2_center_snap_max < 0 || jp_.m2_center_snap_max > 2048) {
        RCLCPP_ERROR(get_logger(), "jaw.m2_center_snap_max must be 0..2048 ticks.");
        config_error_ = true;
    }
    if (jp_.m5_center_snap_max < 0 || jp_.m5_center_snap_max > 2048) {
        RCLCPP_ERROR(get_logger(), "jaw.m5_center_snap_max must be 0..2048 ticks.");
        config_error_ = true;
    }
    jp_.m5_engaged_start = P("m5_engaged_start", jp_.m5_engaged_start);
    jp_.engaged_trend = P("engaged_trend", jp_.engaged_trend);
    jp_.m2_engaged_trend = P("m2_engaged_trend", jp_.m2_engaged_trend);
    jp_.m5_engaged_mid = P("m5_engaged_mid", jp_.m5_engaged_mid);
    jp_.m5_defer_travel = P("m5_defer_travel", jp_.m5_defer_travel);
    jp_.goto_push_ratio = P("goto_push_ratio", jp_.goto_push_ratio);
    jp_.stop_after_step = P("stop_after_step", jp_.stop_after_step);
    jp_.check_tol = P("check_tol", jp_.check_tol);
    jp_.wall_current = P("wall_current", jp_.wall_current);
    jp_.collapse_ratio = P("collapse_ratio", jp_.collapse_ratio);
    jp_.joystick_timeout = P("joystick_timeout", jp_.joystick_timeout);
    jp_.rp_hold_during_jaws = P("rp_hold_during_jaws", jp_.rp_hold_during_jaws);
    jp_.rp_final_hold = P("rp_final_hold", jp_.rp_final_hold);
    jp_.jaw_final_hold = P("jaw_final_hold", jp_.jaw_final_hold);
    jp_.log_dir = P("log_dir", jp_.log_dir);
    jp_.verify = P("verify", jp_.verify);
    jp_.jog_speed = P("jog_speed", jp_.jog_speed);
    jp_.m2_axis = P("m2_axis", jp_.m2_axis);
    jp_.m5_axis = P("m5_axis", jp_.m5_axis);
    jp_.deadband = P("deadband", jp_.deadband);
    if (jp_.sweep_current <= 0 || jp_.sweep_current > 100 || jp_.wall_current > 250 ||
        jp_.max_sweep < 500 || jp_.max_sweep > 8000 || jp_.max_close > 3000 ||
        jp_.move_fast_current > 250 || jp_.move_fast_profile > 150 || jp_.move_slow_margin < 100 ||
        jp_.move_current < 30 || jp_.move_current > 200 ||
        jp_.m5_engaged_mid < 1.0 || jp_.m5_engaged_mid > 1.2 || jp_.m5_defer_travel < 800 ||
        jp_.m5_defer_travel > 2500 || jp_.goto_push_ratio < 0.1 || jp_.goto_push_ratio > 0.7) {
        RCLCPP_ERROR(get_logger(), "jaw.* values out of safe range (tool file or parameters).");
        config_error_ = true;
    }
    // step finder constants (not ROS parameters)
    if (!contact) return;
    sweep_sf_.dx = tool_value("step.dx", sweep_sf_.dx);
    sweep_sf_.min_run = tool_value("step.min_run", sweep_sf_.min_run);
    sweep_sf_.hold = tool_value("step.hold", sweep_sf_.hold);
    sweep_sf_.w = tool_value("step.w", sweep_sf_.w);
    sweep_sf_.min_runup = tool_value("step.min_runup", sweep_sf_.min_runup);
    sweep_sf_.min_ratio = tool_value("step.min_ratio", sweep_sf_.min_ratio);
    sweep_sf_.z = tool_value("step.z", sweep_sf_.z);
    sweep_sf_.engaged_at = tool_value("step.engaged_at", sweep_sf_.engaged_at);
}

std::string ToolHomingNode::hw_error_report()
{
    std::string out;
    for (int id = 1; id <= 5; ++id) {
        int e = dxl_interface_->ReadHardwareError(id);
        if (e > 0) {
            std::string names;
            if (e & 0x01) names += " input-voltage";
            if (e & 0x04) names += " overheating";
            if (e & 0x08) names += " encoder";
            if (e & 0x10) names += " electrical-shock";
            if (e & 0x20) names += " OVERLOAD";
            out += "M" + std::to_string(id) + ":" + names + "; ";
        }
    }
    return out;
}

void ToolHomingNode::start_jaw_homing()
{
    std::string hw = hw_error_report();
    jaw_flags_.clear();
    if (!hw.empty()) {
        RCLCPP_ERROR(get_logger(), "Motor hardware error before jaw homing: %s Power-cycle/reboot the motors.",
                     hw.c_str());
        current_state_ = SetupState::JAW_ABORTED;
        publish_status("ABORTED");
        return;
    }
    // Same conditions as the calibration data: roll/pitch held at 100, jaws at 75 / profile 22.
    dxl_interface_->SetGoalCurrent(1, jp_.rp_hold_during_jaws);
    dxl_interface_->SetGoalCurrent(4, jp_.rp_hold_during_jaws);
    for (int m : {2, 5}) {
        dxl_interface_->SetGoalCurrent(m, jp_.sweep_current);
        dxl_interface_->SetProfileVelocityId(m, jp_.sweep_profile_velocity);
    }
    // CSV log of the whole jaw homing
    try {
        std::filesystem::create_directories(jp_.log_dir);
        std::time_t t = std::time(nullptr);
        std::ostringstream name;
        name << jp_.log_dir << "/jawhome_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << ".csv";
        jaw_log_name_ = name.str();
        jaw_log_.open(jaw_log_name_);
        jaw_log_ << "t,stage,pos_m2,vel_m2,cur_m2,pos_m5,vel_m5,cur_m5,goal_m2,goal_m5,pos_m1,pos_m4\n";
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Jaw log not written: %s", e.what());
    }
    RCLCPP_INFO(get_logger(), "Starting jaw homing (one jaw at a time). A = stop an automatic jaw move, Y = ESTOP.");
    jaw_plan_step_ = -1;
    if (jp_.verify) jaw_plan_ = {0, 1, 5, 2, 3, 6, 9};
    else jaw_plan_ = {0, 1, 2, 3, 9};
    if (jp_.m2_apply_contact_offset || m2_center_angle_ >= 0)   // M2: after the contact, move to its center
        jaw_plan_.insert(std::find(jaw_plan_.begin(), jaw_plan_.end(), 3) + 1, 4);
    for (int m : {2, 5}) has_true_[m] = false;
    if (jp_.verify) RCLCPP_WARN(get_logger(), "VERIFY MODE: you will be asked to confirm each jaw's true center.");
    jaw_dt_.clear();
    jaw_prev_tick_ = 0;
    if (!read_jaws()) { jaw_abort("cannot read jaw motors"); return; }
    jaw_next();
}

void ToolHomingNode::jaw_next()
{
    ++jaw_plan_step_;
    int what = (jaw_plan_step_ < static_cast<int>(jaw_plan_.size())) ? jaw_plan_[jaw_plan_step_] : 9;
    switch (what) {
        case 0: jaw_start_sweep(5); break;
        case 1: jaw_start_goto_m5_middle(); break;   // step estimate, or (engaged start) start - offset
        case 2: jaw_start_sweep(2); break;
        case 3: jaw_start_close(2); break;
        case 4: jaw_start_goto_m2_offset(); break;   // jaw.m2_apply_contact_offset only
        case 5: jaw_start_verify(5); break;          // verify mode only
        case 6: jaw_start_verify(2); break;
        default: jaw_finish(); break;
    }
}

// ---------------- verify mode (calibration of a new adapter/instrument) ----------------
void ToolHomingNode::jaw_start_verify(int m)
{
    verify_m_ = m;
    robot_center_[m] = jpos_[m];
    verify_goal_ = jpos_[m];
    verify_confirm_ = false;
    dxl_interface_->SetProfileVelocityId(m, jp_.move_profile_velocity);
    current_state_ = SetupState::JAW_VERIFY;
    RCLCPP_WARN(get_logger(), "VERIFY M%d: robot put it at %d. Nudge M%d to the TRUE center with the %s stick "
                "(leave it if it is right), release the stick, press A.",
                m, jpos_[m], m, m == 5 ? "RIGHT" : "LEFT");
}

void ToolHomingNode::jaw_tick_verify(double dt)
{
    const int m = verify_m_;
    float val = joy_fresh() ? (m == 5 ? axis_m5_ : axis_m2_) : 0.0f;
    if (verify_confirm_) {
        verify_confirm_ = false;
        if (std::abs(val) >= jp_.deadband) {
            RCLCPP_WARN(get_logger(), "Release the stick before pressing A.");
        } else {
            true_center_[m] = jpos_[m];
            has_true_[m] = true;
            RCLCPP_INFO(get_logger(), "M%d: robot %d, true %d, ERROR %+d ticks (~%+.1f deg)", m, robot_center_[m],
                        true_center_[m], robot_center_[m] - true_center_[m], (robot_center_[m] - true_center_[m]) / 13.0);
            if (mode_step_[m])
                RCLCPP_INFO(get_logger(), "M%d: measured step offset for this adapter = %.0f (parameter now %.0f)",
                            m, (step_pos_[m] - true_center_[m]) * jdir(m), m == 5 ? jp_.m5_step_offset : jp_.m2_step_offset);
            jaw_hold_here();
            // the pause for the operator is not a slow control loop: restart the loop-rate guard
            jaw_dt_.clear();
            jaw_prev_tick_ = 0;
            jaw_next();
            return;
        }
    }
    if (std::abs(val) >= jp_.deadband) {
        double mag = (std::min(std::abs(val), 1.0f) - jp_.deadband) / (1.0 - jp_.deadband);
        verify_goal_ += std::copysign(mag, val) * jp_.jog_speed * dt;
        starting_locations_[m - 1] = static_cast<int>(std::lround(verify_goal_));
        jaw_command();
    }
}

bool ToolHomingNode::read_raw()
{
    int32_t p[5], v[5];
    int16_t c[5];
    if (!dxl_interface_->SyncReadState(p, v, c)) return false;   // ONE serial transaction for all motors
    for (int i = 0; i < 5; ++i) {
        raw_pos_[i + 1] = p[i]; raw_vel_[i + 1] = v[i]; raw_cur_[i + 1] = c[i];
        acc_vel_[i + 1] += v[i]; acc_cur_[i + 1] += c[i];
    }
    ++acc_n_;
    raw_valid_ = true;
    return true;
}

void ToolHomingNode::take_average()
{
    // position = latest; velocity/current = mean of the samples since the last logic tick
    for (int i = 1; i <= 5; ++i) {
        jpos_[i] = raw_pos_[i];
        jvel_[i] = acc_n_ ? static_cast<int32_t>(std::lround(acc_vel_[i] / acc_n_)) : raw_vel_[i];
        jcur_[i] = acc_n_ ? static_cast<int16_t>(std::lround(acc_cur_[i] / acc_n_)) : raw_cur_[i];
        acc_vel_[i] = 0; acc_cur_[i] = 0;
    }
    acc_n_ = 0;
}

bool ToolHomingNode::read_jaws()
{
    // direct read (used outside the timer: start of jaw homing, abort)
    if (!read_raw()) return false;
    take_average();
    return true;
}

void ToolHomingNode::jaw_command()
{
    dxl_interface_->SetBulkPos(starting_locations_);
}

void ToolHomingNode::jaw_hold_here()
{
    // profile 0 + goal = present position: also resets the motors' internal trajectories.
    // Every following jaw move sets its own profile velocity again.
    dxl_interface_->SetProfileVelocityId(2, 0);
    dxl_interface_->SetProfileVelocityId(5, 0);
    starting_locations_[1] = jpos_[2];
    starting_locations_[4] = jpos_[5];
    jaw_command();
}

void ToolHomingNode::jaw_flag(const std::string& text)
{
    jaw_flags_.push_back(text);
    RCLCPP_WARN(get_logger(), "JAW FLAG: %s", text.c_str());
}

void ToolHomingNode::jaw_log_row(const char* stage)
{
    if (!jaw_log_.is_open()) return;
    jaw_log_ << std::fixed << std::setprecision(3) << now_s() << "," << stage << ","
             << jpos_[2] << "," << jvel_[2] << "," << jcur_[2] << ","
             << jpos_[5] << "," << jvel_[5] << "," << jcur_[5] << ","
             << starting_locations_[1] << "," << starting_locations_[4] << ","
             << starting_locations_[0] << "," << starting_locations_[3] << "\n";
}

void ToolHomingNode::jaw_tick()
{
    double now = now_s();   // jpos_/jvel_/jcur_ already filled by take_average()
    // The detectors were calibrated at 50 Hz. Refuse to continue if the loop is much slower.
    double& prev = jaw_prev_tick_;
    if (prev > 0) {
        jaw_dt_.push_back(now - prev);
        if (jaw_dt_.size() > 25) jaw_dt_.pop_front();
        if (jaw_dt_.size() == 25) {
            double mean = 0;
            for (double d : jaw_dt_) mean += d;
            mean /= 25.0;
            if (mean > 1.0 / 30.0) {
                prev = now;
                jaw_abort("jaw control loop too slow (" + std::to_string(static_cast<int>(1.0 / mean)) +
                          " Hz, needs >= 30 Hz)");
                return;
            }
        }
    }
    prev = now;
    if (current_state_ == SetupState::JAW_SWEEP) { jaw_log_row(jaw_m_ == 5 ? "SWEEP_M5" : "SWEEP_M2"); jaw_tick_sweep(now); }
    else if (current_state_ == SetupState::JAW_GOTO) { jaw_log_row("GOTO_M5_MIDDLE"); jaw_tick_goto(now); }
    else if (current_state_ == SetupState::JAW_CLOSE) { jaw_log_row(jaw_m_ == 5 ? "CLOSE_M5" : "CLOSE_M2"); jaw_tick_close(now); }
}

// ---------------- sweep ----------------
void ToolHomingNode::jaw_start_sweep(int m)
{
    jaw_m_ = m;
    dxl_interface_->SetProfileVelocityId(m, jp_.sweep_profile_velocity);
    sweep_start_[m] = jpos_[m];
    sweep_sf_.reset();
    sweep_res_ = jaw_homing::StepResult();
    sweep_det_ = jaw_homing::ContactDetector(jp_.sweep_current, jp_.collapse_ratio);
    sweep_ih_.clear();
    sweep_n_ = 0;
    stall_since_ = -1;
    engaged_deferred_ = false;
    operator_stop_ = false;
    starting_locations_[m - 1] = jpos_[m] + jdir(m) * jp_.max_sweep;   // motor profile sets the speed
    jaw_command();
    jaw_t0_ = now_s();
    current_state_ = SetupState::JAW_SWEEP;
    RCLCPP_INFO(get_logger(), "M%d opening sweep (stops by itself; press A if it opens past ~90 deg).", m);
}

void ToolHomingNode::jaw_tick_sweep(double now)
{
    const int m = jaw_m_;
    const double age = now - jaw_t0_;
    sweep_sf_.add(jpos_[m], jcur_[m]);
    ++sweep_n_;
    bool resist = sweep_det_.update(jvel_[m], jcur_[m], age);
    sweep_ih_.push_back(std::abs(static_cast<double>(jcur_[m])));
    if (sweep_ih_.size() > 15) sweep_ih_.pop_front();
    double ih_mean = 0;
    for (double v : sweep_ih_) ih_mean += v;
    bool at_limit = sweep_ih_.size() == 15 && ih_mean / 15.0 >= 0.95 * jp_.sweep_current;
    if (sweep_n_ % 5 == 0) sweep_res_ = sweep_sf_.analyze();
    const auto& r = sweep_res_;
    double travel = std::abs(static_cast<double>(jpos_[m] - sweep_start_[m]));
    bool stalled = std::abs(jvel_[m]) < jp_.stall_velocity && age > 0.5;
    if (stalled) { if (stall_since_ < 0) stall_since_ = now; } else stall_since_ = -1;
    double trend_limit = (m == 2) ? jp_.m2_engaged_trend : jp_.engaged_trend;

    if (r.valid && r.found && r.mode_step && r.travel >= r.step_end_travel + jp_.stop_after_step)
        return jaw_stop_sweep("step");
    if (r.valid && r.has_early && r.early_trend >= trend_limit && !(r.found && r.mode_step)) {
        // M5: an engaged start is already climbing at 300-450 ticks. If it is flat there, the rise is a step
        // that began inside the early window (happens when the start position puts the step ~500 ticks in):
        // keep sweeping so the step finder can confirm it -- up to m5_defer_travel, then fall back.
        bool step_starting = (m == 5) && r.mid_trend < jp_.m5_engaged_mid;
        if (!step_starting || travel >= jp_.m5_defer_travel) {
            if (engaged_deferred_) jaw_flag("M5 early rise looked like a step but none was confirmed: used engaged start");
            return jaw_stop_sweep("engaged");
        }
        if (!engaged_deferred_) {
            engaged_deferred_ = true;
            RCLCPP_INFO(get_logger(), "M5: current rose at %.0f ticks (trend %.2f) but was flat before it (%.2f): "
                        "a step starting, not an engaged start -- continuing the sweep", travel, r.early_trend,
                        r.mid_trend);
        }
    }
    if (operator_stop_) return jaw_stop_sweep("operator");
    if (!joy_fresh()) return jaw_stop_sweep("joystick_lost");
    if (resist) return jaw_stop_sweep("opening_resistance");
    if (at_limit && age > 1.0) return jaw_stop_sweep("at_current_limit");
    if (stall_since_ >= 0 && now - stall_since_ >= jp_.stall_time) return jaw_stop_sweep("stall");
    if (travel >= jp_.max_sweep - 20) return jaw_stop_sweep("max_sweep");
    jaw_command();
}

void ToolHomingNode::jaw_stop_sweep(const std::string& why)
{
    const int m = jaw_m_;
    jaw_hold_here();
    jaw_homing::StepResult f = sweep_sf_.analyze();
    double need_after_step = 200;
    if (!(f.found && f.mode_step) && (why == "at_current_limit" || why == "opening_resistance")) {
        // the current kept rising until the limit: a shorter "stays up" window is enough here
        jaw_homing::StepResult f2 = sweep_sf_.analyze(200);
        if (f2.found && f2.mode_step) {
            f = f2;
            need_after_step = 150;
            RCLCPP_INFO(get_logger(), "M%d: step found with the short window (sweep ended at the current limit)", m);
        }
    }
    sweep_stop_[m] = jpos_[m];
    sweep_why_[m] = why;
    slip_[m] = f.found && f.slip;
    // trust the step only if the finder is confident AND we saw enough of it; never for an engaged start
    mode_step_[m] = (why != "engaged" && f.found && f.mode_step && f.travel >= f.step_end_travel + need_after_step);
    has_step_est_[m] = mode_step_[m];
    step_pos_[m] = f.found ? f.step_pos : 0;
    if (mode_step_[m]) {
        double off = (m == 2) ? jp_.m2_step_offset : jp_.m5_step_offset;
        step_est_[m] = f.step_pos - jdir(m) * off;
    }
    RCLCPP_INFO(get_logger(), "M%d sweep stopped (%s): mode=%s%s", m, why.c_str(),
                mode_step_[m] ? "step" : "contact",
                mode_step_[m] ? (", center estimate " + std::to_string(static_cast<int>(step_est_[m]))).c_str() : "");
    if (why != "step" && why != "engaged" && why != "at_current_limit")
        jaw_flag("M" + std::to_string(m) + " sweep stopped by " + why);
    if (slip_[m]) jaw_flag("M" + std::to_string(m) + " slip seen during free spin");
    if (why == "joystick_lost" || why == "operator") { jaw_abort("sweep stopped by " + why); return; }
    jaw_next();
}

// ---------------- M5 back to the middle ----------------
void ToolHomingNode::jaw_start_goto_m5_middle()
{
    jaw_m_ = 5;
    if (mode_step_[5]) goto_target_ = static_cast<int32_t>(std::lround(step_est_[5]));
    else goto_target_ = static_cast<int32_t>(std::lround(sweep_start_[5] - jdir(5) * jp_.m5_engaged_start));
    bool snapped = false;
    if (mode_step_[5] && m5_center_angle_ >= 0) {
        // the step only picks the turn: go to the position nearest the step estimate whose angle is the center angle
        const double est = step_est_[5];
        const double ang = std::fmod(std::fmod(est, 4096.0) + 4096.0, 4096.0);
        const double shift = std::fmod(m5_center_angle_ - ang + 6144.0, 4096.0) - 2048.0;   // -2048..2048
        if (std::abs(shift) <= jp_.m5_center_snap_max) {
            goto_target_ = static_cast<int32_t>(std::lround(est + shift));
            snapped = true;
            RCLCPP_INFO(get_logger(), "M5 center: step estimate %.0f snapped %+.0f to motor angle %.0f -> %d",
                        est, shift, m5_center_angle_, goto_target_);
        } else {
            RCLCPP_WARN(get_logger(), "M5 center: step estimate %.0f is %+.0f from the center angle %.0f (more than "
                        "jaw.m5_center_snap_max %.0f): using the STEP estimate (different instrument?)",
                        est, shift, m5_center_angle_, jp_.m5_center_snap_max);
            jaw_flag("M5 step " + std::to_string(static_cast<int>(shift)) +
                     " ticks from the center angle: used the step estimate");
        }
    }
    goto_label_ = "M5 back to the middle";
    goto_final_ = goto_target_;
    // fast part only if there is room: stop move_slow_margin short of the target (on the open side)
    const int32_t here = jpos_[5];
    goto_fast_ = !slow_jaws_ &&                                // slow jaws / calibration: the whole move is slow
                 std::abs(goto_final_ - here) > jp_.move_slow_margin + 50 &&
                 (here - goto_final_) * jdir(5) > 0;          // M5 is open and closing toward the middle
    if (goto_fast_) {
        goto_target_ = goto_final_ + jdir(5) * jp_.move_slow_margin;
        dxl_interface_->SetGoalCurrent(5, jp_.move_fast_current);
        dxl_interface_->SetProfileVelocityId(5, jp_.move_fast_profile);
    } else {
        dxl_interface_->SetGoalCurrent(5, jp_.move_current);
        // calibration: at the SWEEP speed (22, ~340 ticks/s), not move_profile_velocity (60)
        dxl_interface_->SetProfileVelocityId(5, slow_jaws_ ? jp_.sweep_profile_velocity : jp_.move_profile_velocity);
    }
    starting_locations_[4] = goto_target_;
    jaw_command();
    goto_ok_ = 0;
    goto_push_v_.clear();
    jaw_t0_ = now_s();
    goto_deadline_ = jaw_t0_ + 8.0;
    if (slow_jaws_)   // slower move: allow twice its time at the sweep speed (~15.6 ticks/s per velocity unit)
        goto_deadline_ = jaw_t0_ + std::max(8.0, 2.0 + 2.0 * std::abs(goto_final_ - here) /
                                                 (15.6 * std::max(1, jp_.sweep_profile_velocity)));
    current_state_ = SetupState::JAW_GOTO;
    RCLCPP_INFO(get_logger(), "%s -> %d (%s)%s", goto_label_.c_str(), goto_final_,
                snapped ? "center angle" : (mode_step_[5] ? "step estimate" : "engaged start - offset"),
                goto_fast_ ? ", fast then gentle" : "");
}

void ToolHomingNode::jaw_tick_goto(double now)
{
    const int m = jaw_m_;
    starting_locations_[m - 1] = joy_fresh() ? goto_target_ : jpos_[m];   // pause without joystick
    jaw_command();
    bool close_enough = std::abs(jpos_[m] - goto_target_) <= 12;
    bool blocked = (now - jaw_t0_ > 0.5) && std::abs(jvel_[m]) < jp_.stall_velocity;
    if (goto_fast_) {
        bool reached = std::abs(jpos_[m] - goto_target_) <= 40 || (jpos_[m] - goto_target_) * jdir(m) < 0;
        if (reached || (blocked && now - jaw_t0_ > 0.5)) {
            dxl_interface_->SetGoalCurrent(m, jp_.move_current);               // the gentle current
            dxl_interface_->SetProfileVelocityId(m, jp_.move_profile_velocity);
            goto_fast_ = false;
            goto_target_ = goto_final_;
            starting_locations_[m - 1] = goto_target_;
            jaw_command();
            goto_ok_ = 0;
            if (!reached) jaw_flag(goto_label_ + ": blocked during the fast part");
            jaw_t0_ = now;                                                       // fresh blocked timer
            return;
        }
        if (now > goto_deadline_) {
            dxl_interface_->SetGoalCurrent(m, jp_.sweep_current);
            jaw_flag(goto_label_ + " timed out (fast part)");
            jaw_abort(goto_label_ + " did not reach its target");
        }
        return;
    }
    // pushing guard (gentle part): a normal move may run at the current limit too (psm1: steady ~18-20);
    // only a near-stall at the limit (5-sample mean < goto_push_ratio x profile) counts. Not near the target.
    {
        const int push = static_cast<int>(jcur_[m]) * ((goto_target_ - jpos_[m]) > 0 ? 1 : -1);
        bool pushing = (now - jaw_t0_ > 0.3) && std::abs(jpos_[m] - goto_target_) >= 60 &&
                       push >= 0.9 * jp_.move_current;
        if (pushing) goto_push_v_.push_back(std::abs(static_cast<double>(jvel_[m])));
        else goto_push_v_.clear();
        if (goto_push_v_.size() > 5) goto_push_v_.pop_front();
        if (goto_push_v_.size() == 5) {
            double v5 = 0;
            for (double v : goto_push_v_) v5 += v;
            v5 /= 5.0;
            if (v5 < jp_.goto_push_ratio * jp_.move_profile_velocity) {
                jaw_hold_here();
                jaw_flag(goto_label_ + ": pushing something at " + std::to_string(jpos_[m]) + " (speed " +
                         std::to_string(static_cast<int>(v5)) + " at the current limit)");
                jaw_abort(goto_label_ + " was pushing something (the other jaw?) " +
                          std::to_string(std::abs(goto_target_ - jpos_[m])) + " ticks before its target");
                return;
            }
        }
    }
    goto_ok_ = (close_enough || blocked) ? goto_ok_ + 1 : 0;
    if (goto_ok_ >= 5) {
        if (!close_enough) {
            int short_by = std::abs(goto_target_ - jpos_[m]);
            jaw_flag(goto_label_ + " stopped " + std::to_string(short_by) + " ticks short (blocked)");
            bool opening = (goto_target_ - jpos_[m]) * jdir(m) > 0;
            if (opening) { jaw_abort(goto_label_ + " was blocked while OPENING"); return; }
            if (short_by > 150) { jaw_abort(goto_label_ + " ended far short of its target"); return; }
        }
        jaw_hold_here();
        if (m == 5) publish_limit_state(5, 3);    // M5 centered (GUI box goes green before M2 starts)
        jaw_next();
        return;
    }
    if (now > goto_deadline_) {
        jaw_flag(goto_label_ + " timed out");
        jaw_abort(goto_label_ + " did not reach its target");
    }
}

// ---------------- M2 from the contact to contact - offset (jaw.m2_apply_contact_offset) ----------------
// The contact test fires late on some tools (needle driver: ~200 ticks past the true touch, the motor still
// moving at the current limit). The calibrated jaw.m2_contact_offset says how far: move there gently.
void ToolHomingNode::jaw_start_goto_m2_offset()
{
    jaw_m_ = 2;
    const double base = jp_.m2_apply_contact_offset ? contact_pos_ - jdir(2) * jp_.m2_contact_offset
                                                    : static_cast<double>(contact_pos_);
    goto_final_ = static_cast<int32_t>(std::lround(base));
    goto_label_ = "M2 from the contact to contact - offset";
    if (m2_center_angle_ >= 0) {
        // the contact only picks the turn: nearest position with the calibrated M2 center angle
        const double ang = std::fmod(std::fmod(base, 4096.0) + 4096.0, 4096.0);
        const double shift = std::fmod(m2_center_angle_ - ang + 6144.0, 4096.0) - 2048.0;
        const double target = base + shift;
        // never close further than the contact (that would squeeze M5): only snap toward open, or 30 ticks closed
        const bool closes_past = (target - contact_pos_) * jdir(2) < -30;
        if (std::abs(shift) <= jp_.m2_center_snap_max && !closes_past) {
            goto_final_ = static_cast<int32_t>(std::lround(target));
            goto_label_ = "M2 from the contact to its center angle";
            RCLCPP_INFO(get_logger(), "M2 center: %.0f snapped %+.0f to motor angle %.0f -> %d",
                        base, shift, m2_center_angle_, goto_final_);
        } else {
            RCLCPP_WARN(get_logger(), "M2 center: %.0f is %+.0f from the center angle %.0f (%s): kept contact - offset",
                        base, shift, m2_center_angle_, closes_past ? "would close past the contact"
                                                                   : "more than jaw.m2_center_snap_max");
            jaw_flag("M2 contact " + std::to_string(static_cast<int>(shift)) + " ticks from the center angle: not snapped");
        }
    }
    goto_target_ = goto_final_;
    goto_fast_ = false;
    dxl_interface_->SetGoalCurrent(2, jp_.move_current);
    dxl_interface_->SetProfileVelocityId(2, jp_.sweep_profile_velocity);
    starting_locations_[1] = goto_target_;
    jaw_command();
    goto_ok_ = 0;
    goto_push_v_.clear();
    jaw_t0_ = now_s();
    goto_deadline_ = jaw_t0_ + std::max(8.0, 2.0 + 2.0 * std::abs(goto_final_ - jpos_[2]) /
                                                  (15.6 * std::max(1, jp_.sweep_profile_velocity)));
    current_state_ = SetupState::JAW_GOTO;
    RCLCPP_INFO(get_logger(), "%s: %d -> %d (offset %.0f)", goto_label_.c_str(), contact_pos_, goto_final_,
                jp_.m2_contact_offset);
}

// ---------------- M2 closes onto M5 ----------------
void ToolHomingNode::jaw_start_close(int m)
{
    jaw_m_ = m;
    const int wall = (m == 2) ? 5 : 2;
    const int d = jdir(m);
    dxl_interface_->SetGoalCurrent(wall, jp_.wall_current);      // the other jaw becomes a stiff wall
    dxl_interface_->SetProfileVelocityId(m, jp_.sweep_profile_velocity);
    double floor_pos;                                            // never close further than this
    if (m == 2) floor_pos = jpos_[m] - d * static_cast<double>(jp_.max_close);  // M2 vs a held M5: contact is the truth,
                                                                 // a step estimate (adapter-dependent) must not cut it short
    else if (has_step_est_[m]) floor_pos = step_est_[m] - d * 350.0;
    else if (sweep_why_[m] != "step") floor_pos = sweep_start_[m] - d * 350.0;  // engaged: started near center
    else floor_pos = jpos_[m] - d * static_cast<double>(jp_.max_close);
    close_max_travel_ = std::min<double>(jp_.max_close, std::max(0.0, (jpos_[m] - floor_pos) * d));
    close_start_ = jpos_[m];
    close_check_ = has_step_est_[m];
    close_det_ = jaw_homing::ContactDetector(jp_.sweep_current, jp_.collapse_ratio);
    stall_since_ = -1;
    operator_stop_ = false;
    starting_locations_[m - 1] = static_cast<int>(std::lround(jpos_[m] - d * close_max_travel_));
    jaw_command();
    jaw_t0_ = now_s();
    current_state_ = SetupState::JAW_CLOSE;
    RCLCPP_INFO(get_logger(), "M%d closing until it touches M%d (A if you see contact first).", m, wall);
}

void ToolHomingNode::jaw_tick_close(double now)
{
    const int m = jaw_m_;
    const double age = now - jaw_t0_;
    bool hit = close_det_.update(jvel_[m], jcur_[m], age);
    bool stalled = std::abs(jvel_[m]) < jp_.stall_velocity && age > 0.5;
    if (stalled) { if (stall_since_ < 0) stall_since_ = now; } else stall_since_ = -1;
    if (operator_stop_) return jaw_stop_close("operator");
    if (!joy_fresh()) return jaw_stop_close("joystick_lost");
    if (hit) return jaw_stop_close("contact");
    if (stall_since_ >= 0 && now - stall_since_ >= jp_.stall_time) {
        int moved = std::abs(jpos_[m] - close_start_);
        bool pressing = std::abs(jcur_[m]) >= 0.9 * jp_.sweep_current;
        return jaw_stop_close((moved < 40 && pressing) ? "already_touching" : "stall");
    }
    if (std::abs(jpos_[m] - close_start_) >= close_max_travel_ - 20) return jaw_stop_close("max_close");
    jaw_command();
}

void ToolHomingNode::jaw_stop_close(const std::string& why)
{
    const int m = jaw_m_;
    const int wall = (m == 2) ? 5 : 2;
    jaw_hold_here();
    dxl_interface_->SetGoalCurrent(wall, jp_.sweep_current);
    close_why_ = why;
    contact_pos_ = jpos_[m];
    if (why != "contact" && why != "operator" && why != "already_touching") {
        jaw_flag("M" + std::to_string(m) + " close ended by " + why + ", not by contact");
        jaw_abort("M" + std::to_string(m) + " close did not find contact (" + why + ")");
        return;
    }
    has_check_ = false;
    if (close_check_) {
        double from_contact = contact_pos_ - jdir(m) * jp_.m2_contact_offset;
        check_diff_ = from_contact - step_est_[m];
        has_check_ = true;
        RCLCPP_INFO(get_logger(), "CHECK M%d: contact says center %.0f, step said %.0f (difference %+.0f ticks)",
                    m, from_contact, step_est_[m], check_diff_);
        if (std::abs(check_diff_) > jp_.check_tol)
            jaw_flag("M" + std::to_string(m) + " step and contact disagree by " +
                     std::to_string(static_cast<int>(check_diff_)) + " ticks");
    }
    jaw_next();
}

// ---------------- end states ----------------
void ToolHomingNode::jaw_abort(const std::string& why)
{
    read_jaws();
    jaw_hold_here();
    dxl_interface_->SetGoalCurrent(2, jp_.sweep_current);
    dxl_interface_->SetGoalCurrent(5, jp_.sweep_current);
    std::string hw = hw_error_report();
    current_state_ = SetupState::JAW_ABORTED;
    publish_status("ABORTED");
    RCLCPP_ERROR(get_logger(), "JAW HOMING ABORTED: %s. Motors holding; check the tool, then Ctrl+C.", why.c_str());
    if (!hw.empty()) RCLCPP_ERROR(get_logger(), "Motor hardware error: %s Power-cycle/reboot the motors.", hw.c_str());
    if (jaw_log_.is_open()) { jaw_log_ << "# ABORTED: " << why << " " << hw << "\n"; jaw_log_.close(); }
}

void ToolHomingNode::jaw_finish()
{
    jaw_hold_here();
    dxl_interface_->SetGoalCurrent(1, jp_.rp_final_hold);
    dxl_interface_->SetGoalCurrent(4, jp_.rp_final_hold);
    dxl_interface_->SetGoalCurrent(2, jp_.jaw_final_hold);
    dxl_interface_->SetGoalCurrent(5, jp_.jaw_final_hold);
    publish_limit_state(5, 3);
    publish_limit_state(2, 3);
    current_state_ = SetupState::JAW_DONE;
    publish_status("COMPLETE");
    RCLCPP_INFO(get_logger(), "JAW HOMING COMPLETE. M5 = %d (%s, sweep: %s), M2 = %d (%s, closed onto M5)%s",
                starting_locations_[4], mode_step_[5] ? "step" : "engaged start", sweep_why_[5].c_str(),
                starting_locations_[1], mode_step_[2] ? "step-checked" : "contact",
                jaw_flags_.empty() ? "" : "  -- see JAW FLAG warnings");
    RCLCPP_INFO(get_logger(), "All homing complete: roll %d, pitch %d, M2 %d, M5 %d.",
                starting_locations_[0], starting_locations_[3], starting_locations_[1], starting_locations_[4]);
    RCLCPP_INFO(get_logger(), "Time: roll %.1f s, pitch %.1f s, jaws %.1f s, total %.1f s",
                t_roll_done_ - t_homing_start_, t_pitch_done_ - t_roll_done_, now_s() - t_pitch_done_,
                now_s() - t_homing_start_);
    if (jaw_log_.is_open()) jaw_log_.close();
    if (calibrate_) cal_review();
    // one line per homing in the summary file
    try {
        std::string path = jp_.log_dir + "/jaw_homing_summary_v2.csv";
        bool fresh = !std::filesystem::exists(path);
        std::ofstream s(path, std::ios::app);
        if (fresh) s << "log_file,m5_sweep_stop,m5_mode,m5_final,m2_sweep_stop,m2_mode,m2_step_estimate,"
                        "m2_contact,m2_close,check_diff,roll,pitch,roll_limit1,roll_limit2,roll_range,"
                        "t_roll_s,t_pitch_s,t_jaws_s,t_total_s,m5_step_pos,m5_true,m5_error,m2_step_pos,m2_true,m2_error,flags\n";
        std::string flags;
        for (const auto& f : jaw_flags_) flags += f + " | ";
        s << jaw_log_name_ << "," << sweep_why_[5] << "," << (mode_step_[5] ? "step" : "contact") << ","
          << starting_locations_[4] << "," << sweep_why_[2] << "," << (mode_step_[2] ? "step" : "contact") << ","
          << (has_step_est_[2] ? std::to_string(static_cast<int>(step_est_[2])) : "") << ","
          << contact_pos_ << "," << close_why_ << ","
          << (has_check_ ? std::to_string(static_cast<int>(check_diff_)) : "") << ","
          << starting_locations_[0] << "," << starting_locations_[3] << ","
          << roll_lim1_ << "," << roll_lim2_ << "," << std::abs(roll_lim2_ - roll_lim1_) << ","
          << std::fixed << std::setprecision(1)
          << (t_roll_done_ - t_homing_start_) << "," << (t_pitch_done_ - t_roll_done_) << ","
          << (now_s() - t_pitch_done_) << "," << (now_s() - t_homing_start_) << ","
          << std::setprecision(0)
          << (mode_step_[5] ? std::to_string(static_cast<int>(step_pos_[5])) : "") << ","
          << (has_true_[5] ? std::to_string(true_center_[5]) : "") << ","
          << (has_true_[5] ? std::to_string(robot_center_[5] - true_center_[5]) : "") << ","
          << (mode_step_[2] ? std::to_string(static_cast<int>(step_pos_[2])) : "") << ","
          << (has_true_[2] ? std::to_string(true_center_[2]) : "") << ","
          << (has_true_[2] ? std::to_string(robot_center_[2] - true_center_[2]) : "") << ","
          << "\"" << flags << "\"\n";
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Summary not written: %s", e.what());
    }
    write_home();
}

std::string ToolHomingNode::home_file() const
{
    return jp_.log_dir + "/home_" + adapter_ + ".yaml";
}

// The handoff to disc_drive: the five home counts (the goals the homing ended on; M3 is not homed,
// it stays at its start position). Written only on COMPLETE.
void ToolHomingNode::write_home()
{
    std_msgs::msg::Int32MultiArray msg;
    for (int i = 0; i < 5; ++i) msg.data.push_back(starting_locations_[i]);
    home_pub_->publish(msg);
    std::string flags;
    for (const auto& f : jaw_flags_) flags += (flags.empty() ? "" : " | ") + f;
    for (auto& c : flags) if (c == '"') c = '\'';
    std::time_t t = std::time(nullptr);
    std::ofstream f(home_file());
    f << "# tool_homing COMPLETE: disc_drive zero_from_homing reads this (counts = M1..M5)\n"
      << "adapter: " << adapter_ << "\n"
      << "tool: " << tool_name_ << "\n"
      << "time: \"" << std::put_time(std::localtime(&t), "%Y-%m-%dT%H:%M:%S") << "\"\n"
      << "log: \"" << jaw_log_name_ << "\"\n"
      << "flags: \"" << flags << "\"\n"
      << "counts: [" << starting_locations_[0] << ", " << starting_locations_[1] << ", "
      << starting_locations_[2] << ", " << starting_locations_[3] << ", " << starting_locations_[4] << "]\n";
    if (f) RCLCPP_INFO(get_logger(), "Home counts written to %s", home_file().c_str());
    else RCLCPP_ERROR(get_logger(), "Home counts NOT written to %s", home_file().c_str());
}


// =====================================================================================
//  ROLL / PITCH limit homing at ~50 Hz (same sequence as before):
//    step 0: sweep to limit 1   step 1: reverse to limit 2   step 2: center at the midpoint
//  Limit = (stopped OR current above threshold) held for rp.confirm_time, after rp.start_grace.
//  Roll limit1 -> limit2 runs fast until rp.roll_slow_margin before the EXPECTED limit 2, then
//  finishes at the old speed, so the hard stop is still met at the old speed.
// =====================================================================================
void ToolHomingNode::load_rp_params()
{
    auto P = [this](const std::string& n, auto def) { return this->declare_parameter("rp." + n, tool_value("rp." + n, def)); };
    rp_.stall_velocity = P("stall_velocity", rp_.stall_velocity);
    rp_.confirm_time = P("confirm_time", rp_.confirm_time);
    rp_.start_grace = P("start_grace", rp_.start_grace);
    rp_.roll_current_thresh = P("roll_current_thresh", rp_.roll_current_thresh);
    rp_.pitch_current_thresh = P("pitch_current_thresh", rp_.pitch_current_thresh);
    rp_.roll_sweep_profile = P("roll_sweep_profile", rp_.roll_sweep_profile);
    rp_.roll_fast_profile = P("roll_fast_profile", rp_.roll_fast_profile);
    rp_.roll_final_profile = P("roll_final_profile", rp_.roll_final_profile);
    rp_.roll_stop_current = P("roll_stop_current", rp_.roll_stop_current);
    rp_.roll_fast_current = P("roll_fast_current", rp_.roll_fast_current);
    rp_.roll_center_current = P("roll_center_current", rp_.roll_center_current);
    rp_.roll_expected_range = P("roll_expected_range", rp_.roll_expected_range);
    rp_.roll_slow_margin = P("roll_slow_margin", rp_.roll_slow_margin);
    rp_.roll_center_profile = P("roll_center_profile", rp_.roll_center_profile);
    rp_.pitch_center_profile = P("pitch_center_profile", rp_.pitch_center_profile);
    rp_.pitch_sweep_current = P("pitch_sweep_current", rp_.pitch_sweep_current);
    rp_.pitch_contact_sat = P("pitch_contact_sat", rp_.pitch_contact_sat);
    rp_.pitch_contact_ratio = P("pitch_contact_ratio", rp_.pitch_contact_ratio);
    rp_.pitch_sweep_profile = P("pitch_sweep_profile", rp_.pitch_sweep_profile);
    rp_.pitch_fast_profile = P("pitch_fast_profile", rp_.pitch_fast_profile);
    rp_.pitch_expected_range = P("pitch_expected_range", rp_.pitch_expected_range);
    rp_.pitch_slow_margin = P("pitch_slow_margin", rp_.pitch_slow_margin);
    rp_.pitch_push_frac = P("pitch_push_frac", rp_.pitch_push_frac);
    rp_.pitch_push_time = P("pitch_push_time", rp_.pitch_push_time);
    rp_.free_jaws_during_pitch = P("free_jaws_during_pitch", rp_.free_jaws_during_pitch);
    rp_.pitch_fast = P("pitch_fast", rp_.pitch_fast);
    rp_.pitch_min_range = P("pitch_min_range", rp_.pitch_min_range);
    rp_.pitch_max_range = P("pitch_max_range", rp_.pitch_max_range);
    rp_.pitch_angle_tol = P("pitch_angle_tol", rp_.pitch_angle_tol);
    if (rp_.pitch_angle_tol < 0 || rp_.pitch_angle_tol > 1500) {
        RCLCPP_ERROR(get_logger(), "rp.pitch_angle_tol must be 0 (off) .. 1500 ticks.");
        config_error_ = true;
    }
    if (rp_.pitch_min_range < 0 || rp_.pitch_max_range < 0 || rp_.pitch_max_range > 8000 ||
        (rp_.pitch_min_range > 0 && rp_.pitch_max_range > 0 && rp_.pitch_min_range >= rp_.pitch_max_range)) {
        RCLCPP_ERROR(get_logger(), "rp.pitch_min_range / rp.pitch_max_range out of range (0 = off, min < max).");
        config_error_ = true;
    }
    if (rp_.pitch_push_frac < 0.7 || rp_.pitch_push_frac > 1.0 ||
        rp_.pitch_push_time < 0.04 || rp_.pitch_push_time > 0.5 ||rp_.roll_fast_profile > 200 || rp_.roll_center_profile > 200 || rp_.pitch_center_profile > 150 ||
        rp_.roll_sweep_profile > 80 || rp_.roll_final_profile > 60 ||
        rp_.roll_stop_current > 150 || rp_.roll_fast_current > 400 || rp_.roll_center_current > 400 ||
        rp_.confirm_time < 0.1 || rp_.roll_slow_margin < 500 ||
        rp_.pitch_contact_sat < 0.5 || rp_.pitch_contact_sat > 1.0 ||
        rp_.pitch_contact_ratio < 0.3 || rp_.pitch_contact_ratio > 0.95 ||
        rp_.pitch_fast_profile > 100 || rp_.pitch_sweep_profile > 40 || rp_.pitch_slow_margin < 300) {
        RCLCPP_ERROR(get_logger(), "rp.* values out of safe range (tool file or parameters).");
        config_error_ = true;
    }
}

void ToolHomingNode::rp_tick()
{
    const double now = now_s();   // jpos_/jvel_/jcur_ already filled by take_average()
    const int active_id = joint_limit_motor_ + 1;
    if (!rp_log_.is_open()) {
        try {
            std::filesystem::create_directories(jp_.log_dir);
            std::time_t t = std::time(nullptr);
            std::ostringstream name;
            name << jp_.log_dir << "/rollpitch_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << ".csv";
            rp_log_.open(name.str());
            rp_log_ << "t,motor,step,pos,vel,cur,goal,cruise\n";
        } catch (const std::exception&) {}
    }
    if (rp_log_.is_open())
        rp_log_ << std::fixed << std::setprecision(3) << now << "," << active_id << "," << homing_step_ << ","
                << jpos_[active_id] << "," << jvel_[active_id] << "," << jcur_[active_id] << ","
                << starting_locations_[joint_limit_motor_] << "," << rp_cruise_ << "\n";
    const int32_t current_pos = jpos_[active_id];
    const int32_t current_vel = jvel_[active_id];
    const int16_t current_ma = jcur_[active_id];
    const bool is_roll = (active_id == 1);
    // Every non-roll axis uses the pitch-style stop tests. Its values: pitch = rp.pitch_*, the jaw limit axis
    // (hook, homing.jaw_mode limits) = lim.*
    const bool is_lim = lim_active_;
    const std::string axis_name = is_roll ? "Roll" : (is_lim ? "Jaw M" + std::to_string(active_id) : "Pitch");
    const char* an = axis_name.c_str();
    const int ax_sweep_profile = is_lim ? lim_.sweep_profile : rp_.pitch_sweep_profile;
    const int ax_fast_profile = is_lim ? lim_.fast_profile : rp_.pitch_fast_profile;
    const double ax_expected_range = is_lim ? lim_.expected_range : rp_.pitch_expected_range;
    const double ax_slow_margin = is_lim ? lim_.slow_margin : rp_.pitch_slow_margin;
    const int ax_center_profile = is_lim ? lim_.center_profile : rp_.pitch_center_profile;
    const int ax_current_thresh = is_lim ? lim_.current_thresh : rp_.pitch_current_thresh;
    const int ax_sweep_current = is_lim ? lim_.sweep_current : rp_.pitch_sweep_current;
    const double ax_contact_sat = is_lim ? lim_.contact_sat : rp_.pitch_contact_sat;
    const double ax_contact_ratio = is_lim ? lim_.contact_ratio : rp_.pitch_contact_ratio;
    const double ax_push_frac = is_lim ? lim_.push_frac : rp_.pitch_push_frac;
    const double ax_push_time = is_lim ? lim_.push_time : rp_.pitch_push_time;

    if (homing_step_ < 2) {
        if (!phase_initialized_) {
            int profile = is_roll ? rp_.roll_sweep_profile : ax_sweep_profile;   // always explicit
            roll_slowed_ = true;
            if (is_roll) dxl_interface_->SetGoalCurrent(1, rp_.roll_stop_current);
            if (is_roll && homing_step_ == 1 && rp_.roll_expected_range > 0) {
                profile = rp_.roll_fast_profile;                  // fast toward limit 2 ...
                dxl_interface_->SetGoalCurrent(1, rp_.roll_fast_current);   // ... with more current (far from stops)
                roll_slowed_ = false;
            }
            if (!is_roll && homing_step_ == 1 && ax_expected_range > 0 && (is_lim || rp_.pitch_fast)) {
                profile = ax_fast_profile;                 // pitch / jaw: faster in the middle, SAME current cap
                roll_slowed_ = false;
            }
            slowed_t_ = -1;
            if (profile > 0) dxl_interface_->SetProfileVelocityId(active_id, profile);
            starting_locations_[joint_limit_motor_] = current_pos + (20000 * direction_);
            dxl_interface_->SetBulkPos(starting_locations_);
            phase_initialized_ = true;
            lim_phase_start_ = current_pos;
            phase_t0_ = now;
            limit_since_ = -1;
            rp_vh_.clear();
            rp_ih_.clear();
            pitch_contact_n_ = 0;
            pitch_push_hist_.clear();
            rp_cruise_ = 0;
            if (homing_step_ == 1)
                RCLCPP_INFO(get_logger(), "%s toward limit 2: %s", an,
                            roll_slowed_ ? "normal speed" : "FAST until near it");
        }
        // jaw limit axis: a sweep that never meets a stop (jaw disengaged / no stop there) must not run forever
        if (is_lim && std::abs(current_pos - lim_phase_start_) > lim_.max_travel) {
            lim_abort("sweep toward limit " + std::to_string(homing_step_ + 1) + " went " +
                      std::to_string(std::abs(current_pos - lim_phase_start_)) + " ticks without a stop (lim.max_travel " +
                      std::to_string(static_cast<int>(lim_.max_travel)) + ") -- jaw disengaged?");
            return;
        }
        // ... and back to the old speed before the expected limit 2
        if (homing_step_ == 1 && !roll_slowed_) {
            const double range = is_roll ? rp_.roll_expected_range : ax_expected_range;
            const double margin = is_roll ? rp_.roll_slow_margin : ax_slow_margin;
            double expected_l2 = limit_1 + direction_ * range;
            double remaining = (expected_l2 - current_pos) * direction_;
            if (remaining <= margin) {
                if (is_roll) dxl_interface_->SetGoalCurrent(1, rp_.roll_stop_current);   // gentle current first
                if (!is_roll) {
                    // Pitch is current-limited in the fast part when the tool is stiff (psm1: ~45 vs 60), so the
                    // motor's internal target runs ahead; it then kept chasing it at full current and hit limit 2
                    // at ~2x the slow speed. Reset the target to here first (profile 0 + goal = present position).
                    dxl_interface_->SetProfileVelocityId(active_id, 0);
                    starting_locations_[joint_limit_motor_] = current_pos;
                    dxl_interface_->SetBulkPos(starting_locations_);
                }
                dxl_interface_->SetProfileVelocityId(active_id, is_roll ? rp_.roll_final_profile : ax_sweep_profile);
                // a NEW goal (still beyond the stop) makes the motor re-plan from where it really is
                starting_locations_[joint_limit_motor_] = current_pos + direction_ * static_cast<int>(margin + 3000);
                dxl_interface_->SetBulkPos(starting_locations_);
                roll_slowed_ = true;
                slowed_t_ = now;
                // re-learn the cruising speed at the slow speed (the drop is not a contact)
                rp_vh_.clear();
                rp_ih_.clear();
                rp_cruise_ = 0;
                pitch_contact_n_ = 0;
                limit_since_ = -1;
                RCLCPP_INFO(get_logger(), "%s slowing down %.0f ticks before expected limit 2", an, remaining);
            }
        }

        const int current_thresh = is_roll ? rp_.roll_current_thresh : ax_current_thresh;
        // right after a slow-down the motor is still decelerating: don't learn cruise / test contact yet
        const bool settling = slowed_t_ >= 0 && now - slowed_t_ < 0.4;
        if (settling) { rp_vh_.clear(); rp_ih_.clear(); rp_cruise_ = 0; pitch_contact_n_ = 0; }
        // cruising speed = highest median |velocity| seen in this phase (like the jaw contact detector)
        rp_vh_.push_back(std::abs(static_cast<double>(current_vel)));
        if (rp_vh_.size() > 10) rp_vh_.pop_front();
        if (rp_vh_.size() == 10) {
            std::vector<double> v(rp_vh_.begin(), rp_vh_.end());
            std::sort(v.begin(), v.end());
            rp_cruise_ = std::max(rp_cruise_, (v[4] + v[5]) / 2.0);
        }
        const bool stopped = std::abs(current_vel) < (is_lim ? lim_.stall_velocity : rp_.stall_velocity);
        const bool slowing = rp_cruise_ > 0 && std::abs(current_vel) < 0.5 * rp_cruise_;
        // while roll/pitch runs FAST, speed/current changes are normal: only a real stop counts as a limit then
        const bool fast_now = homing_step_ == 1 && !roll_slowed_;
        // high current alone is NOT a limit (normal load bumps); it must come with a sharp slow-down
        const bool loaded = !fast_now && !settling && slowing && std::abs(current_ma) > current_thresh;
        const bool cond = (now - phase_t0_ > rp_.start_grace) && (stopped || loaded);
        if (cond) { if (limit_since_ < 0) limit_since_ = now; }
        else limit_since_ = -1;

        // PITCH: contact test -- torque-limited-ish AND slowing, 3 samples (no extra confirm wait)
        bool pitch_contact = false;
        bool pushed = false;
        if (!is_roll && !fast_now && !settling) {
            rp_ih_.push_back(std::abs(static_cast<double>(current_ma)));
            if (rp_ih_.size() > 3) rp_ih_.pop_front();
            if (rp_vh_.size() >= 3 && rp_ih_.size() == 3 && rp_cruise_ >= 5 && now - phase_t0_ > rp_.start_grace) {
                double v3 = (rp_vh_[rp_vh_.size() - 1] + rp_vh_[rp_vh_.size() - 2] + rp_vh_[rp_vh_.size() - 3]) / 3.0;
                double i3 = (rp_ih_[0] + rp_ih_[1] + rp_ih_[2]) / 3.0;
                bool hit = v3 < ax_contact_ratio * rp_cruise_ && i3 >= ax_contact_sat * ax_sweep_current;
                pitch_contact_n_ = hit ? pitch_contact_n_ + 1 : 0;
                pitch_contact = pitch_contact_n_ >= 3;
            }
            // PUSH test: sustained high current at sweep speed, even if the motor does not slow down
            pitch_push_hist_.emplace_back(now, std::abs(static_cast<double>(current_ma)));
            while (!pitch_push_hist_.empty() && now - pitch_push_hist_.front().first > ax_push_time)
                pitch_push_hist_.pop_front();
            if (!pitch_contact && pitch_push_hist_.size() >= 3 && now - phase_t0_ > rp_.start_grace) {
                double sum = 0;
                for (const auto& h : pitch_push_hist_) sum += h.second;
                if (sum / pitch_push_hist_.size() >= ax_push_frac * ax_sweep_current) {
                    pitch_contact = true;
                    pushed = true;
                    RCLCPP_WARN(get_logger(), "%s PUSH stop at %d: mean current %.0f over %.2f s while still moving "
                                "(%d) -- tool stiff or disc slipping?", an, current_pos, sum / pitch_push_hist_.size(),
                                ax_push_time, current_vel);
                }
            }
        } else {
            pitch_push_hist_.clear();
        }

        if (pitch_contact || (limit_since_ >= 0 && now - limit_since_ >= rp_.confirm_time)) {
            // hold right here: stop pushing the hard stop and reset the motor's internal target
            if (is_roll) dxl_interface_->SetGoalCurrent(1, rp_.roll_stop_current);
            // profile 0 + goal = present position resets the motor's internal trajectory (it can run ahead
            // of the real position when current-limited, which made the next move start late)
            dxl_interface_->SetProfileVelocityId(active_id, 0);
            starting_locations_[joint_limit_motor_] = current_pos;
            dxl_interface_->SetBulkPos(starting_locations_);
            if (homing_step_ == 0) {
                limit_1 = current_pos;
                if (is_roll) roll_lim1_ = current_pos;
                else if (is_lim) lim_lim1_ = current_pos;
                else pitch_lim1_ = current_pos;
                if (!pitch_contact && !stopped) soft_limit_axes_.insert(is_roll ? "roll" : (is_lim ? "jaw" : "pitch"));
                if (pushed) push_limit_axes_.insert(is_lim ? "jaw" : "pitch");
                RCLCPP_INFO(get_logger(), "Motor %d Limit 1 found: %d (%s)", active_id, limit_1,
                            pitch_contact ? "contact" : (stopped ? "stopped" : "current"));
                if (!is_roll && !is_lim) {
                    const int angle = ((limit_1 % 4096) + 4096) % 4096;
                    if (rp_.pitch_angle_tol > 0 && pitch_angle_expect_ >= 0) {
                        double d = std::fmod(angle - pitch_angle_expect_ + 6144.0, 4096.0) - 2048.0;   // -2048..2048
                        RCLCPP_INFO(get_logger(), "Pitch limit 1 disc angle %d (expected %.0f, off by %+.0f, tol %.0f)",
                                    angle, pitch_angle_expect_, d, rp_.pitch_angle_tol);
                        if (std::abs(d) > rp_.pitch_angle_tol) {
                            RCLCPP_ERROR(get_logger(), "Pitch limit 1 disc angle %d is %+.0f ticks from the expected %.0f "
                                         "(~half a turn = the pitch disc did NOT ENGAGE correctly or slipped). Homing STOPPED, "
                                         "motors holding. Remove and remount the tool, then restart.",
                                         angle, d, pitch_angle_expect_);
                            current_state_ = SetupState::JAW_ABORTED;
                            publish_status("ABORTED");
                            if (rp_log_.is_open()) { rp_log_ << "# STOPPED: pitch disc angle\n"; rp_log_.close(); }
                            return;
                        }
                    } else {
                        RCLCPP_INFO(get_logger(), "Pitch limit 1 disc angle %d (no check)", angle);
                    }
                }
                publish_limit_state(active_id, 1);
                direction_ = -direction_;
                homing_step_ = 1;
                phase_initialized_ = false;
            } else {
                limit_2_ = current_pos;
                const int min_range = is_roll ? 2000 : (is_lim ? static_cast<int>(lim_.min_range) : 500);
                if (std::abs(limit_2_ - limit_1) < min_range) {
                    RCLCPP_ERROR(get_logger(), "%s limit 2 (%d) is only %d ticks from limit 1 (%d): reversal failed. "
                                 "Homing STOPPED, motors holding. Ctrl+C and check the tool.",
                                 an, limit_2_, std::abs(limit_2_ - limit_1), limit_1);
                    current_state_ = SetupState::JAW_ABORTED;
                    publish_status("ABORTED");
                    if (rp_log_.is_open()) { rp_log_ << "# STOPPED: range too small\n"; rp_log_.close(); }
                    return;
                }
                if (!is_roll && !roll_slowed_)
                    RCLCPP_WARN(get_logger(), "%s limit 2 came EARLY (while fast): range %d vs expected %.0f",
                                an, std::abs(limit_2_ - limit_1), ax_expected_range);
                if (!is_roll && !is_lim) {
                    const double r = std::abs(limit_2_ - limit_1);
                    const bool too_short = rp_.pitch_min_range > 0 && r < rp_.pitch_min_range;
                    const bool too_long = rp_.pitch_max_range > 0 && r > rp_.pitch_max_range;
                    if (too_short || too_long) {
                        RCLCPP_ERROR(get_logger(), "Pitch range %.0f is too %s (allowed %.0f-%.0f, rp.pitch_min/max_range): "
                                     "%s Homing STOPPED, motors holding. Remove and remount the tool, then restart.",
                                     r, too_long ? "LONG" : "SHORT", rp_.pitch_min_range, rp_.pitch_max_range,
                                     too_long ? "the pitch disc probably slipped or did not engage correctly."
                                              : "probably a false stop before the real limit.");
                        current_state_ = SetupState::JAW_ABORTED;
                        publish_status("ABORTED");
                        if (rp_log_.is_open()) { rp_log_ << "# STOPPED: pitch range out of bounds\n"; rp_log_.close(); }
                        return;
                    }
                }
                if (is_lim) lim_lim2_ = current_pos;
                else if (!is_roll) pitch_lim2_ = current_pos;
                if (is_roll) {
                    roll_lim2_ = current_pos;
                    if (!roll_slowed_)   // limit came earlier than expected while still fast: say so
                        RCLCPP_WARN(get_logger(), "Roll limit 2 came EARLY (while fast): range %d vs expected %.0f",
                                    std::abs(limit_2_ - limit_1), rp_.roll_expected_range);
                }
                if (!pitch_contact && !stopped) soft_limit_axes_.insert(is_roll ? "roll" : (is_lim ? "jaw" : "pitch"));
                if (pushed) push_limit_axes_.insert(is_lim ? "jaw" : "pitch");
                RCLCPP_INFO(get_logger(), "Motor %d Limit 2 found: %d (%s)", active_id, limit_2_,
                            pitch_contact ? "contact" : (stopped ? "stopped" : "current"));
                publish_limit_state(active_id, 2);
                homing_step_ = 2;
                phase_initialized_ = false;
            }
        }
    }
    else if (homing_step_ == 2) {
        int32_t target_park_pos = (limit_1 + limit_2_) / 2;
        if (!phase_initialized_) {
            RCLCPP_INFO(get_logger(), "Centering Motor %d to calculated midpoint: %d", active_id, target_park_pos);
            dxl_interface_->SetProfileVelocityId(active_id, is_roll ? rp_.roll_center_profile : ax_center_profile);
            if (is_roll) dxl_interface_->SetGoalCurrent(1, rp_.roll_center_current);   // no stop on the way
            starting_locations_[joint_limit_motor_] = target_park_pos;
            dxl_interface_->SetBulkPos(starting_locations_);
            phase_initialized_ = true;
            center_ok_ = 0;
        }
        center_ok_ = (std::abs(target_park_pos - current_pos) < 15) ? center_ok_ + 1 : 0;
        if (center_ok_ >= 5) {
            if (is_lim) {          // jaw limit axis centered: homing done
                publish_limit_state(active_id, 3);
                lim_finish();
                return;
            }
            dxl_interface_->SetGoalCurrent(active_id, 1193);
            publish_limit_state(active_id, 3);
            if (joint_limit_motor_ == 0) {
                t_roll_done_ = now_s();
                RCLCPP_INFO(get_logger(), "Roll range %d ticks, roll homing took %.1f s",
                            std::abs(roll_lim2_ - roll_lim1_), t_roll_done_ - t_homing_start_);
                // Combined pitch engagement + homing: sweep Motor 4 directly.
                joint_limit_motor_ = 3;  // Motor 4 (pitch)
                direction_ = 1;          // Same positive direction as old engagement spin
                homing_step_ = 0;
                phase_initialized_ = false;
                dxl_interface_->SetProfileVelocity(22);  // Old engagement speed
                dxl_interface_->SetGoalCurrent(4, rp_.pitch_sweep_current);  // Existing homing current (100)
                RCLCPP_INFO(get_logger(), "Roll homed. Starting combined pitch engagement and homing...");
                if (rp_.free_jaws_during_pitch) set_jaws_free(true);
            } else {
                RCLCPP_INFO(get_logger(), "Motor 4 Homing complete. Roll and Pitch centered.");
                homing_step_ = 3;
                t_pitch_done_ = now_s();
                if (jaw_mode_ != "limits" && rp_log_.is_open()) rp_log_.close();   // limits: jaw rows follow
                RCLCPP_INFO(get_logger(), "Roll center: %d, Pitch center: %d (pitch took %.1f s)",
                            starting_locations_[0], starting_locations_[3], t_pitch_done_ - t_roll_done_);
                if (rp_.free_jaws_during_pitch) set_jaws_free(false);
                if (current_state_ == SetupState::ESTOP) return;
                if (jaw_mode_ == "limits") start_lim_axis();
                else start_jaw_homing();
            }
        }
    }
}


// =====================================================================================
//  JAW LIMIT AXIS (homing.jaw_mode limits): one jaw with hard mechanical stops (cautery hook, M2).
//  After pitch, rp_tick() runs the same 3 steps on the jaw motor with the lim.* values:
//    step 0: sweep to limit 1   step 1: reverse to limit 2   step 2: park at the midpoint
//  Same stop tests as pitch (stopped / loaded / contact / push), plus a max_travel guard.
// =====================================================================================
void ToolHomingNode::load_lim_params()
{
    auto P = [this](const std::string& n, auto def) { return this->declare_parameter("lim." + n, tool_value("lim." + n, def)); };
    lim_.motor = P("motor", lim_.motor);
    lim_.direction = P("direction", lim_.direction);
    lim_.sweep_current = P("sweep_current", lim_.sweep_current);
    lim_.sweep_profile = P("sweep_profile", lim_.sweep_profile);
    lim_.fast_profile = P("fast_profile", lim_.fast_profile);
    lim_.expected_range = P("expected_range", lim_.expected_range);
    lim_.slow_margin = P("slow_margin", lim_.slow_margin);
    lim_.center_profile = P("center_profile", lim_.center_profile);
    lim_.stall_velocity = P("stall_velocity", lim_.stall_velocity);
    lim_.current_thresh = P("current_thresh", lim_.current_thresh);
    lim_.contact_sat = P("contact_sat", lim_.contact_sat);
    lim_.contact_ratio = P("contact_ratio", lim_.contact_ratio);
    lim_.push_frac = P("push_frac", lim_.push_frac);
    lim_.push_time = P("push_time", lim_.push_time);
    lim_.min_range = P("min_range", lim_.min_range);
    lim_.max_travel = P("max_travel", lim_.max_travel);
    lim_.rp_hold = P("rp_hold", lim_.rp_hold);
    lim_.rp_final_hold = P("rp_final_hold", lim_.rp_final_hold);
    lim_.final_hold = P("final_hold", lim_.final_hold);
    if ((lim_.motor != 2 && lim_.motor != 5) || (lim_.direction != 1 && lim_.direction != -1) ||
        lim_.sweep_current < 30 || lim_.sweep_current > 100 ||
        lim_.sweep_profile < 5 || lim_.sweep_profile > 40 || lim_.fast_profile > 100 ||
        lim_.expected_range < 0 || lim_.expected_range > 8000 || lim_.slow_margin < 200 ||
        lim_.center_profile < 5 || lim_.center_profile > 100 ||
        lim_.stall_velocity < 1 || lim_.stall_velocity >= lim_.sweep_profile ||
        lim_.current_thresh < 20 || lim_.current_thresh > 150 ||
        lim_.contact_sat < 0.5 || lim_.contact_sat > 1.0 || lim_.contact_ratio < 0.3 || lim_.contact_ratio > 0.95 ||
        lim_.push_frac < 0.7 || lim_.push_frac > 1.0 || lim_.push_time < 0.04 || lim_.push_time > 0.5 ||
        lim_.min_range < 100 || lim_.min_range > 5000 || lim_.max_travel < 500 || lim_.max_travel > 8000 ||
        lim_.min_range >= lim_.max_travel || (lim_.expected_range > 0 && lim_.expected_range < lim_.min_range) ||
        lim_.rp_hold < 50 || lim_.rp_hold > 400 || lim_.rp_final_hold < 50 || lim_.rp_final_hold > 1193 ||
        lim_.final_hold < 0 || lim_.final_hold > 200) {
        RCLCPP_ERROR(get_logger(), "lim.* values out of safe range (tool file or parameters).");
        config_error_ = true;
    }
}

void ToolHomingNode::start_lim_axis()
{
    std::string hw = hw_error_report();
    if (!hw.empty()) {
        RCLCPP_ERROR(get_logger(), "Motor hardware error before jaw limit homing: %s Power-cycle/reboot the motors.",
                     hw.c_str());
        current_state_ = SetupState::JAW_ABORTED;
        publish_status("ABORTED");
        if (rp_log_.is_open()) { rp_log_ << "# STOPPED: hardware error\n"; rp_log_.close(); }
        return;
    }
    const int m = lim_.motor;
    dxl_interface_->SetGoalCurrent(1, lim_.rp_hold);
    dxl_interface_->SetGoalCurrent(4, lim_.rp_hold);
    dxl_interface_->SetGoalCurrent(m, lim_.sweep_current);
    dxl_interface_->SetProfileVelocityId(m, lim_.sweep_profile);
    lim_active_ = true;
    joint_limit_motor_ = m - 1;
    direction_ = lim_.direction;
    homing_step_ = 0;
    phase_initialized_ = false;
    limit_since_ = -1;
    center_ok_ = 0;
    RCLCPP_INFO(get_logger(), "Pitch homed. Homing jaw M%d between its hard limits (tool '%s', like pitch)...",
                m, tool_name_.c_str());
}

void ToolHomingNode::lim_abort(const std::string& why)
{
    const int m = lim_.motor;
    // hold right here: profile 0 + goal = present position (stops pushing, resets the internal target)
    dxl_interface_->SetProfileVelocityId(m, 0);
    starting_locations_[m - 1] = jpos_[m];
    dxl_interface_->SetBulkPos(starting_locations_);
    RCLCPP_ERROR(get_logger(), "Jaw M%d limit homing STOPPED: %s. Motors holding; Ctrl+C and check the tool.",
                 m, why.c_str());
    current_state_ = SetupState::JAW_ABORTED;
    publish_status("ABORTED");
    lim_active_ = false;
    if (rp_log_.is_open()) { rp_log_ << "# STOPPED: " << why << "\n"; rp_log_.close(); }
}

void ToolHomingNode::lim_finish()
{
    const int m = lim_.motor;
    lim_active_ = false;
    t_lim_done_ = now_s();
    dxl_interface_->SetGoalCurrent(1, lim_.rp_final_hold);
    dxl_interface_->SetGoalCurrent(4, lim_.rp_final_hold);
    dxl_interface_->SetGoalCurrent(m, lim_.final_hold);
    current_state_ = SetupState::JAW_DONE;
    publish_status("COMPLETE");
    if (rp_log_.is_open()) rp_log_.close();
    const int roll_range = std::abs(roll_lim2_ - roll_lim1_), pitch_range = std::abs(pitch_lim2_ - pitch_lim1_);
    const int jaw_range = std::abs(lim_lim2_ - lim_lim1_);
    RCLCPP_INFO(get_logger(), "JAW M%d LIMIT HOMING COMPLETE: center %d (limits %d / %d, range %d%s)", m,
                starting_locations_[m - 1], lim_lim1_, lim_lim2_, jaw_range,
                lim_.expected_range > 0 ? "" : "; lim.expected_range 0 = always slow");
    RCLCPP_INFO(get_logger(), "Ranges: roll %d, pitch %d, jaw M%d %d ticks", roll_range, pitch_range, m, jaw_range);
    RCLCPP_INFO(get_logger(), "All homing complete: roll %d, pitch %d, M%d %d.",
                starting_locations_[0], starting_locations_[3], m, starting_locations_[m - 1]);
    RCLCPP_INFO(get_logger(), "Time: roll %.1f s, pitch %.1f s, jaw %.1f s, total %.1f s",
                t_roll_done_ - t_homing_start_, t_pitch_done_ - t_roll_done_, t_lim_done_ - t_pitch_done_,
                t_lim_done_ - t_homing_start_);
    // one line per homing (the measured ranges are what a new tool's expected_range values come from)
    try {
        std::filesystem::create_directories(jp_.log_dir);
        std::string path = jp_.log_dir + "/lim_homing_summary.csv";
        bool fresh = !std::filesystem::exists(path);
        std::ofstream s(path, std::ios::app);
        if (fresh) s << "time,tool,adapter,roll_limit1,roll_limit2,roll_range,pitch_limit1,pitch_limit2,pitch_range,"
                        "jaw_motor,jaw_limit1,jaw_limit2,jaw_range,roll_center,pitch_center,jaw_center,"
                        "t_roll_s,t_pitch_s,t_jaw_s,t_total_s\n";
        std::time_t t = std::time(nullptr);
        s << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S") << "," << tool_name_ << "," << adapter_name_ << ","
          << roll_lim1_ << "," << roll_lim2_ << "," << roll_range << ","
          << pitch_lim1_ << "," << pitch_lim2_ << "," << pitch_range << ","
          << m << "," << lim_lim1_ << "," << lim_lim2_ << "," << jaw_range << ","
          << starting_locations_[0] << "," << starting_locations_[3] << "," << starting_locations_[m - 1] << ","
          << std::fixed << std::setprecision(1)
          << (t_roll_done_ - t_homing_start_) << "," << (t_pitch_done_ - t_roll_done_) << ","
          << (t_lim_done_ - t_pitch_done_) << "," << (t_lim_done_ - t_homing_start_) << "\n";
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Summary not written: %s", e.what());
    }
    write_home();
    if (calibrate_) cal_review();
}


// =====================================================================================
//  CALIBRATION (calibrate:=true): after a slow homing with verify pauses, propose new tool-file values
//  from ALL saved calibration runs of this tool (config/calibration/<tool>_history.csv + this run):
//    ranges  (rp.roll_expected_range, rp.pitch_expected_range, lim.expected_range): smallest seen - 50
//    offsets (<adapter>.m5_engaged_start / m2_contact_offset / m5_step_offset / m2_step_offset): mean on that adapter
//  A = save (history row + tool file, backup first), B = discard (nothing written).
// =====================================================================================
static const char* const kCalCols[] = {"time", "tool", "adapter", "jaw_mode", "roll_range", "pitch_range", "jaw_range",
                                       "m5_mode", "m5_step_offset", "m5_engaged_start", "m5_error",
                                       "m2_mode", "m2_step_offset", "m2_contact_offset", "m2_error", "flags",
                                       "pitch_angle", "m5_center_angle", "m2_center_angle"};

static std::vector<std::string> split_csv(const std::string& line)
{
    std::vector<std::string> out;
    std::string cur;
    bool q = false;
    for (char c : line) {
        if (c == '"') q = !q;
        else if (c == ',' && !q) { out.push_back(cur); cur.clear(); }
        else cur += c;
    }
    out.push_back(cur);
    return out;
}

static std::string cal_history_path(const std::string& dir, const std::string& tool)
{
    return dir + "/calibration/" + tool + "_history.csv";
}

void ToolHomingNode::cal_review()
{
    cal_row_.clear();
    cal_updates_.clear();
    std::time_t t = std::time(nullptr);
    std::ostringstream ts;
    ts << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S");
    auto I = [](double v) { return std::to_string(static_cast<long>(std::lround(v))); };
    cal_row_["time"] = ts.str();
    cal_row_["tool"] = tool_name_;
    cal_row_["adapter"] = adapter_name_;
    cal_row_["jaw_mode"] = jaw_mode_;
    if (!soft_limit_axes_.count("roll")) cal_row_["roll_range"] = I(std::abs(roll_lim2_ - roll_lim1_));
    if (!soft_limit_axes_.count("pitch") && !push_limit_axes_.count("pitch"))
        cal_row_["pitch_range"] = I(std::abs(pitch_lim2_ - pitch_lim1_));
    if (!soft_limit_axes_.count("pitch"))   // the disc angle of pitch limit 1 (engagement check)
        cal_row_["pitch_angle"] = std::to_string(((pitch_lim1_ % 4096) + 4096) % 4096);
    if (jaw_mode_ == "limits") {
        if (!soft_limit_axes_.count("jaw") && !push_limit_axes_.count("jaw"))
            cal_row_["jaw_range"] = I(std::abs(lim_lim2_ - lim_lim1_));
    } else {
        // definitions match the homing code: step center = step - dir*offset, engaged center = start - dir*offset,
        // contact check = contact - dir*m2_contact_offset (M2 parks AT the contact unless m2_apply_contact_offset)
        cal_row_["m5_mode"] = mode_step_[5] ? "step" : "engaged";
        cal_row_["m2_mode"] = mode_step_[2] ? "step" : "contact";
        if (has_true_[5]) {
            if (mode_step_[5]) cal_row_["m5_step_offset"] = I((step_pos_[5] - true_center_[5]) * jdir(5));
            else cal_row_["m5_engaged_start"] = I((sweep_start_[5] - true_center_[5]) * jdir(5));
            cal_row_["m5_error"] = I(robot_center_[5] - true_center_[5]);
            cal_row_["m5_center_angle"] = std::to_string(((true_center_[5] % 4096) + 4096) % 4096);
        }
        if (has_true_[2]) {
            if (mode_step_[2]) cal_row_["m2_step_offset"] = I((step_pos_[2] - true_center_[2]) * jdir(2));
            cal_row_["m2_contact_offset"] = I((contact_pos_ - true_center_[2]) * jdir(2));
            cal_row_["m2_error"] = I(robot_center_[2] - true_center_[2]);
            cal_row_["m2_center_angle"] = std::to_string(((true_center_[2] % 4096) + 4096) % 4096);
        }
        std::string flags;
        for (const auto& f : jaw_flags_) flags += (flags.empty() ? "" : " | ") + f;
        cal_row_["flags"] = flags;
    }

    // all saved runs of this tool + this one
    std::vector<std::map<std::string, std::string>> runs;
    {
        std::ifstream f(cal_history_path(config_dir_, tool_name_));
        std::string line;
        std::vector<std::string> head;
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            auto v = split_csv(line);
            if (head.empty()) { head = v; continue; }
            std::map<std::string, std::string> r;
            for (size_t i = 0; i < head.size() && i < v.size(); ++i) r[head[i]] = v[i];
            runs.push_back(r);
        }
    }
    runs.push_back(cal_row_);
    auto values = [&](const std::string& col, const std::string& adapter) {
        std::vector<double> out;
        for (auto& r : runs) {
            if (!adapter.empty() && r["adapter"] != adapter) continue;
            if (r[col].empty()) continue;
            try { out.push_back(std::stod(r[col])); } catch (const std::exception&) {}
        }
        return out;
    };
    auto list = [](const std::vector<double>& v) {
        std::string s;
        for (double x : v) s += (s.empty() ? "" : " ") + std::to_string(static_cast<long>(std::lround(x)));
        return s;
    };
    auto propose_min = [&](const std::string& key, const std::string& col) {
        // a limit found by the current test while the motor was still moving may be short of the real stop
        const std::string axis = col.substr(0, col.find('_'));
        if (soft_limit_axes_.count(axis)) {
            RCLCPP_WARN(get_logger(), "  %-26s NOT updated: a %s limit was found by CURRENT while still moving "
                        "(not a stop), so this run's %s range may be too SHORT.", key.c_str(), axis.c_str(), axis.c_str());
            return;
        }
        if (push_limit_axes_.count(axis)) {
            // full speed + high current = grinding PAST the stop (needle driver psm2: 215 ticks past) -> too LONG,
            // the dangerous direction for an expected range
            RCLCPP_WARN(get_logger(), "  %-26s NOT updated: a %s limit was a PUSH stop (motor still moving at full "
                        "speed), so this run's %s range may be too LONG.", key.c_str(), axis.c_str(), axis.c_str());
            return;
        }
        auto v = values(col, "");
        if (v.empty()) return;
        double m = *std::min_element(v.begin(), v.end()) - 50;
        if (m <= 0) return;
        // a range only ever goes DOWN: a larger value than the file's could make the slow-down come too late
        // (the file may hold the smallest of many earlier, hand-measured runs). Raise it by hand if needed.
        double cur = 0;
        auto it = tool_cfg_.find(key);
        if (it != tool_cfg_.end()) { try { cur = std::stod(it->second); } catch (const std::exception&) {} }
        if (cur > 0 && cur <= m) {
            RCLCPP_INFO(get_logger(), "  %-26s %8.0f kept (smallest - 50 of %zu run(s) = %.0f is not smaller: %s)",
                        key.c_str(), cur, v.size(), m, list(v).c_str());
            return;
        }
        cal_updates_.push_back({key, I(m), "calibrated " + ts.str().substr(0, 10) + ": smallest - 50 of " +
                                std::to_string(v.size()) + " run(s): " + list(v)});
    };
    auto propose_mean = [&](const std::string& key, const std::string& col, const std::string& adapter) {
        auto v = values(col, adapter);
        if (v.empty()) return;
        double sum = 0;
        for (double x : v) sum += x;
        cal_updates_.push_back({key, I(sum / v.size()), "calibrated " + ts.str().substr(0, 10) + ": mean of " +
                                std::to_string(v.size()) + " run(s)" + (adapter.empty() ? "" : " on " + adapter) +
                                ": " + list(v)});
    };
    current_state_ = SetupState::CAL_REVIEW;
    publish_status("CAL_REVIEW");
    RCLCPP_WARN(get_logger(), "================ CALIBRATION RESULT: %s on %s ================",
                tool_name_.c_str(), adapter_name_.c_str());
    std::string meas;
    for (const char* c : kCalCols) {
        const std::string k = c;
        if (k == "time" || k == "tool" || k == "adapter" || k == "flags" || cal_row_[k].empty()) continue;
        meas += k + " " + cal_row_[k] + ", ";
    }
    RCLCPP_INFO(get_logger(), "This run: %s", meas.c_str());
    if (!cal_row_["flags"].empty())
        RCLCPP_WARN(get_logger(), "JAW FLAGS this run (check before saving): %s", cal_row_["flags"].c_str());
    for (const auto& kc : std::vector<std::pair<std::string, std::string>>{{"pitch_limit1_angle", "pitch_angle"},
                                                                         {"m5_center_angle", "m5_center_angle"},
                                                                         {"m2_center_angle", "m2_center_angle"}}) {
        // <adapter>.<angle key>: circular mean of this adapter's angles (positions mod 4096)
        if (kc.first == "m5_center_angle" && jaw_mode_ != "contact") continue;
        // M2 angle: opt-in, only updated where the adapter already has the line
        if (kc.first == "m2_center_angle" && !tool_cfg_.count(adapter_name_ + ".m2_center_angle")) continue;
        auto v = values(kc.second, adapter_name_);
        if (!v.empty()) {
            double sx = 0, sy = 0;
            for (double a : v) { sx += std::cos(a / 4096.0 * 2 * M_PI); sy += std::sin(a / 4096.0 * 2 * M_PI); }
            double m = std::atan2(sy, sx) / (2 * M_PI) * 4096.0;
            if (m < 0) m += 4096.0;
            cal_updates_.push_back({adapter_name_ + "." + kc.first, I(std::fmod(m, 4096.0)),
                                    "calibrated " + ts.str().substr(0, 10) + ": mean of " + std::to_string(v.size()) +
                                    " run(s) on " + adapter_name_ + ": " + list(v)});
        }
    }
    propose_min("rp.roll_expected_range", "roll_range");
    propose_min("rp.pitch_expected_range", "pitch_range");
    if (jaw_mode_ == "limits") {
        propose_min("lim.expected_range", "jaw_range");
    } else {
        // per adapter (the start positions differ per adapter); the tool-wide jaw.* stays the default for others
        propose_mean(adapter_name_ + ".m5_engaged_start", "m5_engaged_start", adapter_name_);
        propose_mean(adapter_name_ + ".m2_contact_offset", "m2_contact_offset", adapter_name_);
        propose_mean(adapter_name_ + ".m5_step_offset", "m5_step_offset", adapter_name_);
        propose_mean(adapter_name_ + ".m2_step_offset", "m2_step_offset", adapter_name_);
    }

    if (cal_updates_.empty()) RCLCPP_WARN(get_logger(), "Nothing measured to save.");
    for (const auto& u : cal_updates_) {
        auto it = tool_cfg_.find(u.key);
        std::string cur = it == tool_cfg_.end() ? "(none)" : it->second;
        if (it == tool_cfg_.end() && u.key.find('.') != std::string::npos) {   // per-adapter key: show the default used
            auto def = tool_cfg_.find("jaw." + u.key.substr(u.key.find('.') + 1));
            if (def != tool_cfg_.end()) cur = "(jaw. " + def->second + ")";
        }
        RCLCPP_INFO(get_logger(), "  %-26s %10s -> %-6s (%s)", u.key.c_str(), cur.c_str(), u.value.c_str(), u.note.c_str());
    }
    RCLCPP_WARN(get_logger(), "A = SAVE this run to the history and these values to %s/%s.txt (backup first). "
                "B = DISCARD this run. Motors are holding.", config_dir_.c_str(), tool_name_.c_str());
}

void ToolHomingNode::cal_save()
{
    const std::string tool_path = config_dir_ + "/" + tool_name_ + ".txt";
    std::time_t t = std::time(nullptr);
    std::ostringstream stamp;
    stamp << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
    try {
        std::filesystem::create_directories(config_dir_ + "/calibration");
        // 1) backup of the tool file
        std::string backup = config_dir_ + "/calibration/" + tool_name_ + ".txt." + stamp.str() + ".bak";
        for (int k = 2; std::filesystem::exists(backup); ++k)          // never overwrite an older backup
            backup = config_dir_ + "/calibration/" + tool_name_ + ".txt." + stamp.str() + "_" + std::to_string(k) + ".bak";
        std::filesystem::copy_file(tool_path, backup);
        // 2) history row
        const std::string hist = cal_history_path(config_dir_, tool_name_);
        bool fresh = !std::filesystem::exists(hist);
        {
            // an older history file with fewer columns: rewrite it with the current columns first
            std::string want;
            for (const char* c : kCalCols) want += (want.empty() ? "" : ",") + std::string(c);
            if (!fresh) {
                std::ifstream f(hist);
                std::string head, line;
                std::getline(f, head);
                if (head != want) {
                    auto hv = split_csv(head);
                    std::vector<std::string> out{want};
                    while (std::getline(f, line)) {
                        if (line.empty()) continue;
                        auto v = split_csv(line);
                        std::map<std::string, std::string> r;
                        for (size_t i = 0; i < hv.size() && i < v.size(); ++i) r[hv[i]] = v[i];
                        std::string row;
                        bool first = true;
                        for (const char* c : kCalCols) {
                            std::string x = r[c];
                            if (x.find(',') != std::string::npos) x = "\"" + x + "\"";
                            row += (first ? "" : ",") + x;
                            first = false;
                        }
                        out.push_back(row);
                    }
                    f.close();
                    std::ofstream o(hist);
                    for (const auto& l : out) o << l << "\n";
                }
            }
            std::ofstream h(hist, std::ios::app);
            if (fresh) {
                std::string head;
                for (const char* c : kCalCols) head += (head.empty() ? "" : ",") + std::string(c);
                h << head << "\n";
            }
            std::string row;
            bool first = true;
            for (const char* c : kCalCols) {
                std::string v = cal_row_[c];
                if (v.find(',') != std::string::npos) v = "\"" + v + "\"";
                row += (first ? "" : ",") + v;
                first = false;
            }
            h << row << "\n";
            if (!h) throw std::runtime_error("cannot write " + hist);
        }
        // 3) tool file: replace each key's line (value + note), append keys it does not have yet
        std::vector<std::string> lines;
        {
            std::ifstream f(tool_path);
            std::string line;
            while (std::getline(f, line)) lines.push_back(line);
        }
        auto fmt = [](const CalUpdate& u) {
            std::string kv = u.key + " " + u.value;
            if (kv.size() < 33) kv += std::string(33 - kv.size(), ' ');
            return kv + "# " + u.note;
        };
        std::vector<CalUpdate> appended;
        for (const auto& u : cal_updates_) {
            bool done = false;
            for (auto& line : lines) {
                std::istringstream ss(line.substr(0, line.find('#')));
                std::string k;
                if ((ss >> k) && k == u.key) { line = fmt(u); done = true; break; }
            }
            if (!done) appended.push_back(u);
        }
        if (!appended.empty()) {
            const std::string head = "# ---------------- calibration: per adapter ----------------";
            bool has_head = false;
            for (auto& l : lines) has_head = has_head || l == head;
            if (!has_head) { lines.push_back(""); lines.push_back(head); }
            for (const auto& u : appended) lines.push_back(fmt(u));
        }
        {
            const std::string tmp = tool_path + ".tmp";
            std::ofstream o(tmp);
            for (const auto& l : lines) o << l << "\n";
            o.close();
            if (!o) throw std::runtime_error("cannot write " + tmp);
            std::filesystem::rename(tmp, tool_path);
        }
        RCLCPP_WARN(get_logger(), "CALIBRATION SAVED: %zu value(s) in %s (backup %s), run added to %s.",
                    cal_updates_.size(), tool_path.c_str(), backup.c_str(), hist.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Calibration NOT saved: %s", e.what());
    }
    current_state_ = SetupState::JAW_DONE;
    publish_status("COMPLETE");
}

void ToolHomingNode::cal_discard()
{
    RCLCPP_WARN(get_logger(), "Calibration run DISCARDED: nothing written.");
    current_state_ = SetupState::JAW_DONE;
    publish_status("COMPLETE");
}
