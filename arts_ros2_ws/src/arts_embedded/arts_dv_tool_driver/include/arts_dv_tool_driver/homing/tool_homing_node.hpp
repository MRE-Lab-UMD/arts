#pragma once
// Automatic instrument homing node (the adapter designer's, ported; see src/homing/tool_homing_node.cpp).
 #define RAD_PER_SEC_TO_RPM  30.0/3.141592
 #define RPM_TO_CMD          1.0/0.229

#include <chrono>
#include <deque>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>


#include "arts_dv_tool_driver/homing/dxl_interface.hpp"
#include "arts_dv_tool_driver/homing/jaw_homing.hpp"



using std::placeholders::_1;
using namespace std::chrono_literals;

//all the states, which is used for button input on joy controller
enum class SetupState {
    GO_START_POS,
    WAIT_FOR_TOOL,
    SPIN,
    DELAY_BEFORE_HOMING,
    LIMITS,
    JAW_SWEEP,      // jaw opening sweep (engagement + step / engaged detection)
    JAW_GOTO,       // jaw position move (M5 back to the middle)
    JAW_CLOSE,      // jaw closes until it touches the other jaw
    JAW_VERIFY,     // (jaw.verify) operator nudges a jaw to its true center, then A
    JAW_DONE,       // roll, pitch and jaws homed
    JAW_ABORTED,    // jaw homing stopped safely; motors holding
    CAL_REVIEW,     // (calibrate) homing done, proposed tool-file values shown: A = save, B = discard
    JOY_TELEOP,
    KINEMATICS,
    ESTOP
};

// Jaw homing parameters (ROS parameters "jaw.*"). Defaults = forceps_01 calibration, Nov 2026.
struct JawParams {
    int sweep_current = 75;            // goal current during jaw sweeps/closes (raw units)
    int sweep_profile_velocity = 22;
    int move_profile_velocity = 60;
    int move_fast_profile = 100;       // M5 back to the middle: fast part ...
    int move_fast_current = 150;       // ... with more current (the jaw is current-limited at 75)
    int move_slow_margin = 200;        // last ticks before the target at move_current / move_profile_velocity
    int move_current = 75;             // M5 back to the middle, gentle part (scissors need more: blade friction)
    int dir_m2 = 1, dir_m5 = 1;        // +1: outward (opening) is positive
    int max_sweep = 5000;              // ticks
    int max_close = 1800;
    int stall_velocity = 3;            // |velocity| below this = not moving
    double stall_time = 0.25;          // s
    double m5_step_offset = 107;       // step position - center (ticks, outward)
    double m2_step_offset = 139;
    double m2_contact_offset = -4;     // contact position - center when M5 is centered
    bool m2_apply_contact_offset = false;
    double m5_center_snap_max = 300;        // <adapter>.m5_center_angle snap only if the step estimate is within this
                                            // (needle driver PSM1: one instrument snapped <= 256, another >= 388 = its own
                                            // jaw angle -> farther than this: trust the step estimate instead)
    double m2_center_snap_max = 300;        // same for M2 and <adapter>.m2_center_angle (after the contact)   // true: after the contact M2 moves to contact - offset (needle driver:
                                            // contact fires ~200 ticks past the true touch); false: parks AT contact
    double m5_engaged_start = 63;      // engaged M5 sweep start - center
    double engaged_trend = 1.10;       // M5
    double m2_engaged_trend = 1.06;    // M2 (lower: calling M2 engaged is harmless)
    double m5_engaged_mid = 1.025;     // M5 engaged also needs the climb to have started by 300-450 ticks
                                       // (else it is a step beginning: keep sweeping and confirm it)
    double m5_defer_travel = 1300;     // ... but if no step is confirmed by this travel, fall back to engaged
    double goto_push_ratio = 0.15;     // M5 back to the middle: at the current limit AND slower than this x
                                       // move_profile_velocity (5-sample mean, i.e. < 9) = nearly stalled on
                                       // something -> stop. NOT 0.4: on psm1 the gentle part normally runs
                                       // steadily at ~18-20 at the 75 limit (09:20 run false-aborted at 0.4).
    double stop_after_step = 450;
    double check_tol = 80;
    int wall_current = 150;            // other jaw's goal current while one jaw closes onto it
    double collapse_ratio = 0.8;       // contact: speed < ratio x cruising speed while torque-limited
    double joystick_timeout = 0.5;     // s: automatic jaw motion stops without joystick messages
    int rp_hold_during_jaws = 100;     // roll/pitch goal current during jaw homing (as in the data)
    int rp_final_hold = 1193;          // roll/pitch goal current after homing (previous behavior)
    int jaw_final_hold = 75;
    std::string log_dir = "jaw_homing_logs";
    bool verify = false;               // pause after each jaw so the operator can record its true center
    double jog_speed = 300;            // ticks/s at full stick (verify mode)
    int m2_axis = 1, m5_axis = 4;      // left / right stick vertical
    double deadband = 0.2;
};

// Roll/pitch homing parameters (ROS parameters "rp.*").
struct RpParams {
    int stall_velocity = 5;            // |velocity| units below this = stopped (old rule: <4 ticks per ~160 ms loop)
    double confirm_time = 0.25;        // s the stop condition must hold (old: ~0.65 s at 6 Hz)
    double start_grace = 0.6;          // s ignored after each start/reversal (motor accelerating)
    int roll_current_thresh = 85;      // unchanged from before
    int pitch_current_thresh = 65;     // unchanged from before
    int roll_sweep_profile = 55;       // first roll sweep (unknown limit 1); current stays capped at 100 (was 40)
    int roll_fast_profile = 200;       // limit1 -> limit2 fast section (needs the higher current below)
    int roll_stop_current = 100;       // roll current whenever it may reach a hard stop (unchanged 100)
    int roll_fast_current = 350;       // roll current in the fast section only (far from the stops); ~275 needed at 200
    int roll_center_current = 350;     // roll current while centering (never touches a stop)
    int roll_final_profile = 40;       // final approach to limit 2 (old speed)
    double roll_expected_range = 7592; // ticks between roll limits (measured 7592-7597); 0 = never go fast
    double roll_slow_margin = 1000;    // slow down this many ticks before the expected limit 2
    int roll_center_profile = 200;     // centering never touches a stop
    int pitch_center_profile = 60;
    // pitch stop = contact test (like the jaws): current >= sat x sweep current AND speed < ratio x cruising,
    // for 3 samples. Catches contact ~0.35 s / ~30 ticks earlier -> less pull on the shaft.
    int pitch_sweep_current = 100;
    int pitch_sweep_profile = 22;          // pitch sweeps and final approach (unchanged)
    int pitch_fast_profile = 60;           // pitch limit1 -> limit2 middle section (current cap stays 100)
    double pitch_expected_range = 1704;    // ticks between pitch limits (measured 1702-1706); 0 = never fast
    double pitch_slow_margin = 450;        // slow down this many ticks before the expected pitch limit 2
    double pitch_contact_sat = 0.75;
    double pitch_contact_ratio = 0.85;
    // pitch stop = PUSH test (no speed drop needed): mean |current| over pitch_push_time >= pitch_push_frac x
    // sweep current, at sweep speed. Catches a stop the motor keeps grinding through at full speed
    // (psm1 08:31 run: 1778 ticks past limit 1). Normal sweeping peaks at ~0.80 over 0.1 s.
    double pitch_push_frac = 0.85;
    double pitch_push_time = 0.10;     // s
    // torque OFF on the jaw motors (M2, M5) while pitch homes, so the cable coupling back-drives them instead
    // of twisting the jaws (scissors on psm1 ended up very angled). Re-enabled at their new positions after.
    bool free_jaws_during_pitch = false;
    // false: pitch never runs fast (needle driver: fast pitch is current-limited at the 100 cap, and the push
    // test then fires FALSE ~300 ticks after the slow-down -> short range, pitch off center)
    bool pitch_fast = true;
    // pitch range sanity check (0 = off): outside min..max -> homing stops, motors hold. Catches a disc in
    // a badly engaged / slipping disc (range too long) and false stops (too short).
    double pitch_min_range = 0;
    double pitch_max_range = 0;
    // engagement check: pitch limit 1 position modulo one turn (4096) = the adapter disc's angle at the stop.
    // It repeats unless the disc did not engage correctly (then ~2048 off). Expected angle per adapter in the tool
    // file (<adapter>.pitch_limit1_angle, set by calibration); off by more than this -> STOP. 0 = check off.
    double pitch_angle_tol = 0;
};

// Jaw LIMIT axis (ROS parameters "lim.*"), used when the tool file says "homing.jaw_mode limits":
// a single jaw with hard mechanical stops (cautery hook: M2) is homed like pitch -- limit 1, limit 2,
// park at the midpoint -- with its own values. Defaults are only placeholders: the tool file must give them all.
struct LimParams {
    int motor = 2;                     // jaw motor id (2 or 5)
    int direction = 1;                 // first sweep direction (+1 / -1)
    int sweep_current = 75;            // goal current while it may reach a stop
    int sweep_profile = 22;            // sweeps and final approach to limit 2
    int fast_profile = 60;             // limit1 -> limit2 middle section (current cap stays sweep_current)
    double expected_range = 0;         // ticks between the stops; 0 = never fast (until measured)
    double slow_margin = 450;          // slow down this many ticks before the expected limit 2
    int center_profile = 60;
    int stall_velocity = 3;            // |velocity| below this = stopped (own value: the jaw sweeps slower than pitch)
    int current_thresh = 65;          // 'loaded' stop test: current above this AND slowing sharply
    double contact_sat = 0.75;         // contact test: current >= sat x sweep_current ...
    double contact_ratio = 0.85;       // ... AND speed < ratio x cruising, 3 samples
    double push_frac = 0.85;           // push test: mean current >= frac x sweep_current over push_time
    double push_time = 0.10;           // s
    double min_range = 300;            // limit 2 closer than this to limit 1 = reversal failed -> stop
    double max_travel = 5000;          // a sweep longer than this without a stop -> stop (disengaged?)
    int rp_hold = 100;                 // roll/pitch goal current while the jaw homes
    int rp_final_hold = 1193;          // roll/pitch goal current after homing
    int final_hold = 75;               // jaw goal current after homing
};

class ToolHomingNode : public rclcpp::Node
{
public:
    ToolHomingNode();
    ~ToolHomingNode();

private:
    int starting_locations_[5] = {100, 5000, 2100, 3748, 1800};   // M4 rotated 180 deg (was 1700)
    int current_spin_motor_ = 0;
    int motors_locked_in_ = 0;
    bool prev_button_a_ = false;
    bool prev_button_b_ = false;
    bool prev_button_y_ = false;
    int32_t prev_spin_pos_ = 0;
    int spin_stall_count_ = 0;

    bool prev_button_x_ = false;
    int joint_limit_motor_ = 0;
    int32_t limit_1 = 0;
    int32_t limit_2_ = 0;
    int32_t limit_2_m2_ = 0; // Added variable to track Motor 2's pushed limit
    int32_t prev_pos_ = 0;
    int stall_count_ = 0;
    int direction_ = -1;
    int homing_step_ = 0; // 0 = find limit 1, 1 = find limit 2, 2 = center
    bool phase_initialized_ = false;

    rclcpp::Time spin_complete_time_;

    void publish_limit_state(int axis_id, int state);
    // GUI interface: latched status (MOVING_TO_START, WAIT_FOR_TOOL, HOMING, COMPLETE, ABORTED, ESTOP)
    // and an 'A button' topic (std_msgs/Empty on gui_a) that acts exactly like a joystick A press.
    void publish_status(const std::string& s);
    void handle_a_press();
    void set_jaws_free(bool free);
    void emergency_stop(const char* source);
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr gui_heartbeat_sub_, gui_estop_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr gui_a_sub_;

    // ---------------- jaw homing ----------------
    JawParams jp_;
    int jaw_plan_step_ = 0;            // 0 sweep M5, 1 M5 to middle, 2 sweep M2, 3 close M2, 4 done
    int jaw_m_ = 0;                    // active jaw motor id (2 or 5)
    double last_gui_heartbeat_ = -1e9;        // GUI heartbeat (gui_heartbeat) also satisfies the deadman
    double jaw_t0_ = 0, last_jaw_tick_ = 0, last_joy_time_ = -1e9, stall_since_ = -1, goto_deadline_ = 0;
    // verify mode
    std::vector<int> jaw_plan_;              // 0 sweep5, 1 goto5, 2 sweep2, 3 close2, 5 verify5, 6 verify2, 9 finish
    int verify_m_ = 0;
    double verify_goal_ = 0;
    float axis_m2_ = 0, axis_m5_ = 0;
    bool verify_confirm_ = false;
    int32_t robot_center_[6] = {0}, true_center_[6] = {0};
    bool has_true_[6] = {false};
    double step_pos_[6] = {0};
    void jaw_start_verify(int m);
    void jaw_tick_verify(double dt);
    bool operator_stop_ = false;
    int32_t jpos_[6] = {0}, jvel_[6] = {0};   // index = motor id (1..5)
    int16_t jcur_[6] = {0};
    std::deque<double> jaw_dt_;              // loop-rate guard
    // fast I/O (333 Hz): raw latest sample + accumulators averaged into each 50 Hz logic tick
    double control_hz_ = 333.0;
    int32_t raw_pos_[6] = {0}, raw_vel_[6] = {0};
    int16_t raw_cur_[6] = {0};
    double acc_vel_[6] = {0}, acc_cur_[6] = {0};
    int acc_n_ = 0;
    int read_fail_ = 0;
    bool raw_valid_ = false;
    double io_count_ = 0, io_t0_ = 0;        // measured I/O rate
    bool read_raw();
    void take_average();
    double jaw_prev_tick_ = 0;
    // per jaw motor (index = motor id)
    int32_t sweep_start_[6] = {0}, sweep_stop_[6] = {0};
    std::string sweep_why_[6];
    bool mode_step_[6] = {false};
    bool has_step_est_[6] = {false};
    double step_est_[6] = {0};
    bool slip_[6] = {false};
    // sweep
    jaw_homing::StepFinder sweep_sf_;
    jaw_homing::StepResult sweep_res_;
    jaw_homing::ContactDetector sweep_det_;
    std::deque<double> sweep_ih_;
    int sweep_n_ = 0;
    // goto
    int32_t goto_target_ = 0;           // current (possibly intermediate) target
    int32_t goto_final_ = 0;            // final target
    bool goto_fast_ = false;            // in the fast part of the move
    int goto_ok_ = 0;
    std::deque<double> goto_push_v_;          // |velocity| while M5 is at its current limit (back to middle)
    bool engaged_deferred_ = false;           // M5 early climb looked like a step starting: kept sweeping
    std::string goto_label_;
    // close
    jaw_homing::ContactDetector close_det_;
    int32_t close_start_ = 0;
    double close_max_travel_ = 0;
    bool close_check_ = false;
    double check_diff_ = 0;
    bool has_check_ = false;
    int32_t contact_pos_ = 0;
    std::string close_why_;
    std::vector<std::string> jaw_flags_;
    std::ofstream jaw_log_;
    // roll/pitch 50 Hz loop
    RpParams rp_;
    double rp_last_tick_ = 0, phase_t0_ = 0, limit_since_ = -1;
    std::deque<double> rp_vh_;               // recent |velocity| for cruising speed
    std::deque<double> rp_ih_;               // recent |current| (pitch contact test)
    int pitch_contact_n_ = 0;
    std::deque<std::pair<double, double>> pitch_push_hist_;   // (time, |current|) for the pitch push test
    double rp_cruise_ = 0;
    std::ofstream rp_log_;
    int center_ok_ = 0;
    bool roll_slowed_ = false;               // (roll or pitch) true = not in a fast section
    double slowed_t_ = -1;                   // time of the slow-down (short grace for the stop tests)
    void load_rp_params();
    void rp_tick();
    // jaw limit axis (homing.jaw_mode limits)
    std::string jaw_mode_ = "contact";       // "contact" (forceps, scissors) or "limits" (hook)
    std::string adapter_name_;
    LimParams lim_;
    bool lim_active_ = false;                // rp_tick is running the jaw limit axis (not pitch)
    int32_t lim_phase_start_ = 0;            // position at the start of the current sweep (max_travel guard)
    int32_t lim_lim1_ = 0, lim_lim2_ = 0, pitch_lim1_ = 0, pitch_lim2_ = 0;
    double t_lim_done_ = 0;
    void load_lim_params();
    void start_lim_axis();
    void lim_finish();
    void lim_abort(const std::string& why);
    // calibration mode (calibrate:=true): slow homing + verify pauses, then the measured values go into
    // config/<tool>.txt (after A in CAL_REVIEW). History of saved runs: config/calibration/<tool>_history.csv
    bool calibrate_ = false;
    bool slow_ = false;                                   // slow:=true (or calibrate): no fast sections anywhere
    bool slow_jaws_ = false;
    double pitch_angle_expect_ = -1;
    // <adapter>.m5_center_angle: M5's true jaw center as a motor angle (position mod 4096). The keyed disc puts the
    // jaw center at the same motor angle every mount, so in STEP mode the step only picks the turn and the target
    // is snapped to this angle (PSM1 needle driver: true centers 3583 / 3575, steps 3672-3925). -1 = not used.
    double m5_center_angle_ = -1;
    // <adapter>.m2_center_angle: M2's true center as a motor angle. OPT-IN per adapter (PSM2's M2 jumped half a
    // turn between mounts): after the contact, M2 snaps to it if within jaw.m2_center_snap_max. -1 = not used.
    double m2_center_angle_ = -1;                      // <adapter>.pitch_limit1_angle (-1 = none for this adapter)                              // slow_jaws:=true (or slow): jaw moves at the sweep speed
    std::set<std::string> soft_limit_axes_;               // "roll"/"pitch"/"jaw": a limit came from the current test
    std::set<std::string> push_limit_axes_;               // ... a limit came from the PUSH test (motor still moving)
    std::map<std::string, std::string> cal_row_;           // this run's measurements (history columns)
    struct CalUpdate { std::string key, value, note; };
    std::vector<CalUpdate> cal_updates_;                    // proposed tool-file changes
    void cal_review();
    void cal_save();
    void cal_discard();
    // timing / roll range record (for speeding up roll)
    double t_homing_start_ = 0, t_roll_done_ = 0, t_pitch_done_ = 0;
    int32_t roll_lim1_ = 0, roll_lim2_ = 0;
    std::string jaw_log_name_;
    // ARTS handoff to disc_drive (see tool_homing_node.cpp, top)
    std::string adapter_;                    // the adapter's FTDI serial
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr home_pub_;
    std::string home_file() const;
    void write_home();

    double now_s() const;
    int jdir(int m) const { return m == 2 ? jp_.dir_m2 : jp_.dir_m5; }
    bool joy_fresh() const;
    void load_jaw_params();
    // Tool file config/<tool>.txt (forceps.txt, ...): every tool-specific value, REQUIRED (no silent defaults).
    bool load_tool_file(const std::string& path, std::string& err);
    template <typename T> T tool_value(const std::string& key, T def);
    std::string tool_name_, config_dir_;
    std::map<std::string, std::string> tool_cfg_;
    std::set<std::string> tool_used_;
    std::vector<std::string> tool_missing_, tool_bad_;
    bool config_error_ = false;
    void start_jaw_homing();
    void jaw_next();
    void jaw_tick();
    bool read_jaws();
    void jaw_command();
    void jaw_hold_here();
    void jaw_flag(const std::string& text);
    void jaw_log_row(const char* stage);
    void jaw_start_sweep(int m);
    void jaw_tick_sweep(double now);
    void jaw_stop_sweep(const std::string& why);
    void jaw_start_goto_m5_middle();
    void jaw_start_goto_m2_offset();
    void jaw_tick_goto(double now);
    void jaw_start_close(int m);
    void jaw_tick_close(double now);
    void jaw_stop_close(const std::string& why);
    void jaw_abort(const std::string& why);
    void jaw_finish();
    std::string hw_error_report();

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr current_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr event_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pos_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr limit_pub_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();

    SetupState current_state_;
    std::unique_ptr<arts_homing::DxlInterface> dxl_interface_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ToolHomingNode>());
//     rclcpp::shutdown();
//     return 0;
// }
