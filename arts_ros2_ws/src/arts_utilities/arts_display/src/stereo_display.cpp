// Surgeon display: one component, two outputs, loaded into the camera container so frames
// arrive by pointer (docs/modules/console_display.md).
//
//   HMD    1920x1080 half-width side-by-side: each eye 810x1080 (a 1620x1080 image squeezed
//          to half width; the headset stretches it back), centred in its 960-wide half.
//   tower  1920x1080: left camera only, 1620x1080, centred.
//
// The display carries no data (decision 15). Rotation is display-only; the recording keeps
// the sensor orientation (decision 12). Never ReverseX/ReverseY in the camera.
//
// Threading: subscription callbacks only store the newest frame pointer, so the executor that
// the recorder shares is never held up by drawing. One render thread owns every highgui call.
//
// Status overlay (always on unless status_overlay:=false): the recording state, top left of
// the tower image and at the same place in each HMD eye (so it fuses at screen depth). From
// /console/episode_mark (START = recording, END = paused, published by the record pedal
// around the recorder's resume/pause): "NO RECORDER" (grey) when the camera process has no
// recorder, "READY tap to record episode 1" / "SAVED episode N, tap to record episode N+1"
// (yellow), "REC episode N mm:ss" (red). Episodes are closed, never resumed: the second pedal
// press ends one and the next press starts a new one. The operator must always be able to
// tell whether this moment is being recorded.
//
// Pedal panel (pedal_overlay, on by default), bottom centre between the two PSM labels, laid
// out as the pedals sit at the console (PI, 2026-11-06): the upper row is step layer 2, the
// platform ~10 cm up, with the monopolar pedal's CUT (yellow) and COAG (blue) and the bipolar
// pedal (blue), in energy_pedals order, left to right; the lower row is step layer 1, the
// floor, with the Savant's three pedals (pedal_labels, left to right). A pressed pedal is
// filled: the Savant's from /console/footswitch_teleop, the energy pedals from /esu/state (the
// ESU-TAP board's driven outputs, so a software activation lights them too). The energy row is
// grey while no /esu/state has arrived for 3 s (the esu node sends a 1 Hz snapshot).
// The board reports which contacts are closed; the generator decides what is delivered. The
// panel models the DRE ASG-300 as measured 2026-11-06 (10 W on all three outputs, no
// instrument, all six two-pedal orders): the FIRST pedal pressed is delivered (filled), a
// second one closed meanwhile is ignored (dark fill, coloured outline); releasing the first
// while the second is still held gives fault F6 and no output until every energy pedal is up
// (red, "F6"). Two closed in one board message from rest: not measured, shown pressed, none
// delivered.
//
// PSM labels: "psm_left  OPEN" (or MIS) on one line and the teleop status (ENGAGED, the
// alignment angle, NO STYLUS) on the line below, each centred at 1/4 (3/4) of the width and
// kept between the screen edge and the pedal panel, shrunk if it would not fit.

#include <cmath>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "arts_interfaces/msg/energy_state.hpp"
#include "arts_interfaces/msg/episode_mark.hpp"
#include "arts_interfaces/msg/rcm_state.hpp"
#include "arts_interfaces/msg/teleop_state.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

namespace arts_display
{

using Image = sensor_msgs::msg::Image;
using Clock = std::chrono::system_clock;

static double stamp_s(const Image & m)
{
  return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9;
}

static double now_s()
{
  return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

class StereoDisplay : public rclcpp::Node
{
public:
  explicit StereoDisplay(const rclcpp::NodeOptions & options)
  : Node("stereo_display", options)
  {
    const auto left_topic = declare_parameter<std::string>("left_topic", "/ecm/stereo_left/image_raw");
    const auto right_topic =
      declare_parameter<std::string>("right_topic", "/ecm/stereo_right/image_raw");
    rotate_180_ = declare_parameter<bool>("rotate_180", true);
    show_hmd_ = declare_parameter<bool>("hmd", true);
    show_tower_ = declare_parameter<bool>("tower", true);
    fullscreen_ = declare_parameter<bool>("fullscreen", false);
    hmd_x_ = declare_parameter<int>("hmd_x", 0);
    hmd_y_ = declare_parameter<int>("hmd_y", 0);
    tower_x_ = declare_parameter<int>("tower_x", 0);
    tower_y_ = declare_parameter<int>("tower_y", 0);
    // Window size when not fullscreen (the E7 bench shows a small tower view beside a clock).
    window_w_ = declare_parameter<int>("window_width", 1920);
    window_h_ = declare_parameter<int>("window_height", 1080);
    overlay_ = declare_parameter<bool>("overlay", false);
    status_overlay_ = declare_parameter<bool>("status_overlay", true);
    // Set by cameras.launch.py: is there a recorder in this process at all?
    rec_state_ = declare_parameter<bool>("recorder_present", false) ? kPaused : kNoRecorder;
    // Both cameras fire on one hardware pulse; header stamps of a pair differ by up to ~1 ms.
    pair_tolerance_s_ = declare_parameter<double>("pair_tolerance_ms", 5.0) * 1e-3;
    stats_period_s_ = declare_parameter<double>("stats_period_s", 5.0);

    const char * d = std::getenv("DISPLAY");
    const char * w = std::getenv("WAYLAND_DISPLAY");
    headless_ = !(d && *d) && !(w && *w);
    if (headless_) {
      RCLCPP_WARN(get_logger(), "no DISPLAY: composing frames and reporting timing only");
    }

    // Depth 1: the display only ever wants the newest frame; it must never queue.
    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    left_sub_ = create_subscription<Image>(
      left_topic, qos, [this](Image::ConstSharedPtr m) {store(m, left_);});
    right_sub_ = create_subscription<Image>(
      right_topic, qos, [this](Image::ConstSharedPtr m) {store(m, right_);});

    // Teleop alignment overlay (decision 17, 2026-11-03 amendment; collaborator A's two
    // triads): per PSM, while its teleop waits at the alignment gate, the instrument (thin)
    // and the stylus (thick, light) orientations in the screen frame at a fixed point (1/4 and
    // 3/4 of the width); a mode line (MIS / OPEN, the angle or ENGAGED) for each PSM.
    align_overlay_ = declare_parameter<bool>("align_overlay", true);
    for (int k = 0; k < 2; ++k) {
      const std::string arm = k == 0 ? "psm_left" : "psm_right";
      align_frames_sub_[k] = create_subscription<geometry_msgs::msg::PoseArray>(
        "/" + arm + "/teleop_alignment_frames", rclcpp::QoS(1),
        [this, k](geometry_msgs::msg::PoseArray::ConstSharedPtr m) {
          if (m->poses.size() < 2) {return;}
          std::lock_guard<std::mutex> lk(arm_mutex_);
          for (int j = 0; j < 2; ++j) {
            const auto & o = m->poses[j].orientation;
            arm_[k].rot[j] = quatToRotation(o.x, o.y, o.z, o.w);
          }
          arm_[k].frames_t = now_s();
        });
      align_angle_sub_[k] = create_subscription<std_msgs::msg::Float64>(
        "/" + arm + "/teleop_alignment", rclcpp::QoS(1),
        [this, k](std_msgs::msg::Float64::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lk(arm_mutex_);
          arm_[k].angle = m->data;
        });
      teleop_sub_[k] = create_subscription<arts_interfaces::msg::TeleopState>(
        "/" + arm + "/teleop_state", rclcpp::QoS(1).reliable().transient_local(),
        [this, k](arts_interfaces::msg::TeleopState::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lk(arm_mutex_);
          arm_[k].engaged = m->clutch_engaged;
        });
      rcm_sub_[k] = create_subscription<arts_interfaces::msg::RcmState>(
        "/" + arm + "/rcm_state", rclcpp::QoS(1),
        [this, k](arts_interfaces::msg::RcmState::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lk(arm_mutex_);
          arm_[k].mis = m->enabled;
          arm_[k].rcm_t = now_s();
        });
    }

    pedal_overlay_ = declare_parameter<bool>("pedal_overlay", true);
    pedal_labels_ = declare_parameter<std::vector<std::string>>(
      "pedal_labels", {"CLUTCH", "CAMERA", "REC"});
    energy_pedals_ = declare_parameter<std::vector<std::string>>(
      "energy_pedals", {"cut", "coag", "bipolar"});
    if (pedal_labels_.size() != 3 || energy_pedals_.size() != 3) {
      throw std::invalid_argument("pedal_labels and energy_pedals take three entries each");
    }
    for (const auto & e : energy_pedals_) {
      if (e != "cut" && e != "coag" && e != "bipolar") {
        throw std::invalid_argument("energy_pedals: cut | coag | bipolar, not " + e);
      }
    }
    switch_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/console/footswitch_teleop", rclcpp::QoS(10),
      [this](sensor_msgs::msg::Joy::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(arm_mutex_);
        for (size_t i = 0; i < 3; ++i) {switch_[i] = i < m->buttons.size() && m->buttons[i];}
      });
    esu_sub_ = create_subscription<arts_interfaces::msg::EnergyState>(
      "/esu/state", rclcpp::QoS(10),
      [this](arts_interfaces::msg::EnergyState::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(arm_mutex_);
        energy_ = {m->monopolar_cut, m->monopolar_coag, m->bipolar};
        energy_t_ = now_s();
        const int n = energy_[0] + energy_[1] + energy_[2];
        if (n == 0) {
          energy_active_ = -1;
          energy_fault_ = false;
        } else if (energy_fault_) {
        } else if (energy_active_ >= 0) {
          if (!energy_[energy_active_]) {  // the delivered one released, another still held
            energy_active_ = -1;
            energy_fault_ = true;
          }
        } else if (n == 1) {
          energy_active_ = energy_[0] ? 0 : (energy_[1] ? 1 : 2);
        }
      });

    mark_sub_ = create_subscription<arts_interfaces::msg::EpisodeMark>(
      "/console/episode_mark", rclcpp::QoS(10).reliable(),
      [this](arts_interfaces::msg::EpisodeMark::ConstSharedPtr m) {
        if (m->mark_type == arts_interfaces::msg::EpisodeMark::START) {
          episode_ = m->episode_id;
          rec_since_ = now_s();
          rec_state_ = kRecording;
        } else if (m->mark_type == arts_interfaces::msg::EpisodeMark::END) {
          rec_state_ = kPaused;
        }
      });

    render_thread_ = std::thread([this] {render_loop();});
  }

  ~StereoDisplay() override
  {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      stop_ = true;
    }
    cv_.notify_all();
    if (render_thread_.joinable()) {
      render_thread_.join();
    }
  }

private:
  enum RecState {kNoRecorder = 0, kPaused = 1, kRecording = 2};

  // Text with a dark backing box, so it reads on any tissue colour.
  static void label(cv::Mat & img, const std::string & text, cv::Point at, double scale,
    const cv::Scalar & colour)
  {
    int base = 0;
    const int thick = std::max(1, static_cast<int>(std::lround(2 * scale)));
    const cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thick, &base);
    cv::rectangle(img, at + cv::Point(-8, -sz.height - 8), at + cv::Point(sz.width + 8, base + 6),
      cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(img, text, at, cv::FONT_HERSHEY_SIMPLEX, scale, colour, thick, cv::LINE_AA);
  }

  // label() centred on cx and kept inside [x0, x1] (its backing box included), its scale
  // shrunk until it fits.
  static void label_fit(cv::Mat & img, const std::string & text, int cx, int y, double scale,
    const cv::Scalar & colour, int x0, int x1)
  {
    int base = 0;
    cv::Size sz;
    for (;; scale *= 0.9) {
      const int thick = std::max(1, static_cast<int>(std::lround(2 * scale)));
      sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thick, &base);
      if (sz.width + 16 <= x1 - x0 || scale < 0.45) {break;}
    }
    const int left = std::max(x0 + 8, std::min(cx - sz.width / 2, x1 - 8 - sz.width));
    label(img, text, cv::Point(left, y), scale, colour);
  }

  // A 3x3 rotation (row-major) from a quaternion; the overlay needs nothing more.
  static std::array<double, 9> quatToRotation(double x, double y, double z, double w)
  {
    return {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
  }

  // One triad: screen x right / y up (image y down), z toward the viewer drawn as is
  // (orthographic, same in both eyes: read at screen depth). Colours x red, y green, z blue.
  static void draw_triad(
    cv::Mat & img, cv::Point c, const std::array<double, 9> & r, double len, int thick,
    double light)
  {
    const cv::Scalar base[3] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 200, 0),
      cv::Scalar(255, 80, 0)};
    for (int a = 0; a < 3; ++a) {
      const cv::Point end = c + cv::Point(
        static_cast<int>(std::lround(len * r[0 * 3 + a])),
        static_cast<int>(std::lround(-len * r[1 * 3 + a])));
      const cv::Scalar col = base[a] * (1.0 - light) + cv::Scalar(255, 255, 255) * light;
      cv::line(img, c, end, col, thick, cv::LINE_AA);
    }
  }

  void draw_alignment(cv::Mat & img)
  {
    std::lock_guard<std::mutex> lk(arm_mutex_);
    const double t = now_s();
    for (int k = 0; k < 2; ++k) {
      const ArmOverlay & a = arm_[k];
      if (t - a.rcm_t > 1.0) {continue;}  // that arm is not running
      const char * name = k == 0 ? "psm_left" : "psm_right";
      const int cx = img.cols * (k == 0 ? 1 : 3) / 4;
      const int mid = img.cols / 2, keep = pedal_overlay_ ? kPanelHalf + 16 : 16;
      const int x0 = k == 0 ? 0 : mid + keep, x1 = k == 0 ? mid - keep : img.cols;
      const bool waiting = !a.engaged && t - a.frames_t < 0.5;
      const bool no_stylus = std::isnan(a.angle);
      const cv::Scalar col = a.engaged ? cv::Scalar(80, 220, 80) : cv::Scalar(0, 220, 255);
      label_fit(img, std::string(name) + (a.mis ? "  MIS" : "  OPEN"), cx, img.rows - 100, 0.9,
        col, x0, x1);
      const std::string status = a.engaged ? std::string("ENGAGED") :
        (waiting ? (no_stylus ? std::string("NO STYLUS (VerseGrip off?)") :
        cv::format("align %.1f deg", a.angle)) : std::string());
      if (!status.empty()) {label_fit(img, status, cx, img.rows - 48, 0.9, col, x0, x1);}
      if (waiting) {
        const double len = 0.2 * std::min(img.cols, img.rows);
        const cv::Point c(cx, img.rows / 2);
        if (!no_stylus) {draw_triad(img, c, a.rot[1], len, 9, 0.55);}  // the stylus: thick, light
        draw_triad(img, c, a.rot[0], len, 3, 0.0);   // the instrument: thin
      }
    }
  }

  // The pedal panel (see the header): two rows of three pads, bottom centre.
  void draw_pedals(cv::Mat & img)
  {
    std::array<bool, 3> sw, en;
    bool esu, fault;
    int active;
    {
      std::lock_guard<std::mutex> lk(arm_mutex_);
      sw = switch_;
      en = energy_;
      esu = now_s() - energy_t_ < 3.0;
      fault = energy_fault_;
      active = energy_active_;
    }
    const int w = 2 * (kPanelHalf - 16 - kPadGap) / 3, h = 40;
    const int cx = img.cols / 2, left = cx - kPanelHalf + 16;
    const int y_low = img.rows - 20 - h, y_up = y_low - h - 34;
    cv::rectangle(img, cv::Point(cx - kPanelHalf, y_up - 34), cv::Point(cx + kPanelHalf, img.rows - 8),
      cv::Scalar(0, 0, 0), cv::FILLED);
    const cv::Scalar yellow(0, 220, 255), blue(255, 140, 0), grey(130, 130, 130);
    // on: filled (delivered / pressed); held: pressed but not delivered (dark fill)
    auto pad = [&](int i, int y, const std::string & text, const cv::Scalar & col, bool on,
        bool held = false, const cv::Scalar & edge = cv::Scalar(-1, -1, -1)) {
        const cv::Point a(left + i * (w + kPadGap), y), b = a + cv::Point(w, h);
        if (on) {cv::rectangle(img, a, b, col, cv::FILLED);}
        if (held) {cv::rectangle(img, a, b, cv::Scalar(70, 70, 70), cv::FILLED);}
        cv::rectangle(img, a, b, edge[0] < 0 ? col : edge, held ? 4 : 2, cv::LINE_AA);
        int base = 0;
        const cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &base);
        cv::putText(img, text, a + cv::Point((w - sz.width) / 2, (h + sz.height) / 2),
          cv::FONT_HERSHEY_SIMPLEX, 0.55, on ? cv::Scalar(0, 0, 0) : col, on ? 2 : 1, cv::LINE_AA);
      };
    for (int i = 0; i < 3; ++i) {
      const std::string & e = energy_pedals_[i];
      const int k = e == "cut" ? 0 : (e == "coag" ? 1 : 2);
      const char * text = k == 0 ? "CUT" : (k == 1 ? "COAG" : "BIPOLAR");
      const cv::Scalar red(40, 40, 255);
      pad(i, y_up, text, esu ? (k == 0 ? yellow : blue) : grey, esu && active == k,
        esu && en[k] && active != k, fault && en[k] ? red : cv::Scalar(-1, -1, -1));
    }
    if (!esu) {
      cv::putText(img, "ESU off", cv::Point(left, y_up - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5,
        grey, 1, cv::LINE_AA);
    } else if (fault) {
      cv::putText(img, "F6: release all energy pedals", cv::Point(left, y_up - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(40, 40, 255), 1, cv::LINE_AA);
    }
    const cv::Scalar sw_col[3] = {cv::Scalar(230, 230, 230), cv::Scalar(80, 220, 80),
      cv::Scalar(40, 40, 255)};
    for (int i = 0; i < 3; ++i) {pad(i, y_low, pedal_labels_[i], sw_col[i], sw[i]);}
  }

  void draw_status(cv::Mat & img, cv::Point at, double scale)
  {
    switch (rec_state_.load()) {
      case kRecording: {
          const int s = static_cast<int>(now_s() - rec_since_.load());
          label(img, cv::format("REC  episode %u  %02d:%02d", episode_.load(), s / 60, s % 60),
            at, scale, cv::Scalar(40, 40, 255));
          cv::circle(img, at + cv::Point(-30, static_cast<int>(-12 * scale)),
            static_cast<int>(10 * scale), cv::Scalar(40, 40, 255), cv::FILLED);
          break;
        }
      case kPaused:
        if (episode_.load() == 0) {
          label(img, "READY  tap pedal to record episode 1", at, scale, cv::Scalar(0, 220, 255));
        } else {
          label(img, cv::format("SAVED episode %u   tap pedal to record episode %u",
            episode_.load(), episode_.load() + 1), at, scale, cv::Scalar(0, 220, 255));
        }
        break;
      default:
        label(img, "NO RECORDER", at, scale, cv::Scalar(170, 170, 170));
    }
  }

  void store(const Image::ConstSharedPtr & m, std::deque<Image::ConstSharedPtr> & q)
  {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      q.push_back(m);
      if (q.size() > 4) {
        q.pop_front();
      }
    }
    cv_.notify_one();
  }

  // Newest left frame that has a right frame of the same pulse, and is newer than the last
  // pair shown. Called with mutex_ held.
  bool take_pair(Image::ConstSharedPtr & l, Image::ConstSharedPtr & r)
  {
    for (auto il = left_.rbegin(); il != left_.rend(); ++il) {
      const double sl = stamp_s(**il);
      if (sl <= last_shown_) {
        return false;
      }
      for (const auto & ir : right_) {
        if (std::abs(stamp_s(*ir) - sl) < pair_tolerance_s_) {
          l = *il;
          r = ir;
          return true;
        }
      }
    }
    return false;
  }

  // Bayer (or colour) frame -> BGR, rotated for viewing, scaled to the 1080-high 3:2 image.
  bool to_view(const Image & m, cv::Mat & out)
  {
    const cv::Mat raw(m.height, m.width, CV_8UC(m.step / m.width),
      const_cast<uint8_t *>(m.data.data()), m.step);
    cv::Mat bgr;
    if (m.encoding == "bayer_rggb8") {
      cv::cvtColor(raw, bgr, cv::COLOR_BayerBG2BGR);  // ROS rggb == OpenCV "BG" naming
    } else if (m.encoding == "bgr8") {
      bgr = raw;
    } else if (m.encoding == "rgb8") {
      cv::cvtColor(raw, bgr, cv::COLOR_RGB2BGR);
    } else {
      RCLCPP_ERROR_ONCE(get_logger(), "unsupported encoding '%s'", m.encoding.c_str());
      return false;
    }
    if (rotate_180_) {
      cv::rotate(bgr, bgr, cv::ROTATE_180);
    }
    cv::resize(bgr, out, cv::Size(1620, 1080), 0, 0, cv::INTER_LINEAR);
    return true;
  }

  void open_window(const std::string & name, int x, int y)
  {
    // GUI_NORMAL: no Qt toolbar or status bar (they took a third of a small window).
    cv::namedWindow(name, cv::WINDOW_NORMAL | cv::WINDOW_GUI_NORMAL);
    cv::resizeWindow(name, window_w_, window_h_);
    place_window(name, x, y);
  }

  void place_window(const std::string & name, int x, int y)
  {
    // Moving a fullscreen window leaves it framed and maximized (GNOME, 2026-10-28), so leave
    // fullscreen first.
    if (fullscreen_) {
      cv::setWindowProperty(name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
    }
    cv::moveWindow(name, x, y);
    if (fullscreen_) {
      cv::setWindowProperty(name, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    } else {
      cv::resizeWindow(name, window_w_, window_h_);
    }
  }

  void render_loop()
  {
    const std::string hmd_win = "ARTS HMD", tower_win = "ARTS tower";
    if (!headless_) {
      if (show_hmd_) {open_window(hmd_win, hmd_x_, hmd_y_);}
      if (show_tower_) {open_window(tower_win, tower_x_, tower_y_);}
    }
    cv::Mat hmd(1080, 1920, CV_8UC3, cv::Scalar::all(0));
    cv::Mat tower(1080, 1920, CV_8UC3, cv::Scalar::all(0));
    cv::Mat lv, rv, squeezed;

    double t_stats = now_s(), age_sum = 0, age_max = 0, proc_sum = 0, proc_max = 0;
    int shown = 0, skipped_stats = 0;

    while (rclcpp::ok()) {
      Image::ConstSharedPtr l, r;
      {
        std::unique_lock<std::mutex> lk(mutex_);
        cv_.wait_for(lk, std::chrono::milliseconds(20), [&] {return stop_ || take_pair(l, r);});
        if (stop_) {
          break;
        }
        if (l) {
          const double prev = last_shown_;
          last_shown_ = stamp_s(*l);
          // Pulses between the previous pair shown and this one were never drawn.
          if (prev > 0) {
            skipped_stats += std::max(0, static_cast<int>(std::lround((last_shown_ - prev) * 30.0)) - 1);
          }
        }
      }
      if (!l) {
        if (!headless_) {cv::waitKey(1);}
        continue;
      }

      const double t0 = now_s();
      if (!to_view(*l, lv) || !to_view(*r, rv)) {
        continue;
      }
      if (status_overlay_) {
        // On the full-size views, before the HMD squeeze (the headset stretches each eye back
        // to full width, so text drawn after it would look double-wide), at the same spot in
        // both eyes: zero disparity, read at screen depth. The tower shows lv.
        draw_status(lv, cv::Point(70, 70), 1.3);
        draw_status(rv, cv::Point(70, 70), 1.3);
      }
      if (align_overlay_) {  // same rule: full-size views, both eyes, zero disparity
        draw_alignment(lv);
        draw_alignment(rv);
      }
      if (pedal_overlay_) {
        draw_pedals(lv);
        draw_pedals(rv);
      }
      if (show_hmd_) {
        cv::resize(lv, squeezed, cv::Size(810, 1080), 0, 0, cv::INTER_AREA);
        squeezed.copyTo(hmd(cv::Rect(75, 0, 810, 1080)));
        cv::resize(rv, squeezed, cv::Size(810, 1080), 0, 0, cv::INTER_AREA);
        squeezed.copyTo(hmd(cv::Rect(960 + 75, 0, 810, 1080)));
      }
      if (show_tower_) {
        lv.copyTo(tower(cv::Rect(150, 0, 1620, 1080)));
      }
      if (overlay_ && show_tower_) {
        // The previous pair's age: this pair's is known only after it is handed off.
        cv::putText(tower, cv::format("frame age %.0f ms", last_age_ms_), cv::Point(170, 1060),
          cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
      }
      if (!headless_) {
        if (show_hmd_) {cv::imshow(hmd_win, hmd);}
        if (show_tower_) {cv::imshow(tower_win, tower);}
        cv::waitKey(1);
        // GNOME ignored the placement of a window created before anything was drawn in it
        // (2026-10-28: the HMD window stayed small on the primary screen). Place both again
        // once they have content.
        if (!placed_) {
          if (show_hmd_) {place_window(hmd_win, hmd_x_, hmd_y_);}
          if (show_tower_) {place_window(tower_win, tower_x_, tower_y_);}
          cv::waitKey(1);
          placed_ = true;
        }
      }
      const double t1 = now_s();
      const double proc_ms = (t1 - t0) * 1e3;
      const double age_ms = (t1 - stamp_s(*l)) * 1e3;
      last_age_ms_ = age_ms;

      ++shown;
      age_sum += age_ms;
      age_max = std::max(age_max, age_ms);
      proc_sum += proc_ms;
      proc_max = std::max(proc_max, proc_ms);
      const double t = now_s();
      if (t - t_stats >= stats_period_s_) {
        // Frame age = wall clock at hand-off to the window system minus the frame's header
        // stamp. It excludes the window system and the panel, so it is a lower bound on
        // glass-to-glass latency, which is measured externally (E7).
        RCLCPP_INFO(get_logger(),
          "shown %.2f pairs/s, skipped %d, frame age mean %.1f max %.1f ms, "
          "compose+show mean %.1f max %.1f ms",
          shown / (t - t_stats), skipped_stats, age_sum / shown, age_max,
          proc_sum / shown, proc_max);
        t_stats = t;
        shown = skipped_stats = 0;
        age_sum = age_max = proc_sum = proc_max = 0;
      }
    }
    if (!headless_) {
      cv::destroyAllWindows();
    }
  }

  rclcpp::Subscription<arts_interfaces::msg::EpisodeMark>::SharedPtr mark_sub_;
  struct ArmOverlay
  {
    std::array<double, 9> rot[2]{};  // instrument, stylus (screen frame)
    double frames_t{-1e9}, rcm_t{-1e9}, angle{0.0};
    bool engaged{false}, mis{false};
  };
  ArmOverlay arm_[2];
  std::mutex arm_mutex_;
  bool align_overlay_{true};
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr align_frames_sub_[2];
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr align_angle_sub_[2];
  rclcpp::Subscription<arts_interfaces::msg::TeleopState>::SharedPtr teleop_sub_[2];
  rclcpp::Subscription<arts_interfaces::msg::RcmState>::SharedPtr rcm_sub_[2];
  bool status_overlay_{true};
  static constexpr int kPanelHalf = 150, kPadGap = 8;
  bool pedal_overlay_{true};
  std::vector<std::string> pedal_labels_, energy_pedals_;
  std::array<bool, 3> switch_{}, energy_{};   // Savant left/middle/right; cut, coag, bipolar
  double energy_t_{-1e9};
  int energy_active_{-1};      // the output the generator delivers (first pressed), -1 none
  bool energy_fault_{false};   // F6: the delivered pedal released while another was held
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr switch_sub_;
  rclcpp::Subscription<arts_interfaces::msg::EnergyState>::SharedPtr esu_sub_;
  std::atomic<int> rec_state_{kNoRecorder};
  std::atomic<uint32_t> episode_{0};
  std::atomic<double> rec_since_{0.0};
  rclcpp::Subscription<Image>::SharedPtr left_sub_, right_sub_;
  std::deque<Image::ConstSharedPtr> left_, right_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::thread render_thread_;
  bool stop_ = false, headless_ = false, placed_ = false;
  bool rotate_180_, show_hmd_, show_tower_, fullscreen_, overlay_;
  int hmd_x_, hmd_y_, tower_x_, tower_y_, window_w_, window_h_;
  double pair_tolerance_s_, stats_period_s_, last_shown_ = 0, last_age_ms_ = 0;
};

}  // namespace arts_display

RCLCPP_COMPONENTS_REGISTER_NODE(arts_display::StereoDisplay)
