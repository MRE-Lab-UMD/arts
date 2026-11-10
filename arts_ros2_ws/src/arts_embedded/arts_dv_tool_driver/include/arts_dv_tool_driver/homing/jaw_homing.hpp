// Jaw homing helpers (current-only, no camera). C++ port of jaw_stepfinder.py and the
// ContactDetector in jaw_sweep_collector.py, validated on forceps_01 (Nov 2026).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace jaw_homing
{

// Finds the load step in an opening sweep from |current| vs. position. No absolute current levels:
//   step  : largest rise of smoothed |current| over DX ticks (after MIN_RUN ticks of travel) that is
//           significant vs. this sweep's own earlier rises (z-score) and PERSISTS for HOLD ticks.
//   slip  : an earlier rise of similar size that did NOT persist (partial catch that let go).
//   mode_step : long free-spin run-up + clear rise + no slip -> trust the step.
//   early_trend : current at ENGAGED_AT ticks of travel / current at 150-300 ticks (engaged starts climb).
//   mid_trend   : current at 300-450 ticks / current at 150-300 ticks. An engaged start is ALREADY climbing
//                 here (1.05-1.11 on 9 labeled forceps_01 mounts); a step that just begins inside the
//                 early_trend window is flat here (0.99-1.00, psm1 runs 4 Nov).
struct StepResult
{
    bool valid = false;        // enough samples to analyze
    bool found = false;
    double step_pos = 0, rise = 0, runup = 0, ratio = 0, step_end_travel = 0, travel = 0;
    bool slip = false;
    bool mode_step = false;
    bool has_early = false;
    double early_trend = 0;
    double mid_trend = 0;
};

class StepFinder
{
public:
    double dx = 150, min_run = 150, hold = 450, min_runup = 250, min_ratio = 1.15, z = 4.0, engaged_at = 600;
    int w = 15;

    void reset() { pos_.clear(); cur_.clear(); }
    void add(int32_t pos, int16_t cur) { pos_.push_back(pos); cur_.push_back(std::abs(static_cast<int>(cur))); }
    size_t size() const { return pos_.size(); }

    // hold_override > 0: use a shorter "stays up" window (used when the sweep stopped because the
    // current hit its limit -- then the current demonstrably kept rising until the stop).
    StepResult analyze(double hold_override = -1) const
    {
        const double hold = hold_override > 0 ? hold_override : this->hold;
        StepResult R;
        const int n = static_cast<int>(pos_.size());
        if (n < 40) return R;
        R.valid = true;
        const double start = pos_[0];
        const int sgn = (pos_.back() >= pos_[0]) ? 1 : -1;
        std::vector<double> s(n), sc(n), cs(n + 1, 0.0);
        double run_max = -1e18;
        for (int i = 0; i < n; ++i) {
            run_max = std::max(run_max, sgn * (pos_[i] - start));
            s[i] = run_max;                                   // travel along the sweep
            cs[i + 1] = cs[i] + cur_[i];
        }
        const int h = w / 2;
        for (int i = 0; i < n; ++i) {
            int lo = std::max(0, i - h), hi = std::min(n, i + h + 1);
            sc[i] = (cs[hi] - cs[lo]) / (hi - lo);
        }
        auto lower = [&](double v) {
            return static_cast<int>(std::lower_bound(s.begin(), s.end(), v) - s.begin());
        };
        const int lag = static_cast<int>(dx / 6);
        struct Cand { double rise; int i, j, k; };
        bool have_best = false, have_rej = false;
        Cand best{0, 0, 0, 0}, rej{0, 0, 0, 0};
        std::vector<double> rises;
        double r_sum = 0, r_sq = 0;
        int r_cnt = 0;
        for (int i = 0; i < n; ++i) {
            if (s[i] < min_run) continue;
            int j = lower(s[i] + dx), k = lower(s[i] + dx + hold);
            if (k >= n) break;
            double rise = sc[j] - sc[i];
            while (r_cnt < static_cast<int>(rises.size()) - lag) {
                double r = rises[r_cnt]; r_sum += r; r_sq += r * r; ++r_cnt;
            }
            rises.push_back(rise);
            if (static_cast<int>(rises.size()) - 1 <= lag + 20 || r_cnt < 20) continue;
            double mean = r_sum / r_cnt;
            double noise = std::max(std::sqrt(std::max(r_sq / r_cnt - mean * mean, 0.0)), 1e-3);
            if (rise < mean + z * noise) continue;
            double mn = sc[j];
            for (int q = j; q < k; ++q) mn = std::min(mn, sc[q]);
            bool persists = mn >= sc[j] - 0.5 * rise;           // a slip falls back
            Cand c{rise, i, j, k};
            if (persists) { if (!have_best || rise > best.rise) { best = c; have_best = true; } }
            else if (!have_rej || rise > rej.rise) { rej = c; have_rej = true; }
        }
        R.travel = s.back();
        if (s.back() >= engaged_at) {
            double a = 0, b = 0, c = 0; int na = 0, nb = 0, nc = 0;
            for (int i = 0; i < n; ++i) {
                if (s[i] >= 150 && s[i] <= 300) { a += sc[i]; ++na; }
                if (s[i] >= engaged_at - 150 && s[i] <= engaged_at) { b += sc[i]; ++nb; }
                if (s[i] >= 300 && s[i] <= 450) { c += sc[i]; ++nc; }
            }
            if (na && nb) { R.has_early = true; R.early_trend = (b / nb) / std::max(a / na, 1e-6); }
            if (na && nc) R.mid_trend = (c / nc) / std::max(a / na, 1e-6);
        }
        if (!have_best) return R;
        R.found = true;
        const int i = best.i, j = best.j, k = best.k;
        R.slip = have_rej && rej.i < i && rej.rise >= 0.6 * best.rise;
        double pre = 0; int npre = 0;
        for (int q = 0; q < n; ++q) if (s[q] >= 80 && s[q] <= s[i]) { pre += sc[q]; ++npre; }
        pre = npre ? pre / npre : sc[i];
        double post = 0;
        for (int q = j; q < k; ++q) post += sc[q];
        post /= std::max(1, k - j);
        R.rise = best.rise;
        R.runup = s[i];
        R.ratio = post / std::max(pre, 1e-6);
        R.step_pos = start + sgn * (s[i] + s[j]) / 2.0;
        R.step_end_travel = s[j];
        R.mode_step = (R.runup >= min_runup && R.ratio >= min_ratio && !R.slip);
        return R;
    }

private:
    std::vector<double> pos_, cur_;
};

// Contact (or opening resistance): the motor is TORQUE-LIMITED (|current| >= sat x the goal current we
// commanded) AND SLOWING (speed < ratio x its own cruising speed learned during this move), for
// `need` samples in a row. No tool-specific threshold.
class ContactDetector
{
public:
    ContactDetector(double goal_current = 75, double ratio = 0.8, double sat = 0.9, int need = 3)
        : goal_(goal_current), ratio_(ratio), sat_(sat), need_(need) {}

    bool update(int32_t vel, int16_t cur, double age_s)
    {
        vh_.push_back(std::abs(static_cast<double>(vel)));
        ih_.push_back(std::abs(static_cast<double>(cur)));
        if (vh_.size() >= 10) {
            std::vector<double> last(vh_.end() - 10, vh_.end());
            std::sort(last.begin(), last.end());
            cruise_ = std::max(cruise_, (last[4] + last[5]) / 2.0);
        }
        if (age_s < 0.4 || cruise_ < 5) return false;
        double v3 = (vh_[vh_.size() - 1] + vh_[vh_.size() - 2] + vh_[vh_.size() - 3]) / 3.0;
        double i3 = (ih_[ih_.size() - 1] + ih_[ih_.size() - 2] + ih_[ih_.size() - 3]) / 3.0;
        bool hit = (v3 < ratio_ * cruise_) && (i3 >= sat_ * goal_);
        n_ = hit ? n_ + 1 : 0;
        return n_ >= need_;
    }
    double cruise() const { return cruise_; }

private:
    double goal_, ratio_, sat_;
    int need_;
    int n_ = 0;
    double cruise_ = 0;
    std::vector<double> vh_, ih_;
};

}  // namespace jaw_homing
