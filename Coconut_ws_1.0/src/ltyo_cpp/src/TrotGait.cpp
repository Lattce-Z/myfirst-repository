#include "TrotGait.hpp"
#include <cmath>
#include <algorithm>

TrotGait::TrotGait() {
    constexpr float L = 763.0f;
    constexpr float ang = 32.5f * M_PI / 180.0f;
    a_ = (L / 2.0f) * std::cos(ang);
    b_ = (L / 2.0f) * std::sin(ang);
    offset_ = {0.0f, 0.5f, 0.5f, 0.0f};
}

void TrotGait::setPeriod(float p)           { period_ = p; }
void TrotGait::setDutyCycle(float d)        { duty_cycle_ = d; }
void TrotGait::setStandHeight(float h)      { stand_height_ = h; }
void TrotGait::setSwingHeight(float h)      { swing_height_ = h; }
void TrotGait::setSwingBias(float b)        { swing_bias_ = std::clamp(b, 0.0f, 1.0f); }
void TrotGait::setLiftoffSharpness(float k) { liftoff_k_ = std::clamp(k, 0.1f, 1.0f); }

void TrotGait::reset() {
    t_ = 0.0f;
    first_update_ = true;
}

Eigen::Vector3f TrotGait::getHipOffset(int leg) const {
    switch (leg) {
        case 0: return { a_, 0, -b_ };
        case 1: return { a_, 0,  b_ };
        case 2: return {-a_, 0, -b_ };
        case 3: return {-a_, 0,  b_ };
        default: return {0, 0, 0};
    }
}

Eigen::Vector3f TrotGait::computeTouchdown(const Eigen::Vector3f& v_des,
                                           float omega_des, int leg) const {
    // 支撑相时长
    const float T_stance = period_ * (1.0f - duty_cycle_);
    const Eigen::Vector3f hip = getHipOffset(leg);
    const Eigen::Vector3f omega_vec(0, omega_des, 0);

    // 在支撑相期间，足端相对于身体需要向后移动的距离
    // 这样才能推动身体以 v_des 前进
    // 着地点 = 中立站位 + 半个支撑相的前置量
    const Eigen::Vector3f stance_delta = (v_des + omega_vec.cross(hip)) * (T_stance * 0.5f);

    Eigen::Vector3f touchdown;
    touchdown.x() = stance_delta.x();
    touchdown.y() = stand_height_ + stance_delta.y();
    touchdown.z() = stance_delta.z();

    return touchdown;
}

std::array<Eigen::Vector3f, 4> TrotGait::update(
        float dt,
        const Eigen::Vector3f& v_des,
        float omega_des,
        const std::array<Eigen::Vector3f, 4>& current_feet) {

    t_ += dt;
    while (t_ >= period_) t_ -= period_;

    std::array<Eigen::Vector3f, 4> targets;

    for (int leg = 0; leg < 4; ++leg) {
        float phase = t_ / period_ + offset_[leg];
        phase -= std::floor(phase);
        bool is_swing = (phase < duty_cycle_);

        // ========== 首次调用：初始化状态 ==========
        if (first_update_) {
            legs_[leg].prev_swing = is_swing;
            const Eigen::Vector3f& cur = current_feet[leg];

            if (is_swing) {
                legs_[leg].swing_start  = cur;
                legs_[leg].swing_end    = computeTouchdown(v_des, omega_des, leg);
                legs_[leg].stance_start = legs_[leg].swing_end;
                legs_[leg].stance_end   = computeTouchdown(v_des, omega_des, leg)
                                         - (v_des + Eigen::Vector3f(0,omega_des,0).cross(getHipOffset(leg)))
                                         * (period_ * (1.0f - duty_cycle_));
            } else {
                legs_[leg].stance_start = cur;
                legs_[leg].stance_end   = computeTouchdown(v_des, omega_des, leg)
                                         - (v_des + Eigen::Vector3f(0,omega_des,0).cross(getHipOffset(leg)))
                                         * (period_ * (1.0f - duty_cycle_));
                legs_[leg].swing_start  = cur;
                legs_[leg].swing_end    = computeTouchdown(v_des, omega_des, leg);
            }
        }
        // ========== 相位切换检测 ==========
        else {
            if (is_swing && !legs_[leg].prev_swing) {
                // 刚进入摆动相
                legs_[leg].swing_start = current_feet[leg];
                legs_[leg].swing_end   = computeTouchdown(v_des, omega_des, leg);
            }
            else if (!is_swing && legs_[leg].prev_swing) {
                // 刚进入支撑相
                legs_[leg].stance_start = legs_[leg].swing_end;
            }
            legs_[leg].prev_swing = is_swing;
        }

        const Eigen::Vector3f hip = getHipOffset(leg);
        const Eigen::Vector3f omega_vec(0, omega_des, 0);

        // ========== 支撑相：足端向后运动 ==========
        if (!is_swing) {
            float s = (phase - duty_cycle_) / (1.0f - duty_cycle_);

            // 支撑终点 = 着地点 - 整个支撑相的移动量（回到站立区域后部）
            const float T_stance = period_ * (1.0f - duty_cycle_);
            const Eigen::Vector3f total_shift = (v_des + omega_vec.cross(hip)) * T_stance;
            legs_[leg].stance_end = computeTouchdown(v_des, omega_des, leg) - total_shift;

            targets[leg] = (1.0f - s) * legs_[leg].stance_start
                         + s * legs_[leg].stance_end;
        }
        // ========== 摆动相：贝塞尔曲线 ==========
        else {
            float t = phase / duty_cycle_;
            const Eigen::Vector3f P0 = legs_[leg].swing_start;
            const Eigen::Vector3f P3 = legs_[leg].swing_end;

            Eigen::Vector3f delta = P3 - P0;

            // ---- 控制点 P1：快速抬起，前拉 ----
            Eigen::Vector3f P1 = P0;

            // y 方向：从当前位置快速抬到 swing_height 以上
            float lift_peak = stand_height_ + swing_height_;
            // 起点可能不在 stand_height，自适应
            float y_mid = std::max(lift_peak, P0.y() + swing_height_ * 0.6f);
            P1.y() = P0.y() + (y_mid - P0.y()) * liftoff_k_;

            // xz 方向：提前走完 delta 的一部分
            float pull_xz = swing_bias_ * 0.5f;
            P1.x() += delta.x() * pull_xz;
            P1.z() += delta.z() * pull_xz;

            // ---- 控制点 P2：高位保持，前凸越过终点 ----
            Eigen::Vector3f P2 = P3;

            float hold_y = stand_height_ + swing_height_ * 0.7f;
            P2.y() = std::max(hold_y, P3.y() + swing_height_ * 0.3f);

            float overshoot = swing_bias_ * 0.4f;
            P2.x() = P3.x() + delta.x() * overshoot;
            P2.z() = P3.z() + delta.z() * overshoot * 0.3f;

            // 三次贝塞尔
            float u = 1.0f - t;
            targets[leg] = u*u*u * P0
                         + 3*u*u*t * P1
                         + 3*u*t*t * P2
                         + t*t*t * P3;
        }
    }

    first_update_ = false;
    return targets;
}