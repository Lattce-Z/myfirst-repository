#pragma once
#include <Eigen/Dense>
#include <array>

/**
 * @brief Trot 步态生成器（不对称贝塞尔摆动轨迹）
 *
 * 坐标系（与腿部执行器一致）：
 *   原点：髋关节
 *   x 正向：前
 *   y 正向：上
 *   z 正向：右
 *
 * 输入：机体系期望速度 v_des [vx, vy≈0, vz] 和绕 y 轴角速度 ω (rad/s)
 *       四条腿当前实时足端坐标（髋关节坐标系）
 * 输出：四条腿目标足端坐标 {LF, RF, LH, RH}
 */
class TrotGait {
public:
    TrotGait();

    // --- 参数设置 ---
    void setPeriod(float period);         // 步态周期 (s)，默认 0.5
    void setDutyCycle(float duty);        // 摆动相占比 (0~1)，默认 0.5
    void setStandHeight(float height);    // 站立高度 y (负值)，默认 -260
    void setSwingHeight(float height);    // 摆动最大抬脚高度 (mm)，默认 60
    void setSwingBias(float bias);        // 贝塞尔向前凸出系数 0~1，默认 0.7
    void setLiftoffSharpness(float k);

    void reset();

    /**
     * @brief 更新步态，返回四条腿目标位置
     * @param dt            时间步长 (s)
     * @param v_des         期望机体系速度 [vx, vy, vz]
     * @param omega_des     期望绕 y 轴角速度 (rad/s)
     * @param current_feet  四条腿当前足端坐标 {LF, RF, LH, RH}（髋关节坐标系）
     * @return 四条腿目标位置
     */
    std::array<Eigen::Vector3f, 4> update(float dt,
                                          const Eigen::Vector3f& v_des,
                                          float omega_des,
                                          const std::array<Eigen::Vector3f, 4>& current_feet);

private:
    Eigen::Vector3f getHipOffset(int leg) const;
    Eigen::Vector3f computeTouchdown(const Eigen::Vector3f& v_des,
                                     float omega_des, int leg) const;

    // 可调参数
    float period_ = 0.5f;
    float duty_cycle_ = 0.5f;
    float stand_height_ = -260.0f;
    float swing_height_ = 60.0f;
    float swing_bias_ = 0.7f;
    float liftoff_k_ = 0.0f;

    // 机身尺寸：半长 a，半宽 b
    float a_ = 321.7f;
    float b_ = 205.0f;

    // 相位偏移
    std::array<float, 4> offset_;

    // 内部状态
    float t_ = 0.0f;
    bool first_update_ = true;

    struct LegState {
        bool prev_swing = false;
        Eigen::Vector3f swing_start  = {0, -260, 0};
        Eigen::Vector3f swing_end    = {0, -260, 0};
        Eigen::Vector3f stance_start = {0, -260, 0};
        Eigen::Vector3f stance_end   = {0, -260, 0};
    };
    std::array<LegState, 4> legs_;
};