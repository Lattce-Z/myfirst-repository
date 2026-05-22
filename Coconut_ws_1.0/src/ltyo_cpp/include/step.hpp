#ifndef __STEP_HPP__
#define __STEP_HPP__

#include <Eigen/Geometry>

namespace step{

class Step{
public:
    Step(int leg_id);
    void Leg_Calculate(Eigen::Vector3f target);
    Eigen::Vector3f Leg_Calculate_Inverse(float Alpha, float Beta, float Phi);
    void Data_Output(float *Alpha, float *Beta, float *Phi);

    Eigen::Vector3f Force_sensor(float Alpha, float Beta, float Phi, float T_alpha, float T_Beta, float T_Phi);
private:
    int leg_id_; // 0:左前 1:右前 2:左后 3:右后
    float Alpha, Beta, Phi;
    static constexpr double hip = 95;
    static constexpr double thigh = 220;
    static constexpr double shin = 220;
    static constexpr float ori_Alpha = 70.88f * M_PI / 180;
    static constexpr float ori_Beta = 23.85 * M_PI / 180;
    static constexpr float ori_Phi = -10 * M_PI / 180;
    static constexpr double Beta_RGR = -1.5;    // Beta电机减速比
};

}

#endif
