#include "step.hpp"

using namespace Eigen;

namespace step{

Step::Step(int leg_id)
    : leg_id_(leg_id){};

void Step::Leg_Calculate(Eigen::Vector3f targer)
{
    float x = targer[0], y = targer[1], z = targer[2];
    switch (this->leg_id_){
        case 0:
            z -= this->hip;
            break;
        case 1:
            z += this->hip;
            break;
        case 2:
            z -= this->hip;
            break;
        case 3:
            z += this->hip;
            break;
    }
    float y_1 = sqrt(y*y + z*z - this->hip * this->hip);
    float L = sqrt(x*x + y*y + z*z - this->hip * this->hip);
    this->Beta = acos((shin*shin + thigh*thigh - L*L) / (2*shin*thigh));
    this->Alpha = acos((L*L + shin*shin - thigh*thigh) / (2*L*shin))
                - atan(x / y_1);
    this->Phi = - atan(targer[2] / y);
}


Eigen::Vector3f Step::Leg_Calculate_Inverse(float Alpha, float Beta, float Phi)
{
    Eigen::Vector3f Thigh = {0, -this->thigh, 0}, Shin = {0, this->shin, 0}, Hip = {0, 0, this->hip};
    // 电机角度转换成关节角度
    switch(this->leg_id_){
        case 0:
            Alpha = Alpha + ori_Alpha;
            Beta = Beta / Beta_RGR + ori_Beta;
            Phi = -Phi + ori_Phi;
            Hip[2] = -Hip[2];
            break;
        case 1:
            Alpha = -Alpha + ori_Alpha;
            Beta = -Beta / Beta_RGR + ori_Beta;
            Phi = -Phi - ori_Phi;
            break;
        case 2:
            Alpha = Alpha + ori_Alpha;
            Beta = Beta / Beta_RGR + ori_Beta;
            Phi = Phi + ori_Phi;
            Hip[2] = -Hip[2];
            break;
        case 3:
            Alpha = -Alpha + ori_Alpha;
            Beta = -Beta / Beta_RGR + ori_Beta;
            Phi = Phi - ori_Phi;
            break;
    }
    // 逆运动学计算
    Eigen::Vector3f T = Eigen::AngleAxisf(-Phi, Eigen::Vector3f::UnitX()).toRotationMatrix() *
                   (Eigen::AngleAxisf(-Alpha, Eigen::Vector3f::UnitZ()).toRotationMatrix() * Thigh +
                    Eigen::AngleAxisf(-Alpha-Beta, Eigen::Vector3f::UnitZ()).toRotationMatrix() * Shin + Hip);
    
    switch(this->leg_id_){
        case 0:
            T[2] += hip;
            break;
        case 1:
            T[2] -= hip;
            break;
        case 2:
            T[2] += hip;
            break;
        case 3:
            T[2] -= hip;
            break;
    }
    return T;
}

void Step::Data_Output(float *Alpha, float *Beta, float *Phi)
{
    switch(this->leg_id_){
        case 0:
            *Alpha = (this->Alpha - ori_Alpha);
            *Beta = (this->Beta - ori_Beta) * Beta_RGR;
            *Phi = -(this->Phi - ori_Phi);
            break;
        case 1:
            *Alpha = -(this->Alpha - ori_Alpha);
            *Beta = -(this->Beta - ori_Beta) * Beta_RGR;
            *Phi = -(this->Phi + ori_Phi);
            break;
        case 2:
            *Alpha = (this->Alpha - ori_Alpha);
            *Beta = (this->Beta - ori_Beta) * Beta_RGR;
            *Phi = (this->Phi - ori_Phi);
            break;
        case 3:
            *Alpha = -(this->Alpha - ori_Alpha);
            *Beta = -(this->Beta - ori_Beta) * Beta_RGR;
            *Phi = (this->Phi + ori_Phi);
            break;
    }
}

Eigen::Vector3f Step::Force_sensor(float Alpha, float Beta, float Phi, 
                                    float T_alpha, float T_Beta, float T_Phi)
{
    const float h = 1e-6f;
    
    auto f = [&](float a, float b, float p) {
        return Leg_Calculate_Inverse(a, b, p);
    };

    Eigen::Matrix3f J;
    J.col(0) = (f(Alpha + h, Beta, Phi) - f(Alpha - h, Beta, Phi)) / (2 * h);
    J.col(1) = (f(Alpha, Beta + h, Phi) - f(Alpha, Beta - h, Phi)) / (2 * h);
    J.col(2) = (f(Alpha, Beta, Phi + h) - f(Alpha, Beta, Phi - h)) / (2 * h);

    return J.transpose().lu().solve(Eigen::Vector3f(T_alpha, T_Beta, T_Phi));
}

}
