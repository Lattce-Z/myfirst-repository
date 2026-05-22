#include "pid.hpp"
#include <algorithm>

namespace pid
{

PID::PID(float kp, float ki, float kd,
         float max_integral, float max_output)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _max_integral = max_integral;
    _max_output = max_output;
    _integral = 0.0;
    _prev_error = 0.0;
}

void PID::set_K(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PID::set_M(float max_integral, float max_output)
{
    _max_integral = max_integral;
    _max_output = max_output;
}

float PID::update(float Target, float Current, float dt)
{
    float error = Target - Current;
    _integral += error * dt;
    _integral = std::clamp(_integral, -_max_integral, _max_integral);
    float derivative = (error - _prev_error) / dt;
    float output = _kp * error + _ki * _integral + _kd * derivative;
    output = std::clamp(output, -_max_output, _max_output);
    _prev_error = error;
    return output;
}

}  // namespace pid