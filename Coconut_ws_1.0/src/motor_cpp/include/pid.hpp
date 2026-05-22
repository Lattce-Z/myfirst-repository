#ifndef __PID_HPP__
#define __PID_HPP__

namespace pid
{
class PID
{
public:
    PID(float kp, float ki, float kd,
        float max_integral, float max_output);

    void set_K(float kp, float ki, float kd);
    void set_M(float max_integral, float max_output);
    
    float update(float Target, float Current, float dt);

private:
    float _kp;
    float _ki;
    float _kd;
    float _max_integral;
    float _max_output;
    float _integral;
    float _prev_error;
};

}

#endif