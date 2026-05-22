#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/msg/m_cmd.hpp"
#include "base_interfaces/msg/m_data.hpp"
#include <chrono>
#include <vector>
#include <cstring>

#include "step.hpp"
#include "TrotGait.hpp"

#define USEGAMEPAD

#ifdef USEGAMEPAD
#include "sensor_msgs/msg/joy.hpp"
#else
#include "base_interfaces/msg/keyboard.hpp"
#endif

using namespace std::chrono_literals;

namespace ltyo
{
class LtYO : public rclcpp::Node
{
public:
    LtYO() : Node("ltyo")
    {
        // 初始化 12 个电机的控制指令
        for(int i = 0; i < 12; i++)
        {
            motor_cmd_.mode[i] = 0;
            motor_cmd_.target[i] = 0.0f;
            step_.push_back(step::Step(i));
        }
        cooder_cmd = {Eigen::Vector3f(0,-120,0), Eigen::Vector3f(0,-120,0), Eigen::Vector3f(0,-120,0), Eigen::Vector3f(0,-120,0)};
        trot_ = TrotGait();
        trot_.setPeriod(0.5f); // 1s

        // ros2 初始化
        motor_data_sub_ = this->create_subscription<base_interfaces::msg::MData>(
            "motor_data", 10, std::bind(&LtYO::motor_data_callback, this, std::placeholders::_1));
        motor_cmd_pub_ = this->create_publisher<base_interfaces::msg::MCmd>(
            "motor_cmd", 10);
        #ifdef USEGAMEPAD
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&LtYO::joy_callback, this, std::placeholders::_1)
        );
        #else
        keyboard_sub_ = this->create_subscription<base_interfaces::msg::Keyboard>(
            "keyboard", 10, std::bind(&LtYO::keyboard_callback, this, std::placeholders::_1));
        #endif

        // 控制周期 200Hz = 5ms
        control_timer_ = this->create_wall_timer(
            5ms, std::bind(&LtYO::control_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "LtYO node started, control at 200Hz");
    }

private:
    void motor_data_callback(const base_interfaces::msg::MData::SharedPtr msg);
    void control_timer_callback();
    #ifdef USEGAMEPAD
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    #else
    void keyboard_callback(const base_interfaces::msg::Keyboard::SharedPtr msg);
    #endif

    static constexpr int motor_num = 12;

    /* 全局参数 */
    float n1 = 0, n2 = 0, state = 0;
    int count = 0;

    base_interfaces::msg::MCmd motor_cmd_;          // 一次发12个
    base_interfaces::msg::MData motor_data_;          // 一次收12个
    #ifdef USEGAMEPAD
    sensor_msgs::msg::Joy joy_ = [](){
      sensor_msgs::msg::Joy j;
      j.axes.resize(6, 0.0f);
      j.buttons.resize(10, 0);
      return j;
    }();
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    #else
    base_interfaces::msg::Keyboard keyboard_;
    rclcpp::Subscription<base_interfaces::msg::Keyboard>::SharedPtr keyboard_sub_;
    #endif
    std::vector<step::Step> step_;
    TrotGait trot_;
    std::array<Eigen::Vector3f, 4> cooder_cmd;
    std::array<Eigen::Vector3f, 4> cooder_feedback_;
    std::array<float, 12> q_tmp;
    Eigen::Vector3f V = Eigen::Vector3f(0, 0, 0);// 机体期望速度
    float A = 0.0f;// 机体期望角速度

    rclcpp::Subscription<base_interfaces::msg::MData>::SharedPtr motor_data_sub_;
    rclcpp::Publisher<base_interfaces::msg::MCmd>::SharedPtr motor_cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

void LtYO::control_timer_callback()
{

    /* 控制程序 begin */

    if(joy_.axes[0] + joy_.axes[1] > 0.1 || joy_.axes[0]+joy_.axes[1] < -0.1 || joy_.axes[3] > 0.1 || joy_.axes[3] < -0.1)
    {
        if(n1 == 0)
        {
            n1 = 1;
            trot_.reset();
        }
        state = 1;
        V[0] = joy_.axes[1]*700;
        V[2] = joy_.axes[0]*200;
        A = joy_.axes[3]*0.3;
    }
    else if(joy_.axes[1] <= 0.1 || joy_.axes[1] >= -0.1)
    {
        if(n1 == 1)
        {
            n1 = 0;
        }
        state = 0;
    }

    if(count == 0)
    {
        for(int i = 0; i < 12; i++)
        {
            motor_cmd_.mode[i] = 1;
        }
        count ++;

    }
    if(count < 280)
    {
        for(int i = 0; i < 4; i++)
        {
            cooder[i][1] = -120-0.5*count;
        }
        count ++;
    }
    else
    {
        if(state == 0)
        {
            for(int i = 0; i < 4; i++)
            {
                cooder[i][1] = -260;
            }
        }
        else if(state == 1)
        {
            cooder = trot_.update(0.005, V, A, cooder_feedback_);
        }
    }

    /* 控制程序 end */

    // 计算并赋值
    for(int i = 0; i < 4; i++)
    {
        // 机体倾斜
        // if(i == 0 || i == 1){
        //     cooder[i][1] -= 321.7f * tan(10*M_PI/180);
        // }else{
        //     cooder[i][1] += 321.7f * tan(10*M_PI/180);
        // }
        // cooder[i] = Eigen::AngleAxisf(10*M_PI/180, Eigen::Vector3f::UnitZ()).toRotationMatrix() *cooder[i];

        try(){
            float tmp[3] = {};
            step_[i].Leg_Calculate(cooder_cmd[i]);
            step_[i].Data_Output(&tmp[0], &tmp[1], &tmp[2]);
            Eigen::Vector3f cooder = step_[i].Leg_Calculate_Inverse(tmp[0], tmp[1], tmp[2]);
            if((cooder - cooder_cmd[i]).norm() > 10)
            {
                RCLCPP_ERROR(this->get_logger(), "cooder error");
                throw std::runtime_error("cooder error");
            }
            q_tmp[3*i] = tmp[0];
            q_tmp[3*i+1] = tmp[1];
            q_tmp[3*i+2] = tmp[2];
        }catch(const std::exception& e){}
        motor_cmd_.target[3*i] = q_tmp[3*i];
        motor_cmd_.target[3*i+1] = q_tmp[3*i+1];
        motor_cmd_.target[3*i+2] = q_tmp[3*i+2];

        cooder_feedback_[i] = step_[i].Leg_Calculate_Inverse(motor_data_.q[3*i], motor_data_.q[3*i+1], motor_data_.q[3*i+2]);
        
    }
    // 一次发布所有12个电机的指令
    motor_cmd_pub_->publish(motor_cmd_);
}

// ========电机数据反馈回调========
void LtYO::motor_data_callback(const base_interfaces::msg::MData::SharedPtr msg)
{
    motor_data_ = *msg;
}

// ========控制数据回调函数========
#ifdef USEGAMEPAD
void LtYO::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    joy_ = *msg;
    /*
    axes[0]  左摇杆水平
    axes[1]  左摇杆竖直
    axes[2]  右摇杆水平
    axes[3]  右摇杆竖直
    axes[4]  左侧拨轮
    axes[5]  右侧拨轮

    buttons[0]  左上按键
    buttons[1]  右上按键
    buttons[2]  A
    buttons[3]  B
    buttons[4]  X
    buttons[5]  Y
    buttons[6]  十字键上
    buttons[7]  十字键下
    buttons[8]  十字键左
    buttons[9]  十字键右
    */
}
#else
void LtYO::keyboard_callback(const base_interfaces::msg::Keyboard::SharedPtr msg)
{
    keyboard_ = *msg;
}
#endif

}// namespace ltyo

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ltyo::LtYO>());
    rclcpp::shutdown();
    return 0;
}
