#include "rclcpp/rclcpp.hpp"
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "base_interfaces/msg/m_cmd.hpp"
#include "base_interfaces/msg/m_data.hpp"
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include "pid.hpp"
#include "Tools.hpp"
#include "serialPort/include/errorClass.h"

using namespace std::chrono_literals;

// 12个电机的总线与ID配置
static constexpr int8_t CONFIG_BUS[12] = {0,0,0, 1,1,1, 2,2,2, 3,3,3};
static constexpr int8_t CONFIG_ID[12]  = {6,2,4, 9,1,4, 7,3,5, 8,4,3};

/** 单条总线上的电机运行时数据 */
struct MotorSlot {
    MotorCmd  cmd;
    MotorData data;
    pid::PID  pid_ctrl;
    int  mode     = 0;   // 0:失能 1:位置 2:力矩
    int  mem_mode = 0;
    float target  = 0.0f;
    float ori_angle = 0.0f;
    int8_t id = -1;
    bool  online = false;

    MotorSlot() : pid_ctrl(2.8f, 0.12f, 2.2f, 2.0f, 10.0f) {}
};

/** 单条总线的上下文 */
struct BusContext {
    int bus_id;
    std::shared_ptr<SerialPort> port;
    MotorSlot slots[3];
    int slot_count = 0;
    int slot_index = 0;
    std::mutex mtx;
    std::unique_ptr<Tools::PeriodicTask> task;

    // 无锁命令缓冲：订阅回调写入此处，总线线程原子交换
    // pending_mode[i] == -1 表示无新命令
    std::atomic<int>   pending_mode[3];
    std::atomic<float> pending_target[3];

    BusContext(int id) : bus_id(id) {
        for(int i = 0; i < 3; i++){
            pending_mode[i].store(-1, std::memory_order_release);
            pending_target[i].store(0.0f, std::memory_order_release);
        }
    }

    /** 检查并消费待处理命令（由总线线程调用，已在锁内） */
    void consume_pending(int local_idx) {
        int m = pending_mode[local_idx].exchange(-1, std::memory_order_acq_rel);
        if(m < 0) return; // 无新命令
        auto& slot = slots[local_idx];
        slot.mode = m;
        if(m == 1){
            slot.target = pending_target[local_idx].load(std::memory_order_acquire)
                          * queryGearRatio(MotorType::GO_M8010_6);
        } else {
            slot.target = pending_target[local_idx].load(std::memory_order_acquire);
        }
    }
};

namespace unitree
{

// 全局电机数据缓存（供 1kHz 发布用）
static float g_q[12]    = {0};
static float g_dq[12]   = {0};
static float g_tau[12]  = {0};
static int32_t g_temp[12] = {0};
static std::mutex g_data_mtx;

class Unitree : public rclcpp::Node
{
public:
    BusContext bus_ctx[4] = {0, 1, 2, 3};

    Unitree() : Node("unitree")
    {
        // 扫描并初始化所有总线上的电机
        for(int i = 0; i < 12; i++){
            int bus = CONFIG_BUS[i];
            int id  = CONFIG_ID[i];
            auto& ctx = bus_ctx[bus];
            int& cnt = ctx.slot_count;
            if(cnt >= 3) continue;

            MotorCmd  cmd;
            MotorData data;
            cmd.motorType = MotorType::GO_M8010_6;
            data.motorType = MotorType::GO_M8010_6;
            cmd.id = id;
            cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
            cmd.kp = 0;
            cmd.kd = 0.15f;
            cmd.q  = 0;
            cmd.dq = 0;
            cmd.tau = 0;

            auto port = std::make_shared<SerialPort>(get_port_name(bus));

            bool online = false;
            try {
                online = port->sendRecv(&cmd, &data);
            } catch (const IOException&) {
                online = false;
            }

            auto& slot = ctx.slots[cnt];
            slot.id = id;
            slot.cmd = cmd;
            slot.data = data;
            slot.ori_angle = data.q;
            slot.online = (online && data.merror == 0);

            if(slot.online){
                slot.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                RCLCPP_INFO(this->get_logger(), "Motor id=%d bus=%d ONLINE", id, bus);
            } else {
                RCLCPP_WARN(this->get_logger(), "Motor id=%d bus=%d no response", id, bus);
            }

            if(cnt == 0) ctx.port = port;
            cnt++;
        }

        RCLCPP_INFO(this->get_logger(), "Init done. %d motors online", count_online());

        // 创建发布者和订阅者
        motor_data_pub_ = this->create_publisher<base_interfaces::msg::MData>("motor_data", 10);
        motor_cmd_sub_ = this->create_subscription<base_interfaces::msg::MCmd>(
            "motor_cmd", 10, std::bind(&Unitree::motor_cmd_callback, this, std::placeholders::_1));

        // 启动4条总线的电机控制线程（1kHz）
        for(int b = 0; b < 4; b++){
            auto& ctx = bus_ctx[b];
            if(ctx.slot_count == 0) continue;
            ctx.task = std::make_unique<Tools::PeriodicTask>(
                [this, b](){ this->bus_thread_func(b); }, 1000);
            ctx.task->start();
            RCLCPP_INFO(this->get_logger(), "Bus %d thread started, %d motors", b, ctx.slot_count);
        }

        // 启动数据发布定时器（200Hz，主动反馈）
        pub_timer_ = this->create_wall_timer(5ms, std::bind(&Unitree::pub_timer_callback, this));
    }

    ~Unitree(){
        for(int b = 0; b < 4; b++){
            if(bus_ctx[b].task) bus_ctx[b].task->stop();
        }
    }

private:
    void motor_cmd_callback(const base_interfaces::msg::MCmd::SharedPtr msg);
    void bus_thread_func(int bus_id);
    void pub_timer_callback();
    int count_online();

    std::string get_port_name(int bus) {
        switch(bus){
            case 0: return "/dev/ttyUSB0";
            case 1: return "/dev/ttyUSB1";
            case 2: return "/dev/ttyUSB2";
            case 3: return "/dev/ttyUSB3";
            default: return "/dev/ttyUSB0";
        }
    }

    rclcpp::Publisher<base_interfaces::msg::MData>::SharedPtr motor_data_pub_;
    rclcpp::Subscription<base_interfaces::msg::MCmd>::SharedPtr motor_cmd_sub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
};

int Unitree::count_online(){
    int n = 0;
    for(int b = 0; b < 4; b++)
        for(int s = 0; s < bus_ctx[b].slot_count; s++)
            if(bus_ctx[b].slots[s].online) n++;
    return n;
}

// ====== 控制指令接收（订阅回调，写入无锁缓冲，绝不阻塞）======
void Unitree::motor_cmd_callback(const base_interfaces::msg::MCmd::SharedPtr msg)
{
    for(int i = 0; i < 12; i++){
        if(msg->mode[i] == 0 && msg->target[i] == 0.0f) continue; // 没变化跳过

        int bus   = i / 3;
        int local = i % 3;
        auto& ctx = bus_ctx[bus];
        if(local >= ctx.slot_count) continue;

        // 写入无锁缓冲，总线线程会消费
        ctx.pending_mode[local].store(msg->mode[i], std::memory_order_release);
        ctx.pending_target[local].store(msg->target[i], std::memory_order_release);
    }
}

// ====== 总线线程函数（每个总线独立线程，1kHz，每次只收1个电机）======
void Unitree::bus_thread_func(int bus_id)
{
    auto& ctx = bus_ctx[bus_id];
    if(ctx.slot_count == 0) return;

    int s = ctx.slot_index;

    std::lock_guard<std::mutex> lock(ctx.mtx);

    // 消费待处理命令（如果订阅回调有写入）—— 不需要额外的锁
    ctx.consume_pending(s);

    auto& slot = ctx.slots[s];

    if(slot.online){
        if(slot.mode == 1){
            if(slot.mem_mode != 1){
                slot.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                slot.cmd.kd = 0.15f;
                slot.cmd.tau = 0;
                slot.mem_mode = 1;
            }
            slot.cmd.dq = slot.pid_ctrl.update(slot.target, slot.data.q - slot.ori_angle, 3)
                          * queryGearRatio(MotorType::GO_M8010_6);
            try { ctx.port->sendRecv(&slot.cmd, &slot.data); }
            catch (const IOException&) {}
        }
        else if(slot.mode == 2){
            if(slot.mem_mode != 2){
                slot.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                slot.cmd.kd = 0.0f;
                slot.cmd.dq = 0;
                slot.mem_mode = 2;
            }
            slot.cmd.tau = slot.target / queryGearRatio(MotorType::GO_M8010_6);
            try { ctx.port->sendRecv(&slot.cmd, &slot.data); }
            catch (const IOException&) {}
        }
        else {
            if(slot.mem_mode != 0){
                slot.cmd.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                slot.cmd.kd = 0.15f;
                slot.cmd.dq = 0;
                slot.cmd.tau = 0;
                slot.mem_mode = 0;
            }
            try { ctx.port->sendRecv(&slot.cmd, &slot.data); }
            catch (const IOException&) {}
        }

        // 写入全局数据缓存（供发布用）
        {
            int global_idx = bus_id * 3 + s;
            std::lock_guard<std::mutex> dlock(g_data_mtx);
            g_q[global_idx]    = (slot.data.q - slot.ori_angle) / queryGearRatio(MotorType::GO_M8010_6);
            g_dq[global_idx]   = slot.data.dq / queryGearRatio(MotorType::GO_M8010_6);
            g_tau[global_idx]  = slot.data.tau;
            g_temp[global_idx] = slot.data.temp;
        }
    }

    ctx.slot_index = (s + 1) % ctx.slot_count;
}

// ====== 数据发布定时器（200Hz 主动反馈）======
void Unitree::pub_timer_callback()
{
    base_interfaces::msg::MData msg;
    {
        std::lock_guard<std::mutex> dlock(g_data_mtx);
        for(int i = 0; i < 12; i++){
            msg.q[i]    = g_q[i];
            msg.dq[i]   = g_dq[i];
            msg.tau[i]  = g_tau[i];
            msg.temp[i] = g_temp[i];
        }
    }
    motor_data_pub_->publish(msg);
}

} // namespace unitree

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<unitree::Unitree>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
