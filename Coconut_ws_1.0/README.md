# **README**

## **📑 总览**
这个是纯粹怎么爽怎么写的，几乎没用到任何ros2的工具包  
### 架构：
## **⏳ 使用方法**
方便自己copy指令用的，可直接跳过
- 初次编译
```
colcon build --packages-select base_interfaces
source install/setup.bash
colcon build --packages-select ltyo_cpp
``` 
- 编译
```
colcon build
```
- 添加usb权限
```
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
sudo chmod 666 /dev/ttyUSB2
sudo chmod 666 /dev/ttyUSB3
```
- 使用launch文件启动位置控制节点
```
source install/setup.bash
ros2 launch ltyo_cpp angle_launch.py
```
- 使用launch文件启动力控节点
```
source install/setup.bash
ros2 launch ltyo_cpp force_launch.py
```
- 非launch文件启动位置控制节点
```
source install/setup.bash
ros2 run ltyo_cpp unitree
ros2 run ltyo_cpp ltyo
```
- 非launch文件启动力控节点
```
source install/setup.bash
ros2 run ltyo_cpp unitree
ros2 run ltyo_cpp coco
```
## **🧩 项目架构**
- 功能包与节点
  - ltyo_cpp——控制功能包
    - ltyo——位置控制节点
    - coco——力控制节点
  - motor_cpp——电机控制整合
    - unitree——宇树电机控制
  - keyboard_cpp——键盘控制
    - keyboard——键盘控制节点
- 话题
  - MotorCmd——电机指令发送
  - MotorData——电机状态接收
  - keyboard——键盘指令接收
- 其他
  - motor.cpp——电机控制整合
  - step.cpp——位置控制的运动学解算
  - Leg.cpp——力矩（VMC）的运动学解算
  - BodyPose.cpp——步态解算

## **🛠️ 控制程序替换**
**仅作为架构文件使用不需要控制程序时，可按照下述步骤删除控制程序**
1. 将构造函数 ***LtYO() : Node("ltyo")*** 替换成下面的并加入你自己的初始化函数
```
class LtYO : public rclcpp::Node
{
public:
  LtYO() : Node("ltyo")
  {
    // 默认电机参数
    motor_cmd_.mode = 1;
    motor_cmd_.id = 5;
    motor_cmd_.line = 0;
    motor_cmd_.kd = 0.01;
    motor_cmd_.kp = 0.0;
    motor_cmd_.tau = 0.0;
    motor_cmd_.q = 0;
    motor_cmd_.dq = 0;
    

    motor_data_sub_ = this->create_subscription<base_interfaces::msg::MotorData>(
      "motor_data", 10, std::bind(&LtYO::motor_data_callback, this, std::placeholders::_1));

    motor_cmd_pub_ = this->create_publisher<base_interfaces::msg::MotorCmd>(
      "motor_cmd", 10);
    timer_ = this->create_wall_timer(
      1ms, std::bind(&LtYO::timer_callback, this));
    Control_timer_ = this->create_wall_timer(
      1ms, std::bind(&LtYO::control_timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "LtYO node has been started");
  }
};
```

2. 将[CMakeLists.txt](./src/ltyo_cpp/CMakeLists.txt)中
```
add_executable(ltyo src/ltyo.cpp src/motor.cpp src/step.cpp src/BodyPose.cpp src/FSM.cpp)
add_executable(coco src/coco.cpp src/motor.cpp src/Leg.cpp src/BodyPose.cpp src/FSM.cpp)
```
替换为
```
add_executable(ltyo src/ltyo.cpp)
```
3. 删除[CMakeLists.txt](./src/ltyo_cpp/CMakeLists.txt)中如下所示的代码段
```
target_include_directories(coco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(coco PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
```
```
ament_target_dependencies(
  coco
  "rclcpp"
  "base_interfaces"
)
```
```
install(TARGETS coco
  DESTINATION lib/${PROJECT_NAME})
```
4. 把[ltyo.cpp](./src/ltyo_cpp/src/ltyo.cpp)的 ***void LtYO::control_timer_callback()的实现*** 替换为你自己的控制程序，并对应修改[CMakeLists.txt](./src/ltyo_cpp/CMakeLists.txt)中的描述
5. 最后按照位置控制的launch启动即可
- 控制程序替换后[BodyPose.cpp](./src/ltyo_cpp/src/BodyPose.cpp)、[FSM.cpp](./src/ltyo_cpp/src/FSM.cpp)、[Leg.cpp](./src/ltyo_cpp/src/Leg.cpp)、[step.cpp](./src/ltyo_cpp/src/step.cpp)和[coco.cpp](./src/ltyo_cpp/src/coco.cpp)及其 *hpp文件* 可以选择性删除
