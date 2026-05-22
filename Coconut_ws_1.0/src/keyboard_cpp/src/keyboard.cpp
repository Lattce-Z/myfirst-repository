#include "rclcpp/rclcpp.hpp"
#include "base_interfaces/msg/keyboard.hpp"
#include <chrono>
#include "md.hpp"

using namespace std::chrono_literals;

namespace keyboard
{
  class Keyboard : public rclcpp::Node{
    public:
      Keyboard() : Node("keyboard"){
        publisher_ = this->create_publisher<base_interfaces::msg::Keyboard>("keyboard", 10);
        timer_ = this->create_wall_timer(10ms, std::bind(&Keyboard::timer_callback, this));
        setup_terminal();
      }

    private:
      void timer_callback(){
        auto message = base_interfaces::msg::Keyboard();
        message.value = read_key();
        if(message.value != 0){
          publisher_->publish(message);
          RCLCPP_INFO(this->get_logger(), "I heard: '%c'", message.value);
        }
      }

      rclcpp::Publisher<base_interfaces::msg::Keyboard>::SharedPtr publisher_;
      rclcpp::TimerBase::SharedPtr timer_;
  };
} // namespace keyboard

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<keyboard::Keyboard>());
  rclcpp::shutdown();
  return 0;
}