#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <libusb-1.0/libusb.h>
#include <memory>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

// XBOX 360 2.4G Wireless Gamepad
#define VENDOR_ID  0x3537
#define PRODUCT_ID 0x1040

#define INTERFACE_NUM  0
#define EP_IN          0x81  // interrupt IN endpoint
#define TIMEOUT_MS     100

namespace gamepad_cpp
{

class GamepadNode : public rclcpp::Node
{
public:
  GamepadNode() : Node("gamepad_node"), ctx_(nullptr), handle_(nullptr)
  {
    // 声明参数
    this->declare_parameter<int>("publish_hz", 100);
    int hz = this->get_parameter("publish_hz").as_int();
    auto period = std::chrono::milliseconds(1000 / hz);

    publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
    timer_ = this->create_wall_timer(period, std::bind(&GamepadNode::timer_callback, this));

    // 初始化libusb并打开手柄
    if (!init_libusb()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init gamepad. Node will spin but no data will be published.");
    }
  }

  ~GamepadNode()
  {
    cleanup_libusb();
  }

private:
  bool init_libusb()
  {
    int ret = libusb_init(&ctx_);
    if (ret < 0) {
      RCLCPP_ERROR(this->get_logger(), "libusb_init failed: %s", libusb_error_name(ret));
      return false;
    }

    handle_ = libusb_open_device_with_vid_pid(ctx_, VENDOR_ID, PRODUCT_ID);
    if (!handle_) {
      RCLCPP_ERROR(this->get_logger(),
        "Failed to open device %04x:%04x. "
        "Check: (1) Is the 2.4G receiver plugged in? "
        "(2) Do you have permission? "
        "(3) Is the udev rule at /etc/udev/rules.d/99-gamepad.rules active?",
        VENDOR_ID, PRODUCT_ID);
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Gamepad device opened successfully.");

    // Detach kernel driver if active
    if (libusb_kernel_driver_active(handle_, INTERFACE_NUM) == 1) {
      ret = libusb_detach_kernel_driver(handle_, INTERFACE_NUM);
      if (ret < 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to detach kernel driver: %s", libusb_error_name(ret));
      } else {
        RCLCPP_INFO(this->get_logger(), "Kernel driver detached.");
      }
    }

    ret = libusb_claim_interface(handle_, INTERFACE_NUM);
    if (ret < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to claim interface: %s", libusb_error_name(ret));
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Interface claimed. Gamepad ready.");
    return true;
  }

  void cleanup_libusb()
  {
    if (handle_) {
      libusb_release_interface(handle_, INTERFACE_NUM);
      libusb_close(handle_);
      handle_ = nullptr;
    }
    if (ctx_) {
      libusb_exit(ctx_);
      ctx_ = nullptr;
    }
  }

  bool read_report(uint8_t *buf, int size)
  {
    if (!handle_) return false;

    int transferred = 0;
    int ret = libusb_interrupt_transfer(handle_, EP_IN, buf, size, &transferred, TIMEOUT_MS);
    if (ret == 0 && transferred == size) {
      return true;
    } else if (ret == LIBUSB_ERROR_TIMEOUT) {
      return false;
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "USB read error: %s (ret=%d, transferred=%d)",
        libusb_error_name(ret), ret, transferred);
      return false;
    }
  }

  void timer_callback()
  {
    uint8_t buf[20];
    if (!read_report(buf, sizeof(buf))) {
      return;
    }

    auto msg = sensor_msgs::msg::Joy();

    // timestamp
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "gamepad";

    // axes[6]: [lx, ly, rx, ry, lt, rt]
    // 摇杆: 0~65535 -> -32768~32767 -> normalize to -1.0~1.0
    int16_t lx_raw = (int16_t)(buf[6] | (buf[7] << 8));
    int16_t ly_raw = (int16_t)(buf[8] | (buf[9] << 8));
    int16_t rx_raw = (int16_t)(buf[10] | (buf[11] << 8));
    int16_t ry_raw = (int16_t)(buf[12] | (buf[13] << 8));

    msg.axes.resize(6);
    msg.axes[0] = lx_raw / 32767.0f;   // LX
    msg.axes[1] = ly_raw / 32767.0f;   // LY
    msg.axes[2] = rx_raw / 32767.0f;   // RX
    msg.axes[3] = ry_raw / 32767.0f;   // RY
    msg.axes[4] = buf[4] / 255.0f;     // LT (0~255 -> 0.0~1.0)
    msg.axes[5] = buf[5] / 255.0f;     // RT (0~255 -> 0.0~1.0)

    // buttons[10]: [LB, RB, A, B, X, Y, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT]
    uint8_t btn = buf[3];
    uint8_t dpad = buf[2];
    msg.buttons.resize(10);
    msg.buttons[0] = (btn & 0x01) ? 1 : 0;   // LB
    msg.buttons[1] = (btn & 0x02) ? 1 : 0;   // RB
    msg.buttons[2] = (btn & 0x10) ? 1 : 0;   // A
    msg.buttons[3] = (btn & 0x20) ? 1 : 0;   // B
    msg.buttons[4] = (btn & 0x40) ? 1 : 0;   // X
    msg.buttons[5] = (btn & 0x80) ? 1 : 0;   // Y
    msg.buttons[6] = (dpad & 0x01) ? 1 : 0;  // DPAD_UP
    msg.buttons[7] = (dpad & 0x02) ? 1 : 0;  // DPAD_DOWN
    msg.buttons[8] = (dpad & 0x04) ? 1 : 0;  // DPAD_LEFT
    msg.buttons[9] = (dpad & 0x08) ? 1 : 0;  // DPAD_RIGHT

    publisher_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(),
      "Joy: axes[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] buttons[%d,%d,%d,%d,%d,%d]",
      msg.axes[0], msg.axes[1], msg.axes[2], msg.axes[3], msg.axes[4], msg.axes[5],
      msg.buttons[0], msg.buttons[1], msg.buttons[2],
      msg.buttons[3], msg.buttons[4], msg.buttons[5]);
  }

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  libusb_context *ctx_;
  libusb_device_handle *handle_;
};

} // namespace gamepad_cpp

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<gamepad_cpp::GamepadNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
