#include <pigpio.h>

#include <chrono>
#include <led_strip_controller/led_strip_output.hpp>

using namespace std::literals::chrono_literals;
using namespace std::placeholders;

namespace led_controller
{
led_strip_output::led_strip_output(const rclcpp::NodeOptions & options)
: Node("led_output", options)
{
  // Initialize pigpio before use.
  if (gpioInitialise() < 0) {
    RCLCPP_ERROR(this->get_logger(), "GPIO INITIALIZE FAIL");
  } else {
    RCLCPP_INFO(this->get_logger(), "GPIO INITIALIZE SUCCESS");
  }

  // Subscribe to publishes to "led_controller1/new_color"; whenever a new color is published, set the LED strip to that color.
  this->new_color_sub = this->create_subscription<std_msgs::msg::ColorRGBA>(
    "led_controller1/new_color", 10,
    std::bind(&led_strip_output::new_color_sub_callback, this, _1));
}

// Whenever a new color is published, set the LED strip to that color.
void led_strip_output::new_color_sub_callback(const std_msgs::msg::ColorRGBA msg)
{
  set_strip_color(msg);
}

// Set the LED strip color based on the ColorRGBA passed in.
void led_strip_output::set_strip_color(const std_msgs::msg::ColorRGBA color)
{
  // Set the LEDS using gpioPWM and print an error if it is not successful.
  if (gpioPWM(RED_GPIO, static_cast<int>(color.r)) != 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "SET_STRIP_COLOR ERROR: PI_BAD_USER_GPIO or PI_BAD_DUTYCYCLE WITH GPIO ADDRESS %i", RED_GPIO);
  }
  if (gpioPWM(GREEN_GPIO, static_cast<int>(color.g)) != 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "SET_STRIP_COLOR ERROR: PI_BAD_USER_GPIO or PI_BAD_DUTYCYCLE WITH GPIO ADDRESS %i",
      GREEN_GPIO);
  }
  if (gpioPWM(BLUE_GPIO, static_cast<int>(color.b)) != 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "SET_STRIP_COLOR ERROR: PI_BAD_USER_GPIO or PI_BAD_DUTYCYCLE WITH GPIO ADDRESS %i",
      BLUE_GPIO);
  }
}

}  // namespace led_controller

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(led_controller::led_strip_output)
