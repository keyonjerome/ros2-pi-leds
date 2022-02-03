#ifndef LED_STRIP_OUTPUT_HPP_
#define LED_STRIP_OUTPUT_HPP_

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <led_strip_controller/visibility.h>

namespace led_controller {

class led_strip_output : public rclcpp::Node {

public:
  LED_CONTROLLER_PUBLIC
  explicit led_strip_output(const rclcpp::NodeOptions &options);

private:
  
  using RGBA = std_msgs::msg::ColorRGBA;

  LED_CONTROLLER_LOCAL
  void set_strip_color();
  
  LED_CONTROLLER_LOCAL
  std_msgs::msg::ColorRGBA get_strip_color();

  LED_CONTROLLER_LOCAL
  void curr_color_pub_callback();

  // publisher
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr curr_colour_pub;
  
  // publisher
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr new_colour_sub;

  // callback timer
  rclcpp::TimerBase::SharedPtr timer;

  // set quality of service depth - AKA a backlog
  static constexpr unsigned int QUEUE{10};


};

} // namespace led_controller

#endif //  LED_STRIP_OUTPUT_HPP_