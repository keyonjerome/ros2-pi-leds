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
  void set_strip_color(std_msgs::msg::ColorRGBA color);
  
  LED_CONTROLLER_LOCAL
  std_msgs::msg::ColorRGBA get_strip_color();

  // LED_CONTROLLER_LOCAL
  // void curr_color_pub_callback();

  LED_CONTROLLER_LOCAL
  void new_color_sub_callback(const std_msgs::msg::ColorRGBA msg);

  // LED_CONTROLLER_LOCAL
  // void update_curr_color();


  // publisher
  // rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr curr_color_pub;
  
  // subscriber
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr new_color_sub;

  // callback timer
  rclcpp::TimerBase::SharedPtr timer;
  
  // std::shared_ptr<std_msgs::msg::ColorRGBA> curr_color_msg = std::make_unique<std_msgs::msg::ColorRGBA>(); 

  // set quality of service depth - AKA a backlog
  static constexpr unsigned int QUEUE{10};

  static constexpr unsigned int RED_GPIO{2};
  static constexpr unsigned int GREEN_GPIO{3};
  static constexpr unsigned int BLUE_GPIO{4};



};

} // namespace led_controller

#endif //  LED_STRIP_OUTPUT_HPP_