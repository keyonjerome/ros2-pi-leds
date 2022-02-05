#ifndef BUTTON_PUB_HPP_
#define BUTTON_PUB_HPP_

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <led_strip_controller/visibility.h>

namespace led_controller {

class button_pub : public rclcpp::Node {

public:
  LED_CONTROLLER_PUBLIC
  explicit button_pub(const rclcpp::NodeOptions &options);

private:

  LED_CONTROLLER_LOCAL
  bool get_button_status();
  

  LED_CONTROLLER_LOCAL
  void curr_color_sub_callback(const std_msgs::msg::ColorRGBA msg);
  
  LED_CONTROLLER_LOCAL
  void update_new_color();

  LED_CONTROLLER_LOCAL
  std_msgs::msg::ColorRGBA new_color(float r, float g, float b, float a);


  std::map<std::string,std_msgs::msg::ColorRGBA> color_map;
  // std::shared_ptr<std_msgs::msg::ColorRGBA> curr_color_msg = std::make_unique<std_msgs::msg::ColorRGBA>(); 

  std::shared_ptr<std_msgs::msg::ColorRGBA> curr_color = std::make_unique<std_msgs::msg::ColorRGBA>();

  // subscription
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr curr_color_sub;

  // publisher
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr new_color_pub;
  // callback timer
  rclcpp::TimerBase::SharedPtr button_timer;
  rclcpp::TimerBase::SharedPtr color_pub_timer;

  // set quality of service depth - AKA a backlog
  static constexpr unsigned int QUEUE{10};


};

} // namespace led_controller

#endif //  BUTTON_PUB_HPP_