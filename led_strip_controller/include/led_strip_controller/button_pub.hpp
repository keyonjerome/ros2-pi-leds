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
  void button_loop();

  LED_CONTROLLER_LOCAL
  std_msgs::msg::ColorRGBA new_color(float r, float g, float b, float a);


  std::map<std::string,std_msgs::msg::ColorRGBA> color_map;
  // std::shared_ptr<std_msgs::msg::ColorRGBA> curr_color_msg = std::make_unique<std_msgs::msg::ColorRGBA>(); 

  std::shared_ptr<std_msgs::msg::ColorRGBA> curr_color = std::make_unique<std_msgs::msg::ColorRGBA>();
  std::map<int,std::string> color_int_map;
  std::shared_ptr<int> curr_color_int{0};

  // subscription
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr curr_color_sub;

  // publisher
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr new_color_pub;
  // callback timer
  rclcpp::TimerBase::SharedPtr button_timer;
  rclcpp::TimerBase::SharedPtr color_pub_timer;

  // set quality of service depth - AKA a backlog
  static constexpr unsigned int QUEUE{10};

  static constexpr unsigned int BUTTON_GPIO{17};
  static constexpr float HOLD_TIME{0.25};

  static constexpr long debounce_delay{50};
  // Timer button_t;
  bool last_reading{false};
  long last_debounce_time{50};
  bool published{true};
  int color_counter = 0;


};

} // namespace led_controller

#endif //  BUTTON_PUB_HPP_