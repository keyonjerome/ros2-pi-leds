#ifndef BUTTON_PUB_HPP_
#define BUTTON_PUB_HPP_

#include <led_strip_controller/visibility.h>

#include <chrono>
#include <cstdlib>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <string>

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::chrono::system_clock;

namespace led_controller
{
class button_pub : public rclcpp::Node
{
public:
  LED_CONTROLLER_PUBLIC
  explicit button_pub(const rclcpp::NodeOptions & options);

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
  void button_status_pub();

  LED_CONTROLLER_LOCAL
  std_msgs::msg::ColorRGBA new_color(float r, float g, float b, float a);

  // keep all colors in color wheel in map
  std::map<std::string, std_msgs::msg::ColorRGBA> color_map;
  // use two maps with an int instead of using multi-indexed map (to reduce code bloat)
  std::map<int, std::string> color_int_map;
  // current color
  unsigned int curr_color_int{0};

  // subscription
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr curr_color_sub;

  // publisher
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr new_color_pub;

  // callback timer
  rclcpp::TimerBase::SharedPtr button_pub_timer;
  rclcpp::TimerBase::SharedPtr color_pub_timer;

  // set quality of service depth - a backlog
  static constexpr unsigned int QUEUE{10};
  static constexpr unsigned int BUTTON_GPIO{17};

  // define how long it takes for the button to be considered debounced
  const int64_t debounce_delay{50};

  bool last_reading{false};
  int64_t last_debounce_time;
  // hold whether the button has published a new color yet (for that button press)
  bool published{true};
};

}  // namespace led_controller

#endif  //  BUTTON_PUB_HPP_