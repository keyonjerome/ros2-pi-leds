#include <pigpio.h>
#include <sys/time.h>

#include <chrono>
#include <ctime>
#include <led_strip_controller/button_pub.hpp>

using namespace std::literals::chrono_literals;
using namespace std::placeholders;

namespace led_controller
{
// Helper function for creating ColorRGBA msg objects.
std_msgs::msg::ColorRGBA button_pub::new_color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

button_pub::button_pub(const rclcpp::NodeOptions & options) : Node("button_pub", options)
{
  // use internal raspberry pi pull up resistor on button (to avoid need for second resistor)
  gpioSetPullUpDown(BUTTON_GPIO, PI_PUD_UP);

  // Define publishers and subscribers
  this->new_color_pub = this->create_publisher<std_msgs::msg::ColorRGBA>(
    "led_controller1/new_color", rclcpp::QoS(QUEUE));

  // Query the button status every 10ms.
  this->button_pub_timer =
    this->create_wall_timer(10ms, std::bind(&button_pub::button_status_pub, this));

  // It would be better to create a custom msg and use a multi-index map for this,
  // but in the interest of time this is alright.
  std::string white = "white";
  std::string red = "red";
  std::string blue = "blue";
  std::string purple = "purple";
  std::string medium_slate_blue = "medium_slate_blue";
  std::string off = "off";

  // Define all colors
  this->color_map[white] = new_color(255, 255, 255, 0);
  this->color_map[red] = new_color(255, 0, 0, 0);
  this->color_map[blue] = new_color(0, 0, 255, 0);
  this->color_map[purple] = new_color(255, 0, 255, 0);
  this->color_map[medium_slate_blue] = new_color(106, 90, 205, 0);
  this->color_map[off] = new_color(0, 0, 0, 0);

  // using two maps instead of a multi-index map, because multi-index maps would greatly increase code bloat
  // this defines the ordering of the colors when the button is pressed
  this->color_int_map[0] = white;
  this->color_int_map[1] = red;
  this->color_int_map[2] = blue;
  this->color_int_map[3] = purple;
  this->color_int_map[4] = medium_slate_blue;
  this->color_int_map[5] = off;
}

// Get system time in milliseconds.
int64_t get_time_millis()
{
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

// Query the button and if it is pressed & debounced, send a new color to the LED strip.
void button_pub::button_status_pub()
{
  bool reading = get_button_status();

  //   RCLCPP_INFO(this->get_logger(), "BUTTON: %s ", reading ? "TRUE" : "FALSE");

  auto curr_time = get_time_millis();

  if (last_reading != reading) {
    last_debounce_time = curr_time;
    // RCLCPP_INFO(this->get_logger(), "DEBOUNCE TIME UPDATE: %li", last_debounce_time);
    published = false;
  }

  //   RCLCPP_INFO(this->get_logger(), "TIME DIFF %li", (curr_time - last_debounce_time));
  //   If the button value has not changed for the debounce delay, we know it's stable.
  //   If it's stable and the button is pressed (true), and we haven't changed the color yet (published), then change the color.
  if (!published && ((curr_time - last_debounce_time) > debounce_delay) && reading) {
    RCLCPP_INFO(this->get_logger(), "UPDATING COLOR FROM BUTTON PRESS");
    update_new_color();
    published = true;
  }

  last_reading = reading;
}

// Get the button status from GPIO (true if pressed).
bool button_pub::get_button_status() { return !(gpioRead(BUTTON_GPIO) == 1); }

// Activates a color change.
void button_pub::update_new_color()
{
  this->curr_color_int++;
  // reset to first color if at end of color wheel
  if (this->curr_color_int == this->color_int_map.size()) this->curr_color_int = 0;
  // use map to get the string for the next color
  std::string color = this->color_int_map[this->curr_color_int];
  RCLCPP_INFO(
    this->get_logger(), "UPDATING NEW_COLOR TO %s | COLOR INT: %i ", color.c_str(), curr_color_int);

  this->new_color_pub->publish(this->color_map[color]);
}

}  // namespace led_controller

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(led_controller::button_pub)
