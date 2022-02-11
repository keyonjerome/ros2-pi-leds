#include <chrono>
#include <led_strip_controller/button_pub.hpp>
#include <pigpio.h>
#include <sys/time.h>
#include <ctime>

using namespace std::literals::chrono_literals;

using namespace std::placeholders;

namespace led_controller
{

    std_msgs::msg::ColorRGBA button_pub::new_color(float r,float g, float b, float a) {
        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }
    // bool operator == (std_msgs::msg::ColorRGBA & lhs, std_msgs::msg::ColorRGBA, std_msgs::msg::ColorRGBA & rhs) {
    //     // we don't use the A value 
    //     return lhs.r == rhs.r && lhs.g == rhs.g && && lhs.b == rhs.b;
    // }

    button_pub::button_pub(const rclcpp::NodeOptions &options) :Node("button_pub", options) {

        this->new_color_pub = this->create_publisher<std_msgs::msg::ColorRGBA>(
            "led_controller1/new_color", rclcpp::QoS(QUEUE));
       
       this->curr_color_sub = this->create_subscription<std_msgs::msg::ColorRGBA>(
      "led_controller1/current_color", 10, std::bind(&button_pub::curr_color_sub_callback, this, _1));
        this->color_pub_timer = this->create_wall_timer(
      500ms, std::bind(&button_pub::update_new_color, this));
        
        std::string red = "red"; 
        std::string blue = "blue";
        std::string medium_slate_blue = "medium_slate_blue";

        this->color_map[red] = new_color(255,0,0,0);
        this->color_map[blue] = new_color(0,0,255,0);
        this->color_map[medium_slate_blue] = new_color(106,90,205,0);

        // using two maps instead of a multi-index map, because multi-index maps would greatly increase code bloat
        // this defines the ordering of the colors when the button is pressed
        this->color_int_map[0] = red;
        this->color_int_map[1] = blue;
        this->color_int_map[2] = medium_slate_blue;

        // std::thread button_thread(&button_pub::button_loop);
    }

    void button_pub::button_loop() {
        
        while(true) {

            bool reading = get_button_status();
            
            if(last_reading != reading) {
                last_debounce_time = time(nullptr)*1000;
                published = false;
            }

            //if the button value has not changed for the debounce delay, we know its stable
            if ( !published && (time(nullptr)*1000 - last_debounce_time)  > debounce_delay) {

                update_new_color();
                published = true;
            }

            last_reading = reading;
        }

    }
    

    bool button_pub::get_button_status() {

      return !(gpioRead(BUTTON_GPIO) == 1);

    }

    void button_pub::update_new_color() {
    
        this->curr_color_int++;
        // reset to first color if at end of color wheel
        if(this->curr_color_int == this->color_int_map.size()) this->curr_color_int = 0;
        std::string color = this->color_int_map[this->curr_color_int];
        this->new_color_pub->publish(this->color_map[color]);


    }

    // called whenever a new value is published to "led_controller1/current_color"
    void button_pub::curr_color_sub_callback(const std_msgs::msg::ColorRGBA msg) {
         
        curr_color->r = msg.r;
        curr_color->g = msg.g;
        curr_color->b = msg.b;
        curr_color->a = msg.a;

        RCLCPP_INFO(this->get_logger(), "curr-color %f, %f, %f, %f",
            curr_color->r,
            curr_color->g,
            curr_color->b,
            curr_color->a
        );

    }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(led_controller::button_pub)
    




    