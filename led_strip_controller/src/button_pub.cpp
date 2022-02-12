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
        

        gpioSetPullUpDown(BUTTON_GPIO, PI_PUD_UP); 

        this->new_color_pub = this->create_publisher<std_msgs::msg::ColorRGBA>(
            "led_controller1/new_color", rclcpp::QoS(QUEUE));
       
       this->curr_color_sub = this->create_subscription<std_msgs::msg::ColorRGBA>(
      "led_controller1/current_color", 10, std::bind(&button_pub::curr_color_sub_callback, this, _1));
    //     this->button_stat_sub =  this->create_subscription<std_msgs::msg::Bool>(
    //   "led_controller1/current_color", 10, std::bind(&button_pub::button_stat_sub_callback, this, _1));

        this->button_pub_timer = this->create_wall_timer(
      10ms, std::bind(&button_pub::button_status_pub, this));

    // this->color_pub_timer = this->create_wall_timer(
    //   500ms, std::bind(&button_pub::update_new_color, this));
        
        // It would be better to create a custom msg and use a multi-index map for this,
        // but in the interest of time this is alright.
        std::string white = "white";
        std::string red = "red"; 
        std::string blue = "blue";
        std::string purple = "purple";
        std::string medium_slate_blue = "medium_slate_blue";
        std::string off = "off";

        this->color_map[white] = new_color(255,255,255,0);
        this->color_map[red] = new_color(255,0,0,0);
        this->color_map[blue] = new_color(0,0,255,0);
        this->color_map[purple] = new_color(255,0,255,0);
        this->color_map[medium_slate_blue] = new_color(106,90,205,0);
        this->color_map[off] = new_color(0,0,0,0);

        // using two maps instead of a multi-index map, because multi-index maps would greatly increase code bloat
        // this defines the ordering of the colors when the button is pressed
        this->color_int_map[0] = white;
        this->color_int_map[1] = red;
        this->color_int_map[2] = blue;
        this->color_int_map[3] = purple;
        this->color_int_map[4] = medium_slate_blue;
        this->color_int_map[5] = off;

        // std::thread button_thread(std::bind(&button_pub::button_loop,this));
    }

    void button_pub::button_status_pub() {
        bool reading = get_button_status();
        RCLCPP_INFO(this->get_logger(),"BUTTON: %s ", reading ? "TRUE" : "FALSE");

            if(last_reading != reading) {
                last_debounce_time = time(nullptr)*1000;
                RCLCPP_INFO(this->get_logger(),"DEBOUNCE TIME UPDATE: %li", last_debounce_time);
                published = false;
            }
            RCLCPP_INFO(this->get_logger(),"TIME DIFF %li", (time(nullptr)*1000 - last_debounce_time));
            //if the button value has not changed for the debounce delay, we know its stable
            if ( !published && (time(nullptr)*1000 - last_debounce_time)  > debounce_delay && reading) {
                RCLCPP_INFO(this->get_logger(), "UPDATING COLOR FROM BUTTON PRESS");
                update_new_color();
                published = true;
            }

            last_reading = reading;
    }

    bool button_pub::get_button_status() {

      return !(gpioRead(BUTTON_GPIO) == 1);

    }

    void button_pub::update_new_color() {
    
        this->curr_color_int++;
        // reset to first color if at end of color wheel
        if(this->curr_color_int == this->color_int_map.size()) this->curr_color_int = 0;
        std::string color = this->color_int_map[this->curr_color_int];
        RCLCPP_INFO(this->get_logger(),"UPDATING NEW_COLOR TO %s | COLOR INT: %i ", color.c_str(),curr_color_int);
        this->new_color_pub->publish(this->color_map[color]);


    }

        // called whenever a new value is published to "led_controller1/current_color"
    // void button_pub::button_stat_sub_callback(const std_msgs::msg::Bool msg) {
        
    //     curr_color->r = msg.r;
    //     curr_color->g = msg.g;
    //     curr_color->b = msg.b;
    //     curr_color->a = msg.a;

    //     RCLCPP_INFO(this->get_logger(), "curr-color %f, %f, %f, %f",
    //         curr_color->r,
    //         curr_color->g,
    //         curr_color->b,
    //         curr_color->a
    //     );

    // }
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
    




    