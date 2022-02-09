#include <chrono>
#include <led_strip_controller/led_strip_output.hpp>
#include <pigpio.h>

using namespace std::literals::chrono_literals;

using namespace std::placeholders;

namespace led_controller
{
    // We publish current color, but we set based on new color. 

    led_strip_output::led_strip_output(const rclcpp::NodeOptions &options) :Node("led_output", options) {
        
        if(gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(),"GPIO INITIALIZE FAIL");
        }
        else {
            RCLCPP_INFO(this->get_logger(),"GPIO INITIALIZE SUCCESS");

        }
        
        
        RCLCPP_INFO(this->get_logger(),"RUN NODE A");

        this->curr_color_pub = this->create_publisher<std_msgs::msg::ColorRGBA>(
            "led_controller1/current_color", rclcpp::QoS(QUEUE));


        this->timer = this->create_wall_timer(500ms,std::bind(&led_strip_output::curr_color_pub_callback,this));   

        this->new_color_sub = this->create_subscription<std_msgs::msg::ColorRGBA>(
      "led_controller1/new_color", 10, std::bind(&led_strip_output::new_color_sub_callback, this, _1));


        RCLCPP_INFO(this->get_logger(),"RUN NODE B");
    }

    std_msgs::msg::ColorRGBA led_strip_output::get_strip_color() {
        
        // placeholder
        std_msgs::msg::ColorRGBA curr = std_msgs::msg::ColorRGBA();
        curr.r = 255;
        curr.g = 254;
        curr.b = 253;
        curr.a = 0;

        return curr;
        
    }

    void led_strip_output::new_color_sub_callback(const std_msgs::msg::ColorRGBA msg) {
        
        set_strip_color(msg);
    }

    void led_strip_output::set_strip_color(const std_msgs::msg::ColorRGBA color) {
        
        
        if(gpioPWM(RED_GPIO,static_cast<int>(color.r)) != 0) {
            RCLCPP_ERROR(this->get_logger(),"SET_STRIP_COLOR ERROR: PI_BAD_USER_GPIO or PI_BAD_DUTYCYCLE WITH GPIO ADDRESS %i",RED_GPIO);
        }
        if(gpioPWM(GREEN_GPIO,static_cast<int>(color.g)) != 0) {
            RCLCPP_ERROR(this->get_logger(),"SET_STRIP_COLOR ERROR: PI_BAD_USER_GPIO or PI_BAD_DUTYCYCLE WITH GPIO ADDRESS %i",GREEN_GPIO);
        }
        if(gpioPWM(BLUE_GPIO,static_cast<int>(color.b)) != 0) {
            RCLCPP_ERROR(this->get_logger(),"SET_STRIP_COLOR ERROR: PI_BAD_USER_GPIO or PI_BAD_DUTYCYCLE WITH GPIO ADDRESS %i",BLUE_GPIO);
        }

    }

    
    void led_strip_output::update_curr_color() {
        auto curr_color = get_strip_color();
        
        curr_color_msg->r = curr_color.r;
        curr_color_msg->g = curr_color.g;
        curr_color_msg->b = curr_color.b;
        curr_color_msg->a = curr_color.a;
    }

    void led_strip_output::curr_color_pub_callback() {
        
        // auto curr_color_msg = std::make_unique<std_msgs::msg::ColorRGBA>();

        RCLCPP_INFO(this->get_logger(), "publishing %f, %f, %f, %f",
            curr_color_msg->r,
            curr_color_msg->g,
            curr_color_msg->b,
            curr_color_msg->a
        );

        this->curr_color_pub->publish(*curr_color_msg);


    }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(led_controller::led_strip_output)
    




    