#include <chrono>
#include <led_strip_controller/button_pub.hpp>

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

    button_pub::button_pub(const rclcpp::NodeOptions &options) :Node("button_pub", options) {

        this->new_color_pub = this->create_publisher<std_msgs::msg::ColorRGBA>(
            "led_controller1/new_color", rclcpp::QoS(QUEUE));
       
       this->curr_color_sub = this->create_subscription<std_msgs::msg::ColorRGBA>(
      "led_controller1/current_color", 10, std::bind(&button_pub::curr_color_sub_callback, this, _1));
        this->color_pub_timer = this->create_wall_timer(
      500ms, std::bind(&button_pub::update_new_color, this));
        
        this->color_map["red"] = new_color(255,0,0,0);
        this->color_map["blue"] = new_color(0,0,255,0);

        // define color consts
        // this->color_map["red"] = std_msgs::msg::ColorRGBA();
        // this->color_map["red"].r = 255;
        // this->color_map["red"].g = 0;
        // this->color_map["red"].b = 0;


        // this->color_map["blue"] = std_msgs::msg::ColorRGBA();
        // this->color_map["blue"].r = 0;
        // this->color_map["blue"].g = 0;
        // this->color_map["blue"].b = 255;



    }
    

    bool button_pub::get_button_status() {

      // placeholder
      return true;

    }

    void button_pub::update_new_color() {
        
        get_button_status();
    
        this->new_color_pub->publish(this->color_map["blue"]);


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
    




    