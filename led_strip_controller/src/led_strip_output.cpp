#include <chrono>
#include <led_strip_controller/led_strip_output.hpp>

using namespace std::literals::chrono_literals;

using namespace std::placeholders;

namespace led_controller
{


    led_strip_output::led_strip_output(const rclcpp::NodeOptions &options) :Node("led_output", options) {
        
        
        RCLCPP_INFO(this->get_logger(),"RUN NODE A");

        this->curr_colour_pub = this->create_publisher<std_msgs::msg::ColorRGBA>(
            "/led_controller1/current_colour", rclcpp::QoS(QUEUE));


        this->timer = this->create_wall_timer(10ms,std::bind(&led_strip_output::curr_color_pub_callback,this));        


        RCLCPP_INFO(this->get_logger(),"RUN NODE B");
    }

    std_msgs::msg::ColorRGBA led_strip_output::get_strip_color() {

        return std_msgs::msg::ColorRGBA();
        
    }
    void led_strip_output::curr_color_pub_callback() {
        
        auto curr_color_msg = std::make_unique<std_msgs::msg::ColorRGBA>();
        auto curr_color = get_strip_color();
        
        curr_color_msg->r = curr_color.r;
        curr_color_msg->g = curr_color.g;
        curr_color_msg->b = curr_color.b;
        curr_color_msg->a = curr_color.a;

        RCLCPP_INFO(this->get_logger(), "publishing %f, %f, %f, %f",
            curr_color_msg->r,
            curr_color_msg->g,
            curr_color_msg->b,
            curr_color_msg->a
        );

        this->curr_colour_pub->publish(std::move(curr_color_msg));


    }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(led_controller::led_strip_output)
    




    