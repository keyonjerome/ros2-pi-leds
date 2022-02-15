# ROS2 LED Strip Controller
This project is a ROS2 LED Strip controller written in C++ using pigpio with button input for switching colors, running with `pigpio` on a Raspberry Pi Model 3B+. 

## Design
- The `led_strip_output` node defines a subscriber for `"led_controller1/new_color"`, a `std_msgs::msg::ColorRGBA` object. Upon receipt of a new color by any publisher, the node will set the LED strip to that color.
- The `button_pub` node defines a publisher for `"led_controller1/new_color"` according to GPIO input from a pushbutton. Depending on which color is next in the queue, a button press will immediately publish a new color. The color rotation goes as follows: `White → Red → Blue → Purple → Medium Slate Blue → Off → White`. New colors can be added in the `button_pub.cpp` file in the node's constructor.

## Equipment
- 24V LED strip and power supply with RGB input
- 3 logic-level mosfets for LED strip control
- Raspberry Pi 3B+ running Ubuntu Server 20.04 ([Ubuntu is recommended for ROS2 installation](https://ubuntu.com/raspberry-pi))
- Pushbutton

## Hardware Setup
`pigpio` uses Broadcom numbers for addressing the Raspberry Pi's GPIO outputs. The GPIO arrangement diagram for each Raspberry Pi distribution is available [here](https://abyz.me.uk/rpi/pigpio/).
The following GPIO constants can be changed in their corresponding header files.
- LED Strip:
    - Red: GPIO 2
    - Green: GPIO 3
    - Blue: GPIO 4
- Pushbutton: GPIO 17

## Requirements
- [ROS2 Galactic](https://docs.ros.org/en/galactic/index.html)
- [pigpio](https://abyz.me.uk/rpi/pigpio/)

## Installation and How To Run
1. Make sure ROS2 Galactic and pigpio are installed onto your Raspberry Pi. `pigpio-master` should be in your `home/user` directory (`~/`), as the CMakeLists.txt configuration searches for it there.
2. Clone this repository and after sourcing your ROS2 installation (`source /opt/ros/galactic/setup.bash`), run `colcon build` in the repository folder.
3. Open a new terminal.
4. Enter your root shell using `sudo -i`. This is required for `pigpio` GPIO setting.
5. Navigate to the repository directory in the new shell.
6. In a new terminal, install the newly built package using `. install/setup.bash` from within the repository folder.
7. Before running the program, kill all currently running `pigpio` processes: `killall pigpiod`
8. Run the program with `ros2 launch led_strip_controller led_launch.py`.