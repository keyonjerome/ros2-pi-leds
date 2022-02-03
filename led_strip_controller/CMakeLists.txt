cmake_minimum_required(VERSION 3.8)
project(led_strip_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

#include the 'include' directory
include_directories(include)

#create resource which references the libraries in the binary bin
set(node_plugins "")

#add led_output as plugin
add_library(led_output SHARED
	src/led_strip_output.cpp)
target_compile_definitions(led_output
	PRIVATE "LED_CONTROLLER_DLL")
ament_target_dependencies(led_output
	"rclcpp"
	"rclcpp_components"
	"std_msgs")
# rosidl_target_interfaces(led_output ${PROJECT_NAME} "rosidl_typesupport_cpp") # need this for custom messages - idk why tho
rclcpp_components_register_node(led_output PLUGIN "led_controller::led_strip_output"  EXECUTABLE led_outputter)



#tell where to put binaries
install(TARGETS

  led_output

	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin)

#install launch file
install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
	)

#install configuration file (yaml files) for param server
install(DIRECTORY
	config
	DESTINATION share/${PROJECT_NAME}
	)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
