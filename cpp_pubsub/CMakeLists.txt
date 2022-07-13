cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# To convert the interfaces you defined into language-specific code 
# (like C++ and Python) so that they can be used in those languages
rosidl_generate_interfaces(${PROJECT_NAME}  
 "msg/Num.msg"
)

# add the executable and name it talker so you can RUN your node using ros2 run
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# add the install(TARGETS…) section so ros2 run can FIND your executable
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

  

ament_package()
