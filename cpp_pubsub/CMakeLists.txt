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
find_package(cpp_pubsub REQUIRED) #For custom msg & srv

# To convert the interfaces you defined into language-specific code 
# (like C++ and Python) so that they can be used in those languages
rosidl_generate_interfaces(${PROJECT_NAME}  
 "msg/Num.msg"
 "srv/AddThreeInts.srv"
)

# add the executable and name it talker so you can RUN your node using ros2 run
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# New nodes for custom msg
add_executable(talker_custom_msg src/publisher_custom_msg.cpp)
ament_target_dependencies(talker_custom_msg rclcpp std_msgs cpp_pubsub)

add_executable(listener_custom_msg src/subscriber_custom_msg.cpp)
ament_target_dependencies(listener_custom_msg rclcpp std_msgs cpp_pubsub)

# New nodes for custom srv (add two ints)
add_executable(server src/server_custom_srv.cpp)
ament_target_dependencies(server rclcpp cpp_pubsub)   

add_executable(client src/client_custom_srv.cpp)
ament_target_dependencies(client rclcpp cpp_pubsub)                      

# add the install(TARGETS…) section so ros2 run can FIND your executable
install(TARGETS
  talker
  listener

  # New nodes for custom msg
  talker_custom_msg
  listener_custom_msg

  # New nodes for custom srv
  server
  client

  DESTINATION lib/${PROJECT_NAME})

  

ament_package()
