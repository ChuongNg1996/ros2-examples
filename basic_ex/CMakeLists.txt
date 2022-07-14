cmake_minimum_required(VERSION 3.5)
project(basic_ex)

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
find_package(msg_srv_ex REQUIRED) #For custom msg & srv


# --------------------------------------------------- #
# -------------------- ROS NODES -------------------- #
# --------------------------------------------------- #

# May need to comment out all the node buildings and build the msg & srv first

# add the executable and name it talker so you can RUN your node using ros2 run
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# New nodes for custom msg
add_executable(talker_custom_msg src/publisher_custom_msg.cpp)
ament_target_dependencies(talker_custom_msg rclcpp std_msgs msg_srv_ex)

add_executable(listener_custom_msg src/subscriber_custom_msg.cpp)
ament_target_dependencies(listener_custom_msg rclcpp std_msgs msg_srv_ex)

add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp msg_srv_ex)

# In order to use the messages generated in the same package we need to use the following CMake code
#rosidl_target_interfaces(publish_address_book
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# New nodes for custom srv (add two ints)
add_executable(server src/server_custom_srv.cpp)
ament_target_dependencies(server rclcpp msg_srv_ex)   

add_executable(client src/client_custom_srv.cpp)
ament_target_dependencies(client rclcpp msg_srv_ex)    

# New nodes to test parameters
add_executable(parameter_node src/cpp_parameters_node.cpp)
ament_target_dependencies(parameter_node rclcpp)

# add the install(TARGETS…) section so ros2 run can FIND your executable
install(TARGETS
  talker
  listener

  # New nodes for custom msg
  talker_custom_msg
  listener_custom_msg
  publish_address_book

  # New nodes for custom srv
  server
  client

  # New nodes to test parameters
  parameter_node
  
  DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------- #
# -------------------- ROS LAUNCH ------------------- #
# --------------------------------------------------- #

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# --------------------------------------------------- #
# --------------------- HEADERS --------------------- #
# --------------------------------------------------- #

install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(
  include
)

# --------------------------------------------------- #
# ------------- ACTION SERVER & CLIENT -------------- #
# --------------------------------------------------- #

# Find additional packages
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# Action Server
add_library(action_server SHARED
  src/fibonacci_action_server.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_server
  "msg_srv_ex"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "action_tutorials_cpp::FibonacciActionServer" EXECUTABLE fibonacci_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# Action Client
add_library(action_client SHARED
  src/fibonacci_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "msg_srv_ex"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "action_tutorials_cpp::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
