# ROS 2 Examples
My ROS 2 Examples

## Tutorials

### Resources
* [ROS vs. ROS 2](https://roboticsbackend.com/ros1-vs-ros2-practical-overview/)
* [ROS Foxy (Ubuntu 20.04) Tutorials](https://docs.ros.org/en/foxy/index.html)

### Installation
* [ROS Foxy Installation (Debian)](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
* Install Colcon: `sudo apt install python3-colcon-common-extensions`
* Install RTI Connext DDS: `sudo apt-get install ros-foxy-rmw-connext-cpp`


### Make a ROS 2 Workspace
* Create the workspace
  ```sh
  mkdir -p ~/ros2_ws/src/
  cd ros2_ws
  colcon build
  ```
### cpp_pubsub package
* Testes successfully on ROS Foxy (Ubuntu 20.04), failed on ROS Crystal (Ubuntu 18.04)
* Clone & build the package:
  ```sh
  cd ~/ros2_ws/src/
  git clone https://github.com/ChuongNg1996/ros2-examples
  cd ..
  colcon build
  ```
* Basic ***pub & sub*** example:
  ```sh
  ros2 launch cpp_pubsub cpp_pubsub.launch.py

  ```
  For further info, examine `publisher_member_function.cpp` & `subscriber_member_function.cpp`
 
  
* Basic pub & sub example with ***custom message***:
  ```sh
  ros2 launch cpp_pubsub cpp_pubsub_custom_msg.launch.py
  ```
  For further info, examine `publisher_custom_msg.cpp` & `subscriber_custom_msg.cpp`
  
  
* Basic ***service & client*** example:
  ```sh
  # Terminal 1
  ros2 run cpp_pubsub server 
  # Terminal 2
  ros2 run cpp_pubsub client 2 3 1
  ```
  For further info, examine `server_custom_srv.cpp` & `client_custom_srv.cpp`
  
  
* Another example with ***custom message***:
  ```sh
  # Terminal 1
  ros2 run cpp_pubsub publish_address_book
  # Terminal 2
  ros2 topic echo /address_book
  ```
  For further info, examine `publish_address_book.cpp`

* Example with parameters in C++ Class:
  ```sh
  ros2 launch cpp_pubsub cpp_pubsub_param.py
  ```
  For further info, examine `cpp_parameters_node.cpp`
