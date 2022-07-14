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
* Basic pub & sub example:
  ```sh
  ros2 launch ~/ros2_ws/src/ros2-examples/cpp_pubsub/launch/cpp_pubsub.launch.py
  # OR ros2 launch ~/ros2_ws/src/cpp_pubsub/launch/cpp_pubsub.launch.py
  ```
