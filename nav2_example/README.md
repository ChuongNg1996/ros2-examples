# Nav2 Reverse Engineering Process (ROS 2)

## Installation
   ```sh
   # <distro> = foxy in this example
   sudo apt install ros-<ros2-distro>-navigation2 
   sudo apt install ros-<ros2-distro>-nav2-bringup
   sudo apt install ros-<ros2-distro>-turtlebot3*
   ```
## Reverse Engineering Process

* Read this in conjunction with `nav2_master_launch.py`
1. Create ROS 2 package: `ros2 pkg create --build-type ament_cmake <package_name> # Create a ros2 package`

### ROS Launch File
2. Make `launch` folder in the package and modify
    In `CMakeList.txt`:
    ```sh
    # Add
    install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    )
    ```
    In `package.xml`:
    ```sh
    # Add
    <exec_depend>launch</exec_depend>
    <exec_depend>launch_ros</exec_depend>
    ```

    To install the path
3. Start with a new `ROS launch` file named `nav2_master_launch.py` which is based on master launch file of `nav2_bringup` package.

### (ROS LAUNCH) COMPONENT #1: Robot State Publisher
4. (Arbitrary Order) Copy & Paste `urdf` folder from `nav2_bringup` package. Create path to the folder:
    In `CMakeList.txt`:
    ```sh
    # Add
    install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
    )
    ```
    Then see *VIEW 2: UNIT VIEW* -> *COMPONENT 1: ROS NODE `robot_state_publisher`* for construction in `nav2_master_launch.py` 
    
### (ROS LAUNCH) COMPONENT #2: Gazebo Environment
5. (Arbitrary Order) Copy & Paste `maps` folder from `nav2_bringup` package. Create path to the folder:
    In `CMakeList.txt`:
    ```sh
    # Add
    install(
    DIRECTORY maps
    DESTINATION share/${PROJECT_NAME}
    )
    ```
    Then see *VIEW 2: UNIT VIEW* -> *COMPONENT 2: GAZEBO Simulation* for construction in `nav2_master_launch.py` 

6. (Arbitrary Order) Copy & Paste `worlds` folder from `nav2_bringup` package. Create path to the folder:
    In `CMakeList.txt`:
    ```sh
    # Add
    install(
    DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME}
    )
    ```
    In `visual` tag of both `wheel_left_link` and `wheel_right_link`, `<uri>model://turtlebot3_waffle/meshes/right_tire.dae</uri>` is not found -> caused crashed -> change to same cylinder as `collision` tage.

## (ROS LAUNCH) COMPONENT #3: Rviz
7. (Arbitrary Order) Copy & Paste `rviz` folder from `nav2_bringup` package. Create path to the folder:
    In `CMakeList.txt`:
    ```sh
    # Add
    install(
    DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
    )
    ```
8.  Copy & Paste `rviz_launch.py` from `nav2_bringup` package to `launch` folder of `nav2_example`. Adjust value of `bring_up` from `nav2_bringup` to `nav2_example` 
    Then see *VIEW 2: UNIT VIEW* -> *COMPONENT 3: Rviz* for construction in `nav2_master_launch.py` 

### (ROS LAUNCH) COMPONENT #4: Navigation 2
9. (Arbitrary Order) Copy & Paste `param` folder from `nav2_bringup` package. Create path to the folder:
    In `CMakeList.txt`:
    ```sh
    # Add
    install(
    DIRECTORY params
    DESTINATION share/${PROJECT_NAME}
    )
    ```
10. Copy & Paste `bringup_launch.py`, `localization_launch.py`, `navigation_launch.py`, `slam_launch.py` from `nav2_bringup` package to `launch` folder of `nav2_example`. Adjust value of `bring_up` from `nav2_bringup` to `nav2_example` 
    Then see *VIEW 2: UNIT VIEW* -> *COMPONENT 4: Navigation 2* for construction in `nav2_master_launch.py` 

### FILE HIERARCHY
   ```sh
    [nav2_master_launch.py]

    + ['robot_state_publisher'] ROS Node: ROBOT STATE PUBLISHER 
        ++ /urdf

    + Execute cmd: GAZEBO Simulation    
        ++ /worlds

    + [rviz_launch.py] ROS Launch: Rviz
        ++ ['rviz2'] ROS Node: Start Rviz
        ++ /rviz/nav2_default_view.rviz
    
    + [bringup_launch.py] ROS Launch: Navigation 2
        ++ [slam_launch.py] ROS Launch: SLAM
            +++ /params/nav2_params.yaml
            +++ ['slam_toolbox'] ROS Package
            +++ ['map_saver_server'] ROS Node -> To save Map
            +++ ['lifecycle_manager'] ROS Node: 'lifecycle_manager_slam'
                ++++ lifecycle_nodes = ['map_saver'] 

        ++ [localization_launch.py] ROS Launch: Localization
            +++ /maps/turtlebot3_world.yaml
            +++ /params/nav2_params.yaml
            +++ ['map_server'] ROS Node -> Access static Map
            +++ ['amcl'] ROS Node -> AMCL Localization Method
            +++ ['lifecycle_manager'] ROS Node: 'lifecycle_manager_localization'
                ++++ lifecycle_nodes = ['map_server', 'amcl']

        ++ [navigation_launch.py] ROS Launch: Navigation
            +++ /params/nav2_params.yaml
            +++ ['nav2_bt_navigator'] ROS Package: Behavivour Tree
            +++ ['controller_server'] ROS Node: Local Planner
            +++ ['planner_server'] ROS Node: Global Planner
            +++ ['recoveries_server'] ROS Node: Recovery Behaviour
            +++ ['bt_navigator'] ROS Node: Behaviour Tree
            +++ ['waypoint_follower'] ROS Node: Waypoint Follower
            +++ ['lifecycle_manager'] ROS Node: 'lifecycle_manager_navigation'
                ++++ lifecycle_nodes = ['controller_server', 'planner_server', 'recoveries_server', 'bt_navigator', 'waypoint_follower']

        ++ ['nav2_bt_navigator'] ROS Package: Behaviour Tree
    
    + ['nav2_bt_navigator'] ROS Package: Behaviour Tree
   ```

## Run the example
   ```sh
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/<ros2-distro>/share/turtlebot3_gazebo/models # <ros2-distro> = foxy in this example
    ros2 launch nav2_example nav2_master_launch.py 
   ```
