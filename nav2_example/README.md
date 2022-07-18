# Nav2 Reverse Engineering Process (ROS 2)
* Read this in conjunction with `nav2_master_launch.py`
1. Create ROS 2 package: `ros2 pkg create --build-type ament_cmake <package_name> # Create a ros2 package`

## ROS Launch File
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

## (ROS LAUNCH) COMPONENT: Robot State Publisher
4. (Arbitrary Order) Copy & Paste `urdf` folder from `nav2_bringup` package. Create path to the folder:
    In `CMakeList.txt`:
    ```sh
    # Add
    install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
    )
    ```
    -> *Import the path* in launch file (**step N.1**).

5. (Arbitrary Order) Copy & Paste `maps` folder from `nav2_bringup` package. Create path to the folder:
    In `CMakeList.txt`:
    ```sh
    # Add
    install(
    DIRECTORY maps
    DESTINATION share/${PROJECT_NAME}
    )
    ```

6. (Arbitrary Order) Copy & Paste `worlds` folder from `nav2_bringup` package. Create path to the folder:
    In `CMakeList.txt`:
    ```sh
    # Add
    install(
    DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME}
    )
    ```