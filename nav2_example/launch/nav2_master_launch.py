"""
* In order to fully understand, the file needs to be inspected in various VIEWS. 
* This is written from perspective of a person who just skimmed through ROS Tutorials and wanna jump right in/shorten learning time. Therefore, 
a lot of assumptions will be made and a lot of them will be wrong.

********************************************************************************************************************************************
********************************************************************************************************************************************
********************************************************************************************************************************************

VIEW 1: WATERFALL VIEW
* This VIEW describe the launch file structure as a whole

    STEP 0:                 Import MODULES/LIBRARIES/Functions

    STEP 1 (OPTIONAL):      Get DIRECTORIES to LAUNCH FOLDERS of relevant packages (to launch child launch files)
                            * This is OPTIONAL because the values can be assigned directly in `IncludeLaunchDescription( PythonLaunchDescriptionSource ())`

    STEP 2 (OPTIONAL):      Create LAUNCH CONFIGURATION VARIABLES
                            * This is OPTIONAL because values can be assigned directly
        STEP 2.1 (OPTIONAL):    Declare ARGUMENTS, linked to STEP 2.
    
    STEP 3:                 Execute Command Line by ExecuteProcess() (To initiate Gazebo)

    STEP 4:                 Launch ROS NODES

    STEP 5:                 Run Child ROS Launch 

    STEP 6:                 Create the LAUNCH DESCRIPTION and populate


********************************************************************************************************************************************
********************************************************************************************************************************************
********************************************************************************************************************************************

VIEW 2: UNIT VIEW
* This VIEW describe how to construct ONE completed UNIT in the launch file

* COMPONENT 1: ROS NODE `robot_state_publisher`

    STEP 0:                 Import MODULES/LIBRARIES
    
    STEP 1 (OPTIONAL):      Create LAUNCH CONFIGURATION VARIABLES: 
                            

                            'namespace' with (1) namespace = LaunchConfiguration('namespace') 
                                        and (2) declare_namespace_cmd = DeclareLaunchArgument(...)
                            'use_sim_time' with (1) use_sim_time = LaunchConfiguration('use_sim_time') 
                                           and (2) declare_use_sim_time_cmd = DeclareLaunchArgument()
                            'use_robot_state_pub' ...
                            remappings
                        
                            * This is OPTIONAL because the values can be assigned directly in `IncludeLaunchDescription( PythonLaunchDescriptionSource ())`
    
    STEP 2:                 Use Node: start_robot_state_publisher_cmd = Node()

    STEP 3:                 ADD everything to LAUNCH DESCRIPTION

                                return LaunchDescription([
                                    declare_namespace_cmd,
                                    declare_use_sim_time_cmd,
                                    declare_use_robot_state_pub_cmd,
                                    start_robot_state_publisher_cmd])

                            And RETURN the LAUNCH DESCRIPTION
            


* COMPONENT 2: GAZEBO Simulation

    STEP 0:                 ExecuteProcess, IfCondition, PythonExpression                   (Import MODULES/LIBRARIES/Functions)

    STEP 1 (OPTIONAL):      'use_simulator', 'world', 'headless'                            (LAUNCH CONFIGURATION VARIABLES)

    STEP 2:                 EXECUTE CMD: start_gazebo_server_cmd = ExecuteProcess(), start_gazebo_client_cmd = ExecuteProcess()

    STEP 3:                 ADD everything to LAUNCH DESCRIPTION & RETURN the LAUNCH DESCRIPTION



* COMPONENT 3: Rviz

    STEP 0:                 IncludeLaunchDescription, PythonLaunchDescriptionSource         (Import MODULES/LIBRARIES/Functions)

    STEP 1 (OPTIONAL):      'rviz_config_file', 'use_rviz'                                  (LAUNCH CONFIGURATION VARIABLES)

    STEP 2:                 Run child ROS Launch file: rviz_cmd = IncludeLaunchDescription(PythonLaunchDescriptionSource())

    STEP 3:                 ADD everything to LAUNCH DESCRIPTION & RETURN the LAUNCH DESCRIPTION

* COMPONENT 4: Navigation 2 ...
* For Hierarchy & Functionalities of the Components/Files, go to the end.

"""

# ----------------------------------------------------------- #
# ------------- STEP 0: IMPORT MODULES/LIBRARIES ------------ #
# ----------------------------------------------------------- #

"""
os - Miscellaneous operating system interfaces LINK: https://docs.python.org/3/library/os.html
This module provides a portable way of using operating system dependent functionality.
+ If you just want to read or write a file see open().
+ If you want to manipulate paths, see the os.path module, ...
"""
import os

"""
get_package_share_directory() - as the name suggests, RETURN path of the package. 
"""
from ament_index_python.packages import get_package_share_directory 

"""
LINK: https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Using-Substitutions.html

DeclareLaunchArgument() is used to define the launch argument that can be passed from the above launch file or from the console.
LaunchConfiguration() substitutions allow us to acquire the value of the launch argument in any part of the launch description.
ExecuteProcess() is used to execute command line
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # ----------------------------------------------------------- #
    # ------ STEP 1 (OPTIONAL): Get the LAUNCH DIRECTORIES ------ #
    # ----------------------------------------------------------- #

    nav2_example_dir = get_package_share_directory('nav2_example') # Get path of `nav2_example` package, then ...
    nav2_example_launch_dir = os.path.join(nav2_example_dir, 'launch') # ... Join the path with folder `launch`

    # nav2_example_launch_dir = ~/.../nav2_example/launch
    # Remember to create PATH to `launch` folder (in nav2_example) in CMakeLists.txt

    nav2_bt_navigator_dir = get_package_share_directory('nav2_bt_navigator')

    # Can add more LAUNCH FOLDERS of OTHER PACKAGES, uncomment the belows
    # package_1_dir = get_package_share_directory('package_1') 
    # package_1_launch_dir = os.path.join(package_1_dir, 'launch') 

    # ----------------------------------------------------------- #
    # ---- STEP 2 (OPTIONAL): LAUNCH CONFIGURATION VARIABLES ---- #
    # ----------------------------------------------------------- #

    # Create the LAUNCH CONFIGURATION VARIABLES 

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time') 
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    # Create the LAUNCH CONFIGURATION VARIABLES specific to SIMULATION
   
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_simulator = LaunchConfiguration('use_simulator') # For `start_gazebo_server_cmd`, start Gazebo with ExecuteProcess()
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # ----------------------------------------------------------- #
    # ------------- STEP 2.1 (OPTIONAL): ARGUMENTS -------------- #
    # ----------------------------------------------------------- #

    # Declare the LAUNCH ARGUMENTS

    # Linked with: namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    # Linked with: use_sim_time = LaunchConfiguration('use_sim_time') 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(nav2_example_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_example_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            nav2_bt_navigator_dir,
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    # Linked with: use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    
    # Linked with: use_simulator = LaunchConfiguration('use_simulator')
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(nav2_example_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        #                            'worlds/turtlebot3_worlds/waffle.model'),
        default_value=os.path.join(nav2_example_dir, 'worlds', 'waffle.model'),
        description='Full path to world model file to load')
    
    # ----------------------------------------------------------- #
    # ------------------ STEP 3: EXECUTE CMD -------------------- #
    # ----------------------------------------------------------- #

    ################################################################
                    # COMPONENT 2: GAZEBO SIMULATION
    ################################################################

    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', world],
        cwd=[nav2_example_launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        cmd=['gzclient'],
        cwd=[nav2_example_launch_dir], output='screen')

    # ----------------------------------------------------------- #
    # -------------------- STEP 4: ROS NODES -------------------- #
    # ----------------------------------------------------------- #
    
    ################################################################
            # COMPONENT 1: ROBOT STATE PUBLISHER (ROS NODE)
    ################################################################

    urdf = os.path.join(nav2_example_dir, 'urdf', 'turtlebot3_waffle.urdf') 
    # urdf = ~/.../nav2_example/urdf/turtlebot3_waffle.urdf
    # Remember to create PATH to `urdf` folder (in nav2_example) in CMakeLists.txt

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[urdf])


    # ----------------------------------------------------------- #
    # -------------- STEP 5: ROS LAUNCH FILES ------------------- #
    # ----------------------------------------------------------- #

    ################################################################
                        # COMPONENT 3: Rviz
    ################################################################

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_example_launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())
    
    ################################################################
                    # COMPONENT 4: Navigation 2
    ################################################################

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_example_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    # ----------------------------------------------------------- #
    # --------- STEP 6: Create the LAUNCH DESCRIPTION ------------ #
    # ----------------------------------------------------------- #

    return LaunchDescription([

        # Declare the launch OPTIONS
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        declare_bt_xml_cmd,
        declare_autostart_cmd,
        
        declare_use_robot_state_pub_cmd,
        declare_use_simulator_cmd,
        declare_rviz_config_file_cmd,
        declare_use_rviz_cmd,
        declare_simulator_cmd,
        declare_world_cmd,
        
        # Execute Command Line (For Gazebo Simulation)
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,

        # launch ROS NODES
        start_robot_state_publisher_cmd,
        
        # Run ROS Launch files
        rviz_cmd,
        bringup_cmd,
        ])

"""

# ----------------------------------------------------------- #
# -------------------- FILE HIERARCHY------------------------ #
# ----------------------------------------------------------- #

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
"""