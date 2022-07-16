# LINK: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Cpp.html

from launch import LaunchDescription
from launch_ros.actions import Node

# This tutorial aimed to show how StaticTransformBroadcaster can be used to publish static transforms. In 
# your real development process you shouldn’t have to write this code yourself and should use the dedicated 
# tf2_ros tool to do so. tf2_ros provides an executable named static_transform_publisher that can be used 
# either as a commandline tool or a node that you can add to your launchfiles.

def generate_launch_description():
   return LaunchDescription([
      Node(
            package='learning_tf2_cpp',
            executable='static_turtle_tf2_broadcaster',
            name='static_sample', # Any name is fine
            arguments = ['mystaticturtle', '0', '0', '1', '0', '0', '0']
            # child_frame_id x y z qx qy qz  
      ),
   ])