from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
       Node(
         package='basic_ex',
         namespace='ns1', # Make sure this matches the subscriber's namespace
         executable='talker_custom_msg', # Name of the executable
         name='minimal_publisher' # Any name is fine
       ),
       Node(
         package='basic_ex',
         namespace='ns1', # Make sure this matches the publisher's namespace
         executable='listener_custom_msg', # Name of the executable
         name='minimal_subscriber' # Any name is fine
       )
    ])
