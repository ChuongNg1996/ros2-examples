# LINK: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Substitutions.html#id6

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    colors = {
        'background_r': '200'
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    # "FindPackageShare" substitution is used to find the path to the "basic_ex" package. 
                    # The "PathJoinSubstitution" substitution is then used to join the path to that package 
                    # path with the "example_substitutions.launch.py" file name
                    FindPackageShare('basic_ex'),
                    'launch',
                    'example_substitutions.launch.py'
                ])
            ]),
            # The "launch_arguments" dictionary with "turtlesim_ns" and "use_provided_red" arguments is passed to 
            # the "IncludeLaunchDescription" action. 
            launch_arguments={
                'turtlesim_ns': 'turtlesim2',
                'use_provided_red': 'True',
                # The TextSubstitution substitution is used to define the new_background_r argument with the value 
                # of the background_r key in the colors dictionary.
                'new_background_r': TextSubstitution(text=str(colors['background_r']))
            }.items()
        )
    ])