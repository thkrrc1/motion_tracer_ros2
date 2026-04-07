from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tracer_pkg_name = 'motion_tracer_ros2'
    ld = LaunchDescription()

    initial_pose = LaunchConfiguration('initial_pose')

    initial_pose_arg = DeclareLaunchArgument('initial_pose', default_value='none')

    ld.add_action(initial_pose_arg)

    tracer_teleop_node = Node(
        package=tracer_pkg_name,
        executable='tracer_teleop_node',
        name='tracer_teleop_node',
        output='screen',
        parameters=[
            {'initial_pose': initial_pose},
        ],
    )
    ld.add_action(tracer_teleop_node)

    return ld