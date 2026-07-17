from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tracer_pkg_name = 'motion_tracer_ros2'
    ld = LaunchDescription()

    neck_movement = LaunchConfiguration('neck_movement')
    neck_offset = LaunchConfiguration('neck_offset')
    neck_reverse = LaunchConfiguration('neck_reverse')
    neck_auto = LaunchConfiguration('neck_auto')

    neck_movement_arg = DeclareLaunchArgument('neck_movement', default_value='increment')
    neck_offset_arg = DeclareLaunchArgument('neck_offset', default_value='0')
    neck_reverse_arg = DeclareLaunchArgument('neck_reverse', default_value='false')
    neck_auto_arg = DeclareLaunchArgument('neck_auto', default_value='false')

    ld.add_action(neck_movement_arg)
    ld.add_action(neck_offset_arg)
    ld.add_action(neck_reverse_arg)
    ld.add_action(neck_auto_arg)

    upper_controller_node = Node(
        package=tracer_pkg_name,
        executable='upper_controller_node',
        name='upper_controller_node',
        output='screen',
        parameters=[
            {'neck_movement': neck_movement},
            {'neck_offset': neck_offset},
            {'neck_reverse': neck_reverse},
            {'neck_auto': neck_auto},
        ],
    )
    ld.add_action(upper_controller_node)

    lower_controller_node = Node(
        package=tracer_pkg_name,
        executable='lower_controller_node',
        name='lower_controller_node',
        output='screen'
    )
    ld.add_action(lower_controller_node)

    wrench_controller_node = Node(
        package=tracer_pkg_name,
        executable='wrench_controller_node',
        name='wrench_controller_node',
        output='screen'
    )
    ld.add_action(wrench_controller_node)

    return ld