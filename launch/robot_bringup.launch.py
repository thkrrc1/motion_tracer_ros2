from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    tracer_pkg_name = 'motion_tracer_ros2'
    tracer_pkg = FindPackageShare(tracer_pkg_name)

    ld = LaunchDescription()

    # arg setting
    robot_model = LaunchConfiguration('robot_model')
    simulation = LaunchConfiguration('simulation')
    slam_mode = LaunchConfiguration('slam')
    display_rviz2 = LaunchConfiguration('display_rviz2')
    neck_movement = LaunchConfiguration('neck_movement')
    neck_offset = LaunchConfiguration('neck_offset')
    neck_reverse = LaunchConfiguration('neck_reverse')
    neck_auto = LaunchConfiguration('neck_auto')

    robot_model_arg = DeclareLaunchArgument('robot_model', default_value='noid_lifter_mover')
    simulation_arg = DeclareLaunchArgument('simulation', default_value='false')
    slam_mode_arg = DeclareLaunchArgument('slam', default_value='false')
    display_rviz2_arg = DeclareLaunchArgument('display_rviz2', default_value='true')
    neck_movement_arg = DeclareLaunchArgument('neck_movement', default_value='increment')
    neck_offset_arg = DeclareLaunchArgument('neck_offset', default_value='0')
    neck_reverse_arg = DeclareLaunchArgument('neck_reverse', default_value='false')
    neck_auto_arg = DeclareLaunchArgument('neck_auto', default_value='false')

    ld.add_action(robot_model_arg)
    ld.add_action(simulation_arg)
    ld.add_action(slam_mode_arg)
    ld.add_action(display_rviz2_arg)
    ld.add_action(neck_movement_arg)
    ld.add_action(neck_offset_arg)
    ld.add_action(neck_reverse_arg)
    ld.add_action(neck_auto_arg)

    # robot bringup
    robot_pkg_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(robot_model),
                'launch',
                'bringup_robot.launch.py'
            ])
        ),
        launch_arguments={
            'simulation': simulation,
            'slam': slam_mode,
            'display_rviz2': display_rviz2
        }.items()
    )
    ld.add_action(robot_pkg_launch)

    # data converter
    data_converter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                tracer_pkg,
                'launch',
                'data_converter.launch.py'
            ])
        ),
        launch_arguments={
            'neck_movement': neck_movement,
            'neck_offset': neck_offset,
            'neck_reverse': neck_reverse,
            'neck_auto': neck_auto
        }.items()
    )
    ld.add_action(data_converter_launch)

    return ld