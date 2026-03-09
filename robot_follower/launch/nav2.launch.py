import os
from ament_index_python.packages import get_package_share_directory
from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from nav2_common.launch import RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('setup_path', default_value='/etc/clearpath/'),
    DeclareLaunchArgument('scan_topic', default_value=''),
    DeclareLaunchArgument('robot_base_frame', default_value='base_link')
]

def launch_setup(context, *args, **kwargs):
    pkg_clearpath_nav2_demos = get_package_share_directory('clearpath_nav2_demos')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    setup_path = LaunchConfiguration('setup_path')
    scan_topic = LaunchConfiguration('scan_topic')
    robot_base_frame = LaunchConfiguration('robot_base_frame')

    config = read_yaml(os.path.join(setup_path.perform(context), 'robot.yaml'))
    clearpath_config = ClearpathConfig(config)
    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    eval_scan_topic = scan_topic.perform(context)
    if len(eval_scan_topic) == 0:
        eval_scan_topic = f'/{namespace}/sensors/lidar2d_0/scan'

    file_parameters = PathJoinSubstitution([pkg_clearpath_nav2_demos, 'config', platform_model, 'nav2.yaml'])

    rewritten_parameters = RewrittenYaml(
        source_file=file_parameters,
        param_rewrites={
            'topic': eval_scan_topic, 
            'robot_base_frame': robot_base_frame,
            'base_frame': robot_base_frame 
        },
        convert_types=True
    )

    launch_nav2 = PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    return [GroupAction([
        SetRemap('base_link', robot_base_frame),
        SetRemap('/' + namespace + '/odom', '/' + namespace + '/platform/odom'),
        SetRemap('map', '/' + namespace + '/map'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('params_file', rewritten_parameters),
                ('use_composition', 'False'),
                ('namespace', namespace),
            ]
        ),
    ])]

def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=launch_setup)])