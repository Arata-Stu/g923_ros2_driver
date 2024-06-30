from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir


def generate_launch_description():
    # パラメータファイルへのパスを設定
    param_file_path = '/aichallenge/workspace/src/aichallenge_submit/my_package/g923_ros2_driver/config/param.yaml'

    return LaunchDescription([
        Node(
            package='g923_ros2_driver',
            executable='g923_driver_node',
            name='g923_driver_node',
            parameters=[param_file_path],
            output='screen',
        ),
        
    ])