from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        )
    )

    # SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # Nav2 (SLAM mode)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'True'
        }.items()
    )
    # RViz with config
    rviz_config_file = os.path.join(
        get_package_share_directory('project'),
        'rviz',
        'rviz_conf.rviz'
    )
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    delayed_nav2 = TimerAction(
        period=30.0,   # wait some seconds before starting Nav2
        actions=[nav2_launch]
    )


    return LaunchDescription([
        gazebo_launch,
        slam_launch,
        #nav2_launch,
        rviz_node,
        delayed_nav2
    ])


#
