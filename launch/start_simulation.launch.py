import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Deklaracja argumentów launch
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/ws/png_SLAM_data/custom_params.yaml',
        description='Path to the parameters YAML file for OV2SLAM'
    )
    
    images_folder_arg = DeclareLaunchArgument(
        'images_folder',
        default_value='/ws/png_SLAM_data/left_images',
        description='Path to the folder with PNG images'
    )
    
    timestamp_path_arg = DeclareLaunchArgument(
        'timestamp_path',
        default_value='/ws/png_SLAM_data/timestamp.txt',
        description='Path to the timestamp.txt file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='/ws/src/ov2slam/ov2slam_visualization.rviz',
        description='Path to RViz configuration file'
    )
    
    # do ov2slam arg
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    # Konfiguracja
    params_file = LaunchConfiguration('params_file')
    images_folder = LaunchConfiguration('images_folder')
    timestamp_path = LaunchConfiguration('timestamp_path')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rviz = LaunchConfiguration('enable_rviz')
    
    # Node: OV2SLAM
    ov2slam_node = Node(
        package='ov2slam',
        executable='ov2slam_node',
        name='ov2slam_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[params_file],
        respawn=False,
        emulate_tty=True,
        sigterm_timeout='10',
        sigkill_timeout='15',
    )
    
    # Node: FEEDER_PNG z parametrami ROS2
    feeder_png_node = Node(
        package='ov2slam',
        executable='feeder_png',
        name='feeder_png',
        output='screen',
        parameters=[{
            'images_folder': images_folder,
            'timestamp_path': timestamp_path,
            'use_sim_time': use_sim_time,
        }],
        respawn=False,
        emulate_tty=True,
    )
    
    # Node: RViz2 z software rendering dla Dockera
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        respawn=False,
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(enable_rviz),
    )
    
    # Opóźnij start FEEDER_PNG o 3 sekundy (aby ov2slam był gotowy)
    delayed_feeder = TimerAction(
        period=3.0,
        actions=[feeder_png_node]
    )
    
    # Opóźnij start RViz o 2 sekundy
    delayed_rviz = TimerAction(
        period=2.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([
        # Argumenty
        params_file_arg,
        images_folder_arg,
        timestamp_path_arg,
        rviz_config_arg,
        use_sim_time_arg,
        enable_rviz_arg,
        
        # Nodes (w kolejności)
        ov2slam_node,       # Start natychmiast
        delayed_rviz,       # Start po 2s
        delayed_feeder,     # Start po 3s
    ])