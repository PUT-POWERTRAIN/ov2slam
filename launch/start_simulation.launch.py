import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import launch.conditions


def generate_launch_description():
    # Deklaracja argumentów launch
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/ws/png_SLAM_data/custom_params.yaml',
        description='Path to the parameters YAML file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='/ws/src/ov2slam/ov2slam_visualization.rviz',
        description='Path to RViz configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    data_path_arg = DeclareLaunchArgument(
        'data_path',
        default_value='/ws/png_SLAM_data',
        description='Path to PNG SLAM data directory'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    
    # Konfiguracja
    params_file = LaunchConfiguration('params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    data_path = LaunchConfiguration('data_path')
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
    
    # Node: FEEDER_PNG z ExecuteProcess dla poprawnego przekazania argumentu
    # Używamy ExecuteProcess zamiast Node, bo Node automatycznie dodaje --ros-args
    # co powoduje problem: argv[1] staje się "--ros-args" zamiast ścieżką
    feeder_png_process = ExecuteProcess(
        cmd=[
            '/ws/install/ov2slam/lib/ov2slam/FEEDER_PNG',
            data_path  # argv[1] - ścieżka do folderu
        ],
        output='screen',
        shell=False,
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
        # Software rendering dla Dockera bez GPU
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'},
        condition=IfCondition(enable_rviz),
    )
    
    # Opóźnij start FEEDER_PNG o 3 sekundy (aby ov2slam był gotowy)
    delayed_feeder = TimerAction(
        period=3.0,
        actions=[feeder_png_process]
    )
    
    # Opóźnij start RViz o 2 sekundy
    delayed_rviz = TimerAction(
        period=2.0,
        actions=[rviz_node]
    )
    
    return LaunchDescription([
        # Argumenty
        params_file_arg,
        rviz_config_arg,
        use_sim_time_arg,
        data_path_arg,
        enable_rviz_arg,
        
        # Nodes (w kolejności)
        ov2slam_node,       # Start natychmiast
        delayed_rviz,       # Start po 2s
        delayed_feeder,     # Start po 3s
    ])