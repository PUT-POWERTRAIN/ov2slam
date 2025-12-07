import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Deklaracja argumentów launch
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/datasets/custom_params.yaml',
        description='Path to the parameters YAML file for OV2SLAM'
    )
    images_folder_left_arg = DeclareLaunchArgument(
        'images_folder_left',
        default_value='/datasets/left_images',
        description='Path to the folder with PNG images for left stereo camer'
    )
    images_folder_right_arg = DeclareLaunchArgument(
        'images_folder_right',
        default_value='/datasets/right_images',
        description='Path to the folder with PNG images for right stereo camera'
    )
    enable_stereo_arg = DeclareLaunchArgument(
        'enable_stereo',
        default_value='True',
        description='false - mono simulation, true - stereo simulation'
    )
    timestamp_path_arg = DeclareLaunchArgument(
        'timestamp_path',
        default_value='/datasets/timestamp.txt',
        description='Path to the timestamp.txt file'
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
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='true if loop on dataset'
    )
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu',
        default_value='true',
        description='enables imu feeder'
    )
    imu_path_arg = DeclareLaunchArgument(
        'imu_path',
        default_value='/datasets/ahrs.txt',
        description='enables imu feeder'
    )

    # Konfiguracja
    params_file = LaunchConfiguration('params_file')
    images_folder_left = LaunchConfiguration('images_folder_left')
    images_folder_right = LaunchConfiguration('images_folder_right')
    enable_stereo = LaunchConfiguration('enable_stereo')
    timestamp_path = LaunchConfiguration('timestamp_path')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rviz = LaunchConfiguration('enable_rviz')
    loop = LaunchConfiguration('loop')
    enable_imu = LaunchConfiguration('enable_imu')
    imu_path = LaunchConfiguration('imu_path')

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
            'images_folder_left': images_folder_left,
            'images_folder_right': images_folder_right,
            'enable_stereo': enable_stereo,
            'timestamp_path': timestamp_path,
            'use_sim_time': use_sim_time,
            'loop': loop,
        }],
        respawn=False,
        emulate_tty=True,
    )

    feeder_imu_node = Node(
        package='ov2slam',
        executable='feeder_imu',
        name='feeder_imu',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'loop': loop,
            'imu_path': imu_path,
        }],
        condition = IfCondition(enable_imu),
        respawn=False,
        emulate_tty=True,
    )

    imu_transform_node = Node(
        package='ov2slam',
        executable='imu_transform',
        name='imu_transform',
        output='screen',
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
    )


    # Opóźnij start FEEDER_PNG o 3 sekundy
    delayed_feeder = TimerAction(
        period=3.0,
        actions=[feeder_png_node, feeder_imu_node, imu_transform_node]
    )

    # Opóźnij start RViz o 2 sekundy
    delayed_rviz = GroupAction(
        actions=[
            TimerAction(
                period=2.0,
                actions=[rviz_node]
            )
        ],
        condition=IfCondition(enable_rviz)
    )

    return LaunchDescription([
        # Argumenty
        params_file_arg,
        images_folder_left_arg,
        images_folder_right_arg,
        enable_stereo_arg,
        timestamp_path_arg,
        rviz_config_arg,
        use_sim_time_arg,
        enable_rviz_arg,
        loop_arg,
        enable_imu_arg,
        imu_path_arg,
        # Nodes
        ov2slam_node,
        delayed_rviz,
        delayed_feeder,
    ])