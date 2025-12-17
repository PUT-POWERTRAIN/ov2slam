import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
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
        default_value=PathJoinSubstitution([FindPackageShare('ov2slam'), 'ov2slam_visualization.rviz']),
        description='Path to RViz configuration file'
    )
    rviz_config_ahrs_arg = DeclareLaunchArgument(
        'rviz_config_ahrs',
        default_value=PathJoinSubstitution([FindPackageShare('ov2slam'), 'ov2slam_visualization_world_ahrs.rviz']),
        description='Path to RViz configuration file for AHRS-aligned fixed frame'
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
    playback_speed_arg = DeclareLaunchArgument(
        'playback_speed',
        default_value='1.0',
        description='Playback speed multiplier (1.0 = real-time)'
    )
    start_timestamp_arg = DeclareLaunchArgument(
        'start_timestamp',
        default_value='-1.0',
        description='Dataset unix start time in seconds; <0 disables'
    )
    max_images_arg = DeclareLaunchArgument(
        'max_images',
        default_value='-1',
        description='Max number of images to publish; <0 disables'
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
    enable_gt_arg = DeclareLaunchArgument(
        'enable_gt',
        default_value='true',
        description='enables ground truth feeder'
    )
    gt_path_arg = DeclareLaunchArgument(
        'gt_path',
        default_value='/datasets/baseline.txt',
        description='Path to baseline trajectory file: timestamp qx qy qz qw x y z'
    )
    gt_zero_origin_arg = DeclareLaunchArgument(
        'gt_zero_origin',
        default_value='true',
        description='Subtract first GT pose as origin (true) or keep absolute coordinates (false)'
    )
    gt_frame_id_arg = DeclareLaunchArgument(
        'gt_frame_id',
        default_value='world_ahrs',
        description='Frame id for gt_pose (e.g. world_gt or world_ahrs)'
    )
    enable_csv_logger_arg = DeclareLaunchArgument(
        'enable_csv_logger',
        default_value='false',
        description='Enable CSV trajectory logger (/vo_pose and /gt_pose)'
    )
    csv_output_dir_arg = DeclareLaunchArgument(
        'csv_output_dir',
        default_value='/datasets/exports',
        description='Output directory for CSV logs'
    )
    enable_ahrs_align_tf_arg = DeclareLaunchArgument(
        'enable_ahrs_align_tf',
        default_value='true',
        description='Publish static TF to rotate VO world using AHRS (measurement aid)'
    )
    ahrs_parent_frame_arg = DeclareLaunchArgument(
        'ahrs_parent_frame',
        default_value='world_ahrs',
        description='Parent frame for AHRS-aligned world'
    )
    ahrs_child_frame_arg = DeclareLaunchArgument(
        'ahrs_child_frame',
        default_value='world',
        description='Child frame (OV2SLAM world frame id)'
    )

    # Konfiguracja
    params_file = LaunchConfiguration('params_file')
    images_folder_left = LaunchConfiguration('images_folder_left')
    images_folder_right = LaunchConfiguration('images_folder_right')
    enable_stereo = LaunchConfiguration('enable_stereo')
    timestamp_path = LaunchConfiguration('timestamp_path')
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_ahrs = LaunchConfiguration('rviz_config_ahrs')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rviz = LaunchConfiguration('enable_rviz')
    loop = LaunchConfiguration('loop')
    playback_speed = LaunchConfiguration('playback_speed')
    start_timestamp = LaunchConfiguration('start_timestamp')
    max_images = LaunchConfiguration('max_images')
    enable_imu = LaunchConfiguration('enable_imu')
    imu_path = LaunchConfiguration('imu_path')
    enable_gt = LaunchConfiguration('enable_gt')
    gt_path = LaunchConfiguration('gt_path')
    gt_zero_origin = LaunchConfiguration('gt_zero_origin')
    gt_frame_id = LaunchConfiguration('gt_frame_id')
    enable_csv_logger = LaunchConfiguration('enable_csv_logger')
    csv_output_dir = LaunchConfiguration('csv_output_dir')
    enable_ahrs_align_tf = LaunchConfiguration('enable_ahrs_align_tf')
    ahrs_parent_frame = LaunchConfiguration('ahrs_parent_frame')
    ahrs_child_frame = LaunchConfiguration('ahrs_child_frame')

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
    feeder_dataset_node = Node(
        package='ov2slam',
        executable='feeder_dataset',
        name='feeder_dataset',
        output='screen',
        parameters=[{
            'enable_images': True,
            'images_folder_left': images_folder_left,
            'images_folder_right': images_folder_right,
            'enable_stereo': enable_stereo,
            'timestamp_path': timestamp_path,
            'use_sim_time': use_sim_time,
            'loop': loop,
            'playback_speed': playback_speed,
            'start_timestamp': start_timestamp,
            'max_images': max_images,
            'enable_imu': enable_imu,
            'imu_path': imu_path,
            'enable_gt': enable_gt,
            'gt_path': gt_path,
            'gt_frame_id': gt_frame_id,
            'zero_origin': gt_zero_origin,
        }],
        respawn=False,
        emulate_tty=True,
    )

    trajectory_csv_logger_node = Node(
        package='ov2slam',
        executable='trajectory_csv_logger',
        name='trajectory_csv_logger',
        output='screen',
        parameters=[{
            'output_dir': csv_output_dir,
            'vo_topic': '/vo_pose',
            'gt_topic': '/gt_pose',
            'vo_filename': 'vo_pose.csv',
            'gt_filename': 'gt_pose.csv',
        }],
        condition=IfCondition(enable_csv_logger),
        respawn=False,
        emulate_tty=True,
    )

    ahrs_align_tf_node = Node(
        package='ov2slam',
        executable='ahrs_align_tf',
        name='ahrs_align_tf',
        output='screen',
        parameters=[{
            'imu_topic': '/imu_data',
            'vo_topic': '/vo_pose',
            'parent_frame': ahrs_parent_frame,
            'child_frame': ahrs_child_frame,
        }],
        condition=IfCondition(enable_ahrs_align_tf),
        respawn=False,
        emulate_tty=True,
    )

    # Node: RViz2 z software rendering dla Dockera
    rviz_node_world = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        respawn=False,
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1', 'QT_X11_NO_MITSHM': '1'},
        condition=UnlessCondition(enable_ahrs_align_tf),
    )

    rviz_node_ahrs = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_ahrs],
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        respawn=False,
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1', 'QT_X11_NO_MITSHM': '1'},
        condition=IfCondition(enable_ahrs_align_tf),
    )

    # Opóźnij start FEEDER_PNG o 3 sekundy
    delayed_feeder = TimerAction(
        period=3.0,
        actions=[feeder_dataset_node, trajectory_csv_logger_node, ahrs_align_tf_node]
    )

    # Opóźnij start RViz o 2 sekundy
    delayed_rviz = GroupAction(
        actions=[
            TimerAction(
                period=2.0,
                actions=[rviz_node_world, rviz_node_ahrs]
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
        rviz_config_ahrs_arg,
        use_sim_time_arg,
        enable_rviz_arg,
        loop_arg,
        playback_speed_arg,
        start_timestamp_arg,
        max_images_arg,
        enable_imu_arg,
        imu_path_arg,
        enable_gt_arg,
        gt_path_arg,
        gt_zero_origin_arg,
        gt_frame_id_arg,
        enable_csv_logger_arg,
        csv_output_dir_arg,
        enable_ahrs_align_tf_arg,
        ahrs_parent_frame_arg,
        ahrs_child_frame_arg,
        # Nodes
        ov2slam_node,
        delayed_rviz,
        delayed_feeder,
    ])
