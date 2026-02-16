# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    parameters=[{
          'frame_id':LaunchConfiguration('frame_id'),
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':True,
          'database_path': LaunchConfiguration('database_path'),
          'Mem/InitWMWithAllNodes':ParameterValue(
              PythonExpression(['"', LaunchConfiguration("localization"), '".lower() == "true"']),
              value_type=str),
          'Mem/IncrementalMemory':ParameterValue(
              PythonExpression(['"', LaunchConfiguration("localization"), '".lower() != "true"']),
              value_type=str)}]
    
    camera_name = LaunchConfiguration('camera_name')

    rgb_image = PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='color/image_raw')])
    rgb_info = PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='color/camera_info')])
    depth_image = PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='aligned_depth_to_color/image_raw')])

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', rgb_image),
          ('rgb/camera_info', rgb_info),
          ('depth/image', depth_image)]
    
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_bag', default_value=TextSubstitution(text = 'false'),
            description='Use bag file as input instead of live camera'),

        DeclareLaunchArgument(
            'localization', default_value=TextSubstitution(text = 'false'),
            description = "Run RTAB-Map in localization mode (map is not updated, only used for localization)"),

        DeclareLaunchArgument(
            'unite_imu_method', default_value='2',
            description='0-None, 1-copy, 2-linear_interpolation. Use unite_imu_method:="1" if imu topics stop being published.'),

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),
        
        DeclareLaunchArgument(
            'args', default_value='',
            description='Extra arguments set to rtabmap and odometry nodes.'),
        
        DeclareLaunchArgument(
            'odom_args', default_value='',
            description='Extra arguments just for odometry node. If the same argument is already set in \"args\", it will be overwritten by the one in \"odom_args\".'),

        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtabmap.db',
            description='Where is the map saved/loaded.'),
        
        DeclareLaunchArgument(
            'frame_id', default_value='camera_link',
            description='Robot base frame used by RTAB-Map (must exist in TF).'),

        DeclareLaunchArgument(
            'publish_tf', default_value='true',
            description='Whether the RealSense driver should publish the internal camera TF frames'),

        DeclareLaunchArgument(
            'camera_name', default_value='camera',
            description='Camera namespace to prefix all camera topic names'),

        DeclareLaunchArgument(
            'rtabmap_viz', default_value='true',
            description='Launch rtabmap_viz GUI'),

        # Launch camera driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'), 'launch'),
                '/rs_launch.py']),
            condition = UnlessCondition(LaunchConfiguration("use_bag")),
            launch_arguments={'camera_namespace': '',
                            'camera_name': LaunchConfiguration('camera_name'),
                            'enable_gyro': 'true',
                            'enable_accel': 'true',
                            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
                            'align_depth.enable': 'true',
                            'enable_sync': 'true',
                            'rgb_camera.profile': '640x360x30',
                            'publish_tf': LaunchConfiguration('publish_tf')}.items()
        ),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args")],
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=[LaunchConfiguration("args")]),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration("rtabmap_viz"))),

        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='imu')]))]),
    ])
