# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py
#
#
#
#
# VINS-Fusion example:
#   Add to your ros2 workspace the package https://github.com/zinuok/VINS-Fusion-ROS2
#   Apply this patch https://gist.github.com/matlabbe/ebbb343cd744da9d6d6d6ded2e1557fd
#      -> Revert "#define USE_GPU" change if you want to build VINS-Fusion with GPU support.
#   That may be counterintuitive, but we need to build VINS-Fusion first, then rebuild rtabmap with VINS-Fusion support.
#      -> in your ros2 workspace, do "colcon build --packages-select vins"
#      -> go back under rtabmap library repo, then rebuild/install with "cmake -DWITH_VINS_FUSION=ON ..."
#      -> do "colcon build" in your ros2 workspace again to rebuild rtabmap_ros
#   $ ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py odom_args:="--Odom/Strategy 9 OdomVINSFusion/ConfigPath ~/ros2_ws/src/VINS-Fusion-ROS2/config/realsense_d435i/realsense_stereo_imu_config.yaml"
#      -> set "imu: 1" in realsense_stereo_imu_config.yaml to do stereo inertial odometry, otherwise only stereo odometry is done.
#
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
    parameters={
          'frame_id':LaunchConfiguration('frame_id'),
          'subscribe_stereo':True,
          'subscribe_odom_info':True,
          'wait_imu_to_init':True,
          'subscribe_depth':True,
          'use_sim_time':LaunchConfiguration('use_bag'),
          'database_path':LaunchConfiguration('database_path'),
          'Mem/InitWMWithAllNodes':ParameterValue(
              PythonExpression(['"', LaunchConfiguration("localization"), '".lower() == "true"']),
              value_type=str),
          'Mem/IncrementalMemory':ParameterValue(
              PythonExpression(['"', LaunchConfiguration("localization"), '".lower() != "true"']),
              value_type=str),
          'Reg/Force3DoF':'true',               # z, pitch and roll of pose are set to 0 (flat ground)
          'Grid/3D':'false',                    # Use 2D occupancy
          'Grid/RayTracing':'true',             # Fill empty space
          'Grid/NormalsSegmentation':'false',   # Use passthrough filter to detect obstacles (flat ground)
          'Grid/MaxGroundHeight':'0.05',        # All points above 0.05 m are obstacles
          'Grid/MaxObstacleHeight':'1.0',       # All points above 1 m are ignored
          'Grid/RangeMax':'5',                  # All points beyond 5 m are ignored
          'Odom/ResetCountdown':'10',           # Auto-reset odometry after 10 lost frames

          # GPU optimizations (requires CUDA and compatible graphcis card)
          'SURF/GpuVersion':'true',             # Use GPU version of SURF feature detector (requires CUDA)
          'FAST/Gpu':'true',                    # Use GPU version of FAST feature detector (requires CUDA)
          'GFTT/Gpu':'true',                    # Use GPU version of GFTT feature detector (requires CUDA)
          'ORB/Gpu':'true',                     # Use GPU version of ORB feature detector (requires CUDA)
          'SIFT/Gpu':'true',                    # Use GPU version of SIFT feature detector (requires CUDA)
          'SuperPoint/Cuda':'true',             # Use CUDA for SuperPoint feature detector (requires Torch with CUDA)
          'SuperPointRpautrat/Cuda':'true',     # Use CUDA for SuperPoint-Rpautrat feature detector (requires Torch with CUDA)
          'PyDetector/Cuda':'true',             # Use CUDA for custom Python feature detector
          'Vis/CorFlowGpu':'true',              # Use GPU for optical flow correspondence computation (requires CUDA)
          'Stereo/Gpu':'true'                   # Use GPU for stereo matching (requires CUDA)
    }

    camera_name = LaunchConfiguration('camera_name')

    left_image = PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='infra1/image_rect_raw')])
    left_info = PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='infra1/camera_info')])
    right_image = PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='infra2/image_rect_raw')])
    right_info = PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='infra2/camera_info')])

    remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', left_image),
          ('left/camera_info', left_info),
          ('right/image_rect', right_image),
          ('right/camera_info', right_info)]

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

        #Hack to disable IR emitter
        SetParameter(name='depth_module.emitter_enabled', value=0),
        
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
                            'enable_infra1': 'true',
                            'enable_infra2': 'true',
                            'enable_sync': 'true',
                            'publish_tf': LaunchConfiguration('publish_tf')}.items(),
            ),

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[parameters],
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args")],
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=[LaunchConfiguration("args")]),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[parameters,
                        {'odometry_node_name': "stereo_odometry"}],
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration("rtabmap_viz"))),
                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='imu')]))]),

        # Obstacle detection with the camera for nav2 local costmap.
        # First, we need to convert depth image to a point cloud.
        # Second, we segment the floor from the obstacles.
        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02}],
            remappings=[('depth/image', PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='aligned_depth_to_color/image_raw')])),
                        # Image, input # if doesnt, work, try /gravikart_camera/depth/image_raw, also try image_rect_raw instead of image_raw

                        ('depth/camera_info', PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='aligned_depth_to_color/camera_info')])),
                        # CameraInfo, input # if doesnt, work, try /gravikart_camera/depth/camera_info

                        ('cloud', PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='cloud')]))]),
                        # PointCloud2, output
        Node(
            package='rtabmap_util', executable='obstacles_detection', output='screen',
            parameters=[parameters],
            remappings=[('cloud', PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='cloud')])),
                        # PointCloud2, input

                        ('obstacles', PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='obstacles')])),
                        # PointCloud2, output

                        ('ground', PathJoinSubstitution([TextSubstitution(text='/'), camera_name, TextSubstitution(text='ground')]))]),
                        # PointCloud2, output
    ])
