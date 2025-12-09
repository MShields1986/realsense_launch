from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

package_dir = FindPackageShare('realsense_launch')

launch_params =         [{'name': 'rviz',                         'default': 'true',        'description': 'flag to run RViz'},
                         {'name': 'logging',                      'default': 'true',        'description': 'flag to enable ROS bag recording'}
                         ]

realsense_node_params = [{'name': 'serial_no',                    'default': '',            'description': 'choose device by serial number'},
                         {'name': 'device_type',                  'default': '',            'description': 'choose device by type'},
                         {'name': 'camera_name',                  'default': 'rs_camera',   'description': 'camera unique name'}, 
                         {'name': 'camera_namespace',             'default': 'rs_camera',   'description': 'camera namespace'},
                         {'name': 'log_level',                    'default': 'info',        'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                         {'name': 'intra_process_comms',          'default': 'true',        'description': 'enable intra-process communication'},
                         {'name': 'clip_distance',                'default': '2.5',         'description': 'distance limit for depth data'},
                         {'name': 'rgb_camera.color_profile',     'default': '1280x720x15', 'description': 'color image size and rate'},
                         {'name': 'depth_module.depth_profile',   'default': '848x480x15',  'description': 'depth image size and rate'},
                         {'name': 'enable_color',                 'default': 'true',        'description': 'enable color stream'},
                         {'name': 'enable_depth',                 'default': 'true',        'description': 'enable depth stream'},
                         {'name': 'enable_infra',                 'default': 'false',       'description': 'enable infra stream'},
                         {'name': 'enable_infra1',                'default': 'false',       'description': 'enable infra1 stream'},
                         {'name': 'enable_infra2',                'default': 'false',       'description': 'enable infra2 stream'},
                         {'name': 'enable_gyro',                  'default': 'false',       'description': 'enable gyro stream'},
                         {'name': 'enable_accel',                 'default': 'false',       'description': 'enable accel stream'},
                         {'name': 'unite_imu_method',             'default': '0',           'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                         {'name': 'enable_sync',                  'default': 'true',        'description': 'enable sync mode'},
                         {'name': 'pointcloud.enable',            'default': 'false',       'description': ''},
                         {'name': 'enable_rgbd',                  'default': 'true',        'description': 'enable rgbd topic'},
                         {'name': 'align_depth.enable',           'default': 'true',        'description': 'enable align depth filter'},
                         {'name': 'publish_tf',                   'default': 'true',        'description': '[bool] enable/disable publishing static & dynamic TF'},
                         {'name': 'tf_publish_rate',              'default': '50.0',        'description': '[double] rate in HZ for publishing dynamic TF'},
                         #{'name': 'config_file',                  'default': [config],      'description': 'yaml config file'},
                        ]

register_node_params = [{'name': 'queue_size',                     'default': '10',          'description': 'size of message queue for synchronizing subscribed topics'},
                        {'name': 'use_rgb_timestamp',              'default': 'true',          'description': 'use timestamp of rgb image instead of depth image for the registered image'}
                        ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(launch_params) +
        declare_configurable_parameters(realsense_node_params) +
        declare_configurable_parameters(register_node_params) +
        [
        ComposableNodeContainer(name='realsense_container',
                                namespace='',
                                package='rclcpp_components',
                                # executable='component_container',
                                executable='component_container_mt',
                                parameters=[{'thread_num': 10, 'use_sim_time': True}],
                                composable_node_descriptions=[
                                            ComposableNode(package='realsense2_camera',
                                                           namespace='',
                                                           plugin='realsense2_camera::RealSenseNodeFactory',
                                                           name="camera",
                                                           parameters=[set_configurable_parameters(realsense_node_params)],
                                                           extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}],
                                                           ),
                                            ComposableNode(package='image_proc',
                                                           namespace='',
                                                           plugin='image_proc::RectifyNode',
                                                           name='rectify_node',
                                                           remappings=[('camera_info', '/camera/color/camera_info'),
                                                                       ('image', '/camera/color/image_raw'),
                                                                       ('image_rect', '/camera/color/image_rect')
                                                                       ],
                                                            extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}],
                                                           ),
                                            ComposableNode(package='depth_image_proc',
                                                           namespace='',
                                                           plugin='depth_image_proc::RegisterNode',
                                                           name='register_node',
                                                           parameters=[set_configurable_parameters(register_node_params)],
                                                           remappings=[('depth/image_rect', '/camera/depth/image_rect_raw'),
                                                                       ('depth/camera_info', '/camera/depth/camera_info'),
                                                                       ('rgb/camera_info', '/camera/color/camera_info'),
                                                                       ('depth_registered/camera_info', '/camera/depth_registered/camera_info'),
                                                                       ('depth_registered/image_rect', '/camera/depth_registered/image_rect_raw'),
                                                                       ],
                                                            extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}],
                                                           ),
                                            ComposableNode(package='depth_image_proc',
                                                           namespace='',
                                                           plugin='depth_image_proc::PointCloudXyzrgbNode',
                                                           name='xyzrgb_points',
                                                           remappings=[('depth_registered/image_rect', '/camera/depth_registered/image_rect_raw'),
                                                                       ('rgb/image_rect_color', '/camera/color/image_rect'),
                                                                       ('rgb/camera_info', '/camera/color/camera_info'),
                                                                       ('points', '/camera/color/pointcloud'),
                                                                       ],
                                                            extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}],
                                                           ),
                                            ComposableNode(package='realsense_launch',
                                                           namespace='',
                                                           plugin='realsense_launch::VoxelGridNode',
                                                           name='voxel_grid',
                                                           remappings=[('input', '/camera/color/pointcloud'),
                                                                       ('output', '/camera/color/voxel_grid')
                                                                       ],
                                                            parameters=[{'leaf_size': 0.05,
                                                                         'filter_field_name': 'z',
                                                                         'filter_limit_min': 0.01,
                                                                         'filter_limit_max': 2.30,
                                                                         'filter_limit_negative': False,
                                                                         'queue_size': 10
                                                                         }],
                                                            extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}],
                                                           )
                                ],
                                output='screen',
                                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                                )
        ] +
        [
        Node(package='rviz2',
                     executable='rviz2',
                     output='screen',
                     arguments=['-d', PathJoinSubstitution([package_dir, 'rviz', 'template.rviz'])],
                     condition=IfCondition(LaunchConfiguration('rviz')),
                     )
        ] +
        [
        ExecuteProcess(cmd=['ros2', 'bag', 'record',
                            '-o', '/ros2_ws/src/realsense_launch/bags/realsense_recording',
                            '--topics',
                            '/camera/color/voxel_grid',
                            '/camera/color/camera_info',
                            '/camera/color/image_raw',
                            '/camera/depth/camera_info',
                            '/camera/depth/image_rect_raw',
                            '/tf',
                            '/tf_static',
                            '/clock'],
                       output='screen',
                       condition=IfCondition(LaunchConfiguration('logging')),
                       )
        ]
        )
