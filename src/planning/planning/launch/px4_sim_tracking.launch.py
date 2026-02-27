"""
PX4 仿真环境下的目标跟踪 Launch 文件

支持两种里程计来源：
  1. odom_source:=nav_msgs  — 使用 MAVROS 发布的 nav_msgs/Odometry (默认)
  2. odom_source:=px4_msgs  — 使用 PX4 原生 VehicleOdometry, 通过 px4_odom_bridge 自动转换

数据流:
  [PX4 VehicleOdometry] ──(px4_odom_bridge)──► /odom (nav_msgs/Odometry)
  或 [MAVROS] ─────────────────────────────────► /odom (nav_msgs/Odometry)

  [YOLO Detection2DArray] ──► target_ekf_node (直接订阅, 无需中间转换)

  /odom + depth ──► mapping_node ──gridmap_inflate──► planning_node ──trajectory──► traj_server
  /yolo + /odom ──► target_ekf_node ──target_odom───► planning_node

使用方法:
  # 模式1: MAVROS (默认)
  ros2 launch planning px4_sim_tracking.launch.py

  # 模式2: PX4 原生 odom
  ros2 launch planning px4_sim_tracking.launch.py odom_source:=px4_msgs

  # 自定义 topic
  ros2 launch planning px4_sim_tracking.launch.py odom_source:=px4_msgs px4_odom_topic:=/fmu/out/vehicle_odometry
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    """根据 odom_source 参数动态生成节点列表"""

    odom_source = LaunchConfiguration('odom_source').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    depth_topic = LaunchConfiguration('depth_topic')
    yolo_topic = LaunchConfiguration('yolo_topic')
    target_odom_topic = LaunchConfiguration('target_odom_topic')
    target_class = LaunchConfiguration('target_class').perform(context)

    # 所有节点共享的时间参数
    sim_time_param = {'use_sim_time': use_sim_time}

    # 根据 odom 来源决定下游节点使用的 odom topic
    if odom_source == 'px4_msgs':
        odom_topic_for_nodes = '/odom'
        px4_odom_topic = LaunchConfiguration('px4_odom_topic').perform(context)

        bridge_node = Node(
            package='px4_odom_bridge',
            executable='px4_odom_bridge',
            name='px4_odom_bridge',
            output='screen',
            parameters=[
                sim_time_param,
                {
                    'px4_odom_topic': px4_odom_topic,
                    'ros_odom_topic': odom_topic_for_nodes,
                    'frame_id': 'odom',
                    'child_frame_id': 'base_link',
                },
            ],
        )
    else:
        odom_topic_for_nodes = LaunchConfiguration('nav_odom_topic').perform(context)
        bridge_node = None

    # 相机到机体坐标系的变换 (两个相机共用，取决于安装位置)
    cam_transform = {
        'cam2body_R': [0.0, 0.0, 1.0,
                       -1.0, 0.0, 0.0,
                       0.0, -1.0, 0.0],
        'cam2body_p': [0.0, 0.0, 0.05],
    }

    # ───────────────── 深度相机参数 (mapping 用) ─────────────────
    depth_cam_params = {
        'camera_rate': 30.0,
        'camera_range': 7.0,
        'cam_width': 640,
        'cam_height': 480,
        'cam_fx': 435.1,
        'cam_fy': 435.1,
        'cam_cx': 320.0,
        'cam_cy': 240.0,
        'depth_scaling_factor': 1000.0,
    }

    # ───────────────── 彩色相机参数 (target_ekf YOLO 用) ─────────────────
    color_cam_params = {
        'cam_width': 1920,
        'cam_height': 1080,
        'cam_fx': 1403.5,
        'cam_fy': 1403.5,
        'cam_cx': 960.0,
        'cam_cy': 540.0,
    }

    # ───────────────── 1. Mapping Node ─────────────────
    mapping_container = ComposableNodeContainer(
        name='mapping_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='mapping',
                plugin='mapping::MappingNode',
                name='mapping_node',
                remappings=[
                    ('depth', depth_topic),
                    ('odom', odom_topic_for_nodes),
                ],
                parameters=[
                    sim_time_param,
                    depth_cam_params,
                    cam_transform,
                    {
                        'use_global_map': False,
                        'use_mask': True,
                        'down_sample_factor': 2,
                        'depth_filter_tolerance': 0.15,
                        'depth_filter_mindist': 0.2,
                        'depth_filter_margin': 2,
                        'resolution': 0.15,
                        'local_x': 20.0,
                        'local_y': 20.0,
                        'local_z': 5.0,
                        'inflate_size': 2,
                        'p_min': -199,
                        'p_max': 220,
                        'p_hit': 62,
                        'p_mis': 62,
                        'p_occ': 139,
                        'p_def': -199,
                    },
                ],
            ),
        ],
        output='screen',
    )

    # ───────────────── 2. Target EKF Node ─────────────────
    target_ekf_node = Node(
        package='target_ekf',
        executable='target_ekf_node',
        name='target_ekf_node',
        output='screen',
        remappings=[
            ('yolo', yolo_topic),
            ('odom', odom_topic_for_nodes),
            ('target_odom', target_odom_topic),
        ],
        parameters=[
            sim_time_param,
            color_cam_params,
            cam_transform,
            {
                'pitch_thr': 0.0,
                'ekf_rate': 20,
                'target_class': target_class,
                'target_real_height': 1.7,
            },
        ],
    )

    # ───────────────── 3. Planning Node ─────────────────
    planning_container = ComposableNodeContainer(
        name='planning_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='planning',
                plugin='planning::PlanningNode',
                name='planning_node',
                remappings=[
                    ('odom', odom_topic_for_nodes),
                    ('gridmap_inflate', '/gridmap_inflate'),
                    ('target', target_odom_topic),
                ],
                parameters=[sim_time_param, {
                    'plan_hz': 10,
                    'K': 8,
                    'vmax': 0.8,
                    'amax': 1.5,
                    'rhoT': 100.0,
                    'rhoP': 10000.0,
                    'rhoV': 1000.0,
                    'rhoA': 1000.0,
                    'rhoTracking': 1000.0,
                    'rhosVisibility': 10000.0,
                    'theta_clearance': 0.8,
                    'clearance_d': 0.4,
                    'tolerance_d': 1.0,
                    'tracking_dist': 3.5,
                    'tracking_dur': 3.0,
                    'tracking_dt': 0.2,
                    'debug': False,
                    'fake': False,
                    'prediction/rho_a': 1.0,
                    'prediction/vmax': 4.0,
                }],
            ),
        ],
        output='screen',
    )

    # ───────────────── 4. Trajectory Server ─────────────────
    traj_server_node = Node(
        package='planning',
        executable='traj_server',
        name='traj_server',
        output='screen',
        remappings=[
            ('trajectory', '/trajectory'),
            ('heartbeat', '/heartbeat'),
            ('position_cmd', '/drone_0_planning/pos_cmd'),
        ],
        parameters=[sim_time_param, {
            'time_forward': 1.0,
        }],
    )

    # ───────────────── 5. Mapping Visualization ─────────────────
    mapping_vis_node = Node(
        package='mapping',
        executable='mapping_vis_node',
        name='mapping_vis_node',
        output='screen',
        parameters=[sim_time_param],
        remappings=[
            ('gridmap', '/gridmap'),
            ('gridmap_inflate', '/gridmap_inflate'),
        ],
    )

    # ───────────────── 6. Static TF: world → odom ─────────────────
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_tf',
        arguments=['0', '0', '0.45', '0', '0', '0', 'world', 'odom'],
        parameters=[sim_time_param],
    )

    # ───────────────── 7. RViz2 可视化 ─────────────────
    rviz_config = os.path.join(
        get_package_share_directory('planning'), 'config', '1.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[sim_time_param],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ───────────────── 组装节点列表 ─────────────────
    nodes = []
    if bridge_node is not None:
        nodes.append(bridge_node)
    nodes.extend([
        mapping_container,
        target_ekf_node,
        planning_container,
        traj_server_node,
        mapping_vis_node,
        static_tf_node,
        rviz_node,
    ])
    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ───────────────── Launch Arguments ─────────────────
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间 (Gazebo 发布 /clock)'),

        DeclareLaunchArgument(
            'odom_source',
            default_value='nav_msgs',
            description='里程计来源: "nav_msgs" (MAVROS) 或 "px4_msgs" (PX4 原生 VehicleOdometry)'),

        DeclareLaunchArgument(
            'nav_odom_topic',
            default_value='/Odometry',
            description='[nav_msgs 模式] nav_msgs/Odometry topic (如 /Odometry, /mavros/local_position/odom)'),

        DeclareLaunchArgument(
            'px4_odom_topic',
            default_value='/fmu/out/vehicle_odometry',
            description='[px4_msgs 模式] PX4 原生 VehicleOdometry topic'),

        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/depth/image_raw',
            description='深度相机 topic'),

        DeclareLaunchArgument(
            'yolo_topic',
            default_value='/yolo_detector/detected_bounding_boxes',
            description='YOLO 检测器输出 topic (vision_msgs/Detection2DArray)'),

        DeclareLaunchArgument(
            'target_class',
            default_value='person',
            description='目标类别过滤 (空字符串=不过滤)'),

        DeclareLaunchArgument(
            'target_odom_topic',
            default_value='/target_ekf_odom',
            description='目标 EKF 滤波后的位姿 topic'),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='启动 RViz2 可视化 (true/false)'),

        # 动态生成节点
        OpaqueFunction(function=launch_setup),
    ])
