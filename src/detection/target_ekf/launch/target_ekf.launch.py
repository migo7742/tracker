import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_name = 'target_ekf'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 2. 拼接配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'camera.yaml')

    # 3. 定义节点
    target_ekf_node = Node(
        package=pkg_name,
        executable='target_ekf_node',  # 对应 CMakeLists.txt 中的 add_executable 名称
        name='target_ekf_node',        # 对应 yaml 文件中的节点名
        output='screen',
        emulate_tty=True,
        parameters=[
            config_file,          # 加载 YAML 文件
            {'pitch_thr': 37.0}   # 覆盖或追加参数 (对应原 launch 中的 param 标签)
        ],
        remappings=[
            # 格式：('原话题名', '新话题名')
            # 对应原 launch 中的 <remap from="~yolo" ...>
            ('yolo', '/yolov5trt/bboxes_pub'), 
            # 对应原 launch 中的 <remap from="~odom" ...>
            ('odom', '/ekf/ekf_odom')
        ]
    )

    return LaunchDescription([
        target_ekf_node
    ])
