import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Gazebo 시뮬레이터 실행 (worlds 폴더의 sdf 파일 로드)
    pkg_share = get_package_share_directory('robot_vision')
    world_file = os.path.join(pkg_share, 'worlds', 'camera_test.sdf')
    
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen'
    )

    # ROS-Gazebo 브릿지 노드 실행 (단방향 통신)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image'
        ],
        output='screen'
    )

    # YOLO 비전 노드 실행
    yolo_node = Node(
        package='robot_vision',
        executable='yolo_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_process,
        bridge_node,
        yolo_node
    ])
