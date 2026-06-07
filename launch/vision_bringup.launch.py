import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

def generate_launch_description():
    # Gazebo 시뮬레이터 실행 
    pkg_share = get_package_share_directory('robot_vision')
    # ur3 + 카메라 xacro 파일 파싱
    xacro_file = os.path.join(pkg_share, 'urdf', 'ur3_vision.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}
    # 월드 모델 파일
    world_file = os.path.join(pkg_share, 'worlds', 'yolo_test.sdf')
    
    # 로봇 상태 퍼블리셔 (관절 위치와 tf 좌표계 계산해주는 노드)
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 가제보 실행
    gazebo_process = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file, '-r'],
        output='screen'
    )

    # 가제보에 로봇 소환
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'ur3_vision',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.0'
        ],
        output='screen'
    )

    # ROS-Gazebo 브릿지 노드 실행 (카메라 데이터 통신)
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
        rsp_node,
        gazebo_process,
        spawn_node,
        bridge_node,
        yolo_node
    ])
