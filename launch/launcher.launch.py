from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime


def launch_setup(context, *args, **kwargs):
    # 1) bag 저장 경로 만들기
    base_bag_dir = os.path.expanduser('~/hitl_ws/src/flying_pen/bag/folder')
    os.makedirs(base_bag_dir, exist_ok=True)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir = os.path.join(base_bag_dir, f"rosbag2_{ts}")

    # 2) 패키지 share 경로에서 urdf, rviz 찾기
    fp_share = get_package_share_directory('flying_pen')
    urdf_path = os.path.join(fp_share, 'models', 'model.urdf')
    rviz_config = os.path.join(fp_share, 'config', 'config.rviz')

    actions = []

    # 3) data_logging 노드 (제일 먼저)
    data_logging_node = Node(
        package='flying_pen',
        executable='data_logging',
        name='data_logging',
        output='screen'
    )
    actions.append(data_logging_node)

    # 4) rosbag record (/data_logging_msg만)
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '/data_logging_msg'
        ],
        output='screen'
    )
    actions.append(bag_record)

    # 5) robot_state_publisher (URDF 있으면)
    if os.path.exists(urdf_path):
      rsp_node = Node(
          package='robot_state_publisher',
          executable='robot_state_publisher',
          name='robot_state_publisher',
          parameters=[{'robot_description': open(urdf_path, 'r').read()}],
          output='screen'
      )
      # TF가 먼저 올라와야 RViz가 보기 좋으니까 약간만 딜레이
      actions.append(
          TimerAction(period=1.0, actions=[rsp_node])
      )

    # 6) 네가 만든 rviz_visual 노드
    rviz_visual_node = Node(
        package='flying_pen',
        executable='rviz_visual',
        name='rviz_visual',
        output='screen'
    )
    actions.append(
        TimerAction(period=1.0, actions=[rviz_visual_node])
    )

    # 7) rviz2 실행 (config 있으면 그걸로)
    rviz_args = []
    if os.path.exists(rviz_config):
        rviz_args = ['-d', rviz_config]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen'
    )
    actions.append(
        TimerAction(period=1.5, actions=[rviz_node])
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
