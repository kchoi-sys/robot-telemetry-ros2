"""TurtleBot3 Gazebo 시뮬레이션(화면 없이) + 관제 서버 브릿지를 한 번에 띄우는 launch 파일.

turtlebot3_gazebo의 turtlebot3_world.launch.py와 같은 구성인데, 화면(gzclient)만 뺐다.
로봇 위치는 관제 대시보드의 이동 경로에서 본다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    tb3 = get_package_share_directory("turtlebot3_gazebo")
    gazebo_ros = get_package_share_directory("gazebo_ros")
    tb3_launch = os.path.join(tb3, "launch")
    world = os.path.join(tb3, "worlds", "turtlebot3_world.world")
    server_url = os.environ.get("SERVER_URL", "http://localhost:8000")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_launch, "robot_state_publisher.launch.py")),
            launch_arguments={"use_sim_time": "true"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_launch, "spawn_turtlebot3.launch.py")),
            launch_arguments={"x_pose": "-2.0", "y_pose": "-0.5"}.items()),
        # 로봇이 생성될 시간을 잠깐 준 뒤 브릿지 시작
        TimerAction(period=5.0, actions=[ExecuteProcess(
            cmd=["python3", "/ws/bridge/ros_bridge.py", "--ros-args",
                 "-p", "robot_id:=TB3-01", "-p", "source:=sim", "-p", f"server_url:={server_url}", "-p", "upload_period_s:=0.2"],
            output="screen")]),
    ])
