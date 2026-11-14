# ROS2 Humble + TurtleBot3 Gazebo 시뮬레이션 + 브릿지
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-turtlebot3-gazebo \
        ros-humble-turtlebot3-description \
    && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=burger
WORKDIR /ws
COPY bridge ./bridge
COPY launch ./launch

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /usr/share/gazebo/setup.sh && ros2 launch /ws/launch/sim_with_bridge.launch.py"]
