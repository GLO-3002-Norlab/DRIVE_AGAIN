FROM osrf/ros:humble-desktop
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

# Updating and installing deps
RUN apt-get update -y && apt-get install -y python3-pip
RUN pip install --upgrade pip setuptools wheel

# Installing python module
RUN mkdir -p /home/root/DRIVE_AGAIN/src
WORKDIR /home/root/DRIVE_AGAIN

COPY pyproject.toml .
RUN pip install -e .
COPY src/ src/

# Installing ros workspace
RUN mkdir -p /home/root/ros2_ws/src
WORKDIR /home/root/ros2_ws

COPY ros/ src/
RUN rosdep install --from-paths src -y --ignore-src --rosdistro ${ROS_DISTRO}
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1

COPY entrypoint.bash /
ENTRYPOINT [ "/entrypoint.bash" ]