FROM osrf/ros:foxy-desktop
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-rmw-cyclonedds-cpp

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp