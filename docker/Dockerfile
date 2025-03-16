#FROM ros:humble-ros-base
FROM osrf/ros:humble-desktop-full

# Switch to cyclonedds
#RUN apt update \
#  && apt install -y \
#  ros-humble-rmw-cyclonedds-cpp
#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#ENV CYCLONEDDS_URI="<CycloneDDS><Domain><General><AllowMulticast>true</AllowMulticast><Interfaces><NetworkInterface address='10.1.1.0' /></Interfaces></General></Domain></CycloneDDS>"
# Increase buffer size as recommended by rmw_cyclonedds_cpp
#RUN echo 'net.core.rmem_max=2147483647' >> /etc/sysctl.conf
#RUN echo 'net.core.rmem_default=8388608' >> /etc/sysctl.conf

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install -y \
  ros-humble-realsense2-* \
  ros-humble-rosbag2 \
  ros-humble-rosbag2-transport \
  ros-humble-image-pipeline

COPY ../src /ros2_ws/src

WORKDIR /ros2_ws

RUN source /opt/ros/humble/setup.bash \
# && rosdep update \
# && rosdep install -i --from-path src --rosdistro humble -y \
 && colcon build \
 && echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo 'source /ros2_ws/install/setup.bash' >> /root/.bashrc \
 && echo 'source /ros2_ws/install/setup.bash' >> /ros_entrypoint.sh

RUN sed -i '/exec "$@"/d' /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]

ENV DEBIAN_FRONTEND=dialog
