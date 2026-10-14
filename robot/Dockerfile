FROM ros:melodic-ros-base-bionic

ENV DEBIAN_FRONTEND=noninteractive

# Arreglo de la llave de ROS: la original expiro en 2025 (igual que en sim/Dockerfile)
RUN rm -f /etc/apt/sources.list.d/ros*.list \
 && apt-get update && apt-get install -y curl gnupg \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" \
    > /etc/apt/sources.list.d/ros1.list

# Drivers del robot real: base Kobuki, RPLIDAR y rosbridge para el cerebro
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-kobuki-node \
    ros-melodic-rplidar-ros \
    ros-melodic-rosbridge-server \
 && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc

# El codigo se monta como volumen desde docker-compose.jetson.yml
WORKDIR /root/ros_ws
