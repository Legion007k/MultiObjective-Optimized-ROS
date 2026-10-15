FROM ros:melodic-ros-base-bionic

ENV DEBIAN_FRONTEND=noninteractive

# Arreglo de la llave de ROS: la original expiro en 2025 (igual que en sim/Dockerfile)
RUN rm -f /etc/apt/sources.list.d/ros*.list \
 && apt-get update && apt-get install -y curl gnupg \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" \
    > /etc/apt/sources.list.d/ros1.list

# Dependencias que si existen como paquete instalable
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential \
    ros-melodic-rplidar-ros \
    ros-melodic-rosbridge-server \
 && rm -rf /var/lib/apt/lists/*

# Codigo fuente de la base Kobuki (kobuki_node no existe como paquete para Melodic)
WORKDIR /opt/robot_deps_ws/src
RUN git clone https://github.com/yujinrobot/kobuki_msgs.git \
 && git clone --single-branch --branch melodic https://github.com/yujinrobot/kobuki.git \
 && mv kobuki/kobuki_description kobuki/kobuki_node kobuki/kobuki_keyop \
       kobuki/kobuki_safety_controller kobuki/kobuki_bumper2pc ./ \
 && rm -rf kobuki \
 && git clone --single-branch --branch melodic https://github.com/yujinrobot/kobuki_core.git \
 && git clone https://github.com/yujinrobot/yujin_ocs.git \
 && mv yujin_ocs/yocs_cmd_vel_mux yujin_ocs/yocs_controllers ./ \
 && rm -rf yujin_ocs

# Instalar lo que falte y compilar
WORKDIR /opt/robot_deps_ws
RUN apt-get update \
 && rosdep update --include-eol-distros \
 && rosdep install --from-paths src --ignore-src -r -y \
 && /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make" \
 && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/robot_deps_ws/devel/setup.bash" >> /root/.bashrc \
 && echo "[ -f /root/ros_ws/devel/setup.bash ] && source /root/ros_ws/devel/setup.bash" >> /root/.bashrc

# El codigo se monta como volumen desde docker-compose.jetson.yml
WORKDIR /root/ros_ws
