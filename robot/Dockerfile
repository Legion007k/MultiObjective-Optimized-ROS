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

# Camara Orbbec Astra S. Va en capas separadas despues de compilar el Kobuki para
# que Docker reutilice esa compilacion y solo se agregue el driver de la camara.
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config libusb-1.0-0-dev libudev-dev libdw-dev libeigen3-dev libjpeg-dev \
    python-numpy python-opencv \
    ros-melodic-backward-ros ros-melodic-cv-bridge ros-melodic-image-geometry \
    ros-melodic-image-transport ros-melodic-camera-info-manager \
    ros-melodic-dynamic-reconfigure ros-melodic-tf2-ros ros-melodic-rgbd-launch \
 && rm -rf /var/lib/apt/lists/*

# libuvc fijo en v0.0.7: desde v0.0.9 pide CMake 3.13 y Ubuntu 18.04 trae 3.10
WORKDIR /opt
RUN git clone --depth 1 --branch v0.0.7 https://github.com/libuvc/libuvc.git \
 && mkdir libuvc/build && cd libuvc/build \
 && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4 && make install && ldconfig \
 && rm -rf /opt/libuvc

# Driver fijado a un commit para que la imagen sea reproducible
WORKDIR /opt/robot_deps_ws/src
RUN git clone https://github.com/orbbec/ros_astra_camera.git \
 && git -C ros_astra_camera checkout 210f530144314da7c70072215bacba12130d9b20

WORKDIR /opt/robot_deps_ws
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make -j4"

RUN echo "source /opt/robot_deps_ws/devel/setup.bash" >> /root/.bashrc \
 && echo "[ -f /root/ros_ws/devel/setup.bash ] && source /root/ros_ws/devel/setup.bash" >> /root/.bashrc

# El codigo se monta como volumen desde docker-compose.jetson.yml
WORKDIR /root/ros_ws
