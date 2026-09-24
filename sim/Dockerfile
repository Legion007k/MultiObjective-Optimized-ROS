FROM osrf/ros:melodic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Arreglo de la llave de ROS: la original expiró en 2025
RUN rm -f /etc/apt/sources.list.d/ros*.list \
 && apt-get update && apt-get install -y curl gnupg \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu bionic main" \
    > /etc/apt/sources.list.d/ros1.list

# Dependencias que sí existen como paquete instalable
RUN apt-get update && apt-get install -y \
    git nano pyqt5-dev-tools \
    ros-melodic-kobuki-* \
    ros-melodic-ecl-streams \
    ros-melodic-depthimage-to-laserscan \
    ros-melodic-joy \
    ros-melodic-yocs-velocity-smoother \
    ros-melodic-rosbridge-server \
 && rm -rf /var/lib/apt/lists/*

# Código fuente del TurtleBot 2 (no existe como paquete para Melodic)
WORKDIR /opt/tb2_ws/src
RUN git clone https://github.com/turtlebot/turtlebot.git \
 && git clone https://github.com/turtlebot/turtlebot_msgs.git \
 && git clone https://github.com/turtlebot/turtlebot_apps.git \
 && git clone https://github.com/turtlebot/turtlebot_simulator.git \
 && git clone https://github.com/yujinrobot/kobuki_msgs.git \
 && git clone --single-branch --branch melodic https://github.com/yujinrobot/kobuki.git \
 && mv kobuki/kobuki_description kobuki/kobuki_node kobuki/kobuki_keyop \
       kobuki/kobuki_safety_controller kobuki/kobuki_bumper2pc ./ \
 && rm -rf kobuki \
 && git clone --single-branch --branch melodic https://github.com/yujinrobot/kobuki_desktop.git \
 && mv kobuki_desktop/kobuki_gazebo_plugins ./ \
 && rm -rf kobuki_desktop \
 && git clone https://github.com/yujinrobot/yujin_ocs.git \
 && mv yujin_ocs/yocs_cmd_vel_mux yujin_ocs/yocs_controllers ./ \
 && rm -rf yujin_ocs

# Instalar lo que falte y compilar
WORKDIR /opt/tb2_ws
RUN apt-get update \
 && rosdep update --include-eol-distros \
 && rosdep install --from-paths src --ignore-src -r -y \
 && /bin/bash -c "source /opt/ros/melodic/setup.bash && catkin_make" \
 && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/tb2_ws/devel/setup.bash" >> /root/.bashrc \
 && echo "[ -f /root/ros_ws/devel/setup.bash ] && source /root/ros_ws/devel/setup.bash" >> /root/.bashrc