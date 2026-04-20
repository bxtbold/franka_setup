FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    build-essential \
    cmake \
    pkg-config \
    libeigen3-dev \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list' \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base \
    ros-noetic-rviz \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    python3-catkin-tools \
    ros-noetic-combined-robot-hw \
    ros-noetic-control-msgs \
    ros-noetic-actionlib \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    ros-noetic-controller-interface \
    ros-noetic-hardware-interface \
    ros-noetic-transmission-interface \
    ros-noetic-realtime-tools \
    ros-noetic-control-toolbox \
    ros-noetic-angles \
    ros-noetic-urdf \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    ros-noetic-tf2-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-srvs \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-eigen-conversions \
    ros-noetic-kdl-parser \
    ros-noetic-pluginlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-xacro \
    ros-noetic-joint-limits-interface \
    ros-noetic-joint-state-controller \
    ros-noetic-controller-manager \
    ros-noetic-rqt-reconfigure \
    ros-noetic-rqt-controller-manager \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/opt/franka_setup/ros_ws

RUN mkdir -p ${CATKIN_WS}/src
WORKDIR ${CATKIN_WS}/src

ARG LIBFRANKA_REF=0.8.0
ARG FRANKA_ROS_REF=0.8.0
ARG SERL_FRANKA_CONTROLLERS_REF=main
RUN git clone --depth 1 --branch ${FRANKA_ROS_REF} https://github.com/frankaemika/franka_ros.git
RUN git clone --depth 1 --branch ${SERL_FRANKA_CONTROLLERS_REF} https://github.com/rail-berkeley/serl_franka_controllers.git
RUN touch franka_ros/franka_gazebo/CATKIN_IGNORE \
    franka_ros/franka_example_controllers/CATKIN_IGNORE \
    franka_ros/franka_visualization/CATKIN_IGNORE

WORKDIR /tmp
RUN git clone --recursive --depth 1 --branch ${LIBFRANKA_REF} https://github.com/frankaemika/libfranka.git \
    && cd libfranka \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF .. \
    && cmake --build . --parallel \
    && cmake --install . \
    && ldconfig

WORKDIR ${CATKIN_WS}
RUN rosdep init || true && rosdep update
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic --skip-keys=libfranka"
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make --pkg franka_description franka_msgs franka_hw franka_control franka_gripper serl_franka_controllers"

# Ignore realtime requirement for systems without realtime kernel.
RUN if [ -f /opt/ros/noetic/share/franka_control/config/franka_control_node.yaml ]; then \
      sed -i 's/^realtime_config:.*/realtime_config: ignore/' /opt/ros/noetic/share/franka_control/config/franka_control_node.yaml; \
    fi
RUN if [ -f ${CATKIN_WS}/src/franka_ros/franka_control/config/franka_control_node.yaml ]; then \
      sed -i 's/^realtime_config:.*/realtime_config: ignore/' ${CATKIN_WS}/src/franka_ros/franka_control/config/franka_control_node.yaml; \
    fi

WORKDIR /opt/franka_setup
COPY python/franka_bridge /opt/franka_setup/python/franka_bridge
COPY python/franka_bridge_client /opt/franka_setup/python/franka_bridge_client
COPY scripts/start_robot_stack.sh /opt/franka_setup/scripts/start_robot_stack.sh
COPY scripts/set_collision_behavior.sh /opt/franka_setup/scripts/set_collision_behavior.sh

RUN python3 -m pip install --no-cache-dir --upgrade pip \
    && python3 -m pip install --no-cache-dir \
      pyzmq \
      msgpack \
    && python3 -m pip install --no-cache-dir -e /opt/franka_setup/python/franka_bridge \
    && python3 -m pip install --no-cache-dir -e /opt/franka_setup/python/franka_bridge_client \
    && chmod +x /opt/franka_setup/scripts/start_robot_stack.sh /opt/franka_setup/scripts/set_collision_behavior.sh

RUN /bin/bash -lc "touch /root/.bashrc \
    && grep -qxF 'source /opt/ros/noetic/setup.bash' /root/.bashrc \
      || echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
    && grep -qxF 'if [ -f /opt/franka_setup/ros_ws/devel/setup.bash ]; then source /opt/franka_setup/ros_ws/devel/setup.bash; fi' /root/.bashrc \
      || echo 'if [ -f /opt/franka_setup/ros_ws/devel/setup.bash ]; then source /opt/franka_setup/ros_ws/devel/setup.bash; fi' >> /root/.bashrc"

ENV FRANKA_CMD_PORT=5555
ENV FRANKA_STATE_PORT=5556
ENV FRANKA_GRIPPER_CMD_PORT=5557
ENV FRANKA_GRIPPER_STATE_PORT=5558
ENV FRANKA_STATE_HZ=60
ENV FRANKA_GRIPPER_STATE_HZ=30

CMD ["/opt/franka_setup/scripts/start_robot_stack.sh"]
