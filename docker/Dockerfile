FROM ubuntu:20.04

# Arguments
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install common dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python-is-python3 \
    git \
    wget \
    curl \
    gnupg2 \
    locales \
    lsb-release \
    build-essential \
    software-properties-common \
    libeigen3-dev && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Install catkin_tools
RUN pip3 install -U catkin_tools

# --------------------------
# Install ROS1 Noetic
# --------------------------
RUN add-apt-repository universe && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    ros-noetic-cmake-modules \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    libyaml-cpp-dev \ 
    protobuf-compiler \ 
    autoconf

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# --------------------------
# Install ROS2 Foxy
# --------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    python3-colcon-common-extensions

# --------------------------
# Aliases and Environment Setup
# --------------------------
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "alias noetic='source /opt/ros/noetic/setup.bash'" >> ~/.bashrc && \
    echo "alias foxy='source /opt/ros/foxy/setup.bash'" >> ~/.bashrc

# --------------------------
# Create Workspaces
# --------------------------
RUN mkdir -p /root/voxblox_ws/ros1_ws/src
RUN mkdir -p /root/voxblox_ws/ros_bridge_ws/src

# Clone your ROS1 package
WORKDIR /root/voxblox_ws/ros1_ws
RUN catkin init
RUN catkin config --extend /opt/ros/noetic
RUN catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN catkin config --merge-devel
WORKDIR /root/voxblox_ws/ros1_ws/src
RUN git clone https://github.com/snt-arg/mav_voxblox_planning.git
RUN wstool init . ./mav_voxblox_planning/install/install_https.rosinstall
RUN wstool update

# Clone missing protobuf_catkin dependency
RUN git clone https://github.com/ethz-asl/protobuf_catkin.git

WORKDIR /root/voxblox_ws/ros1_ws
# Build protobuf_catkin first, then the main package
RUN catkin build protobuf_catkin
RUN catkin build mav_voxblox_planning

# Clone ros1_bridge
WORKDIR /root/voxblox_ws/ros_bridge_ws
RUN git clone -b foxy https://github.com/ros2/ros1_bridge src/ros1_bridge

# --------------------------
# Build ros1_bridge
# --------------------------
SHELL ["/bin/bash", "-i", "-c"]
RUN noetic && foxy && \
    colcon build --packages-select ros1_bridge --cmake-force-configure

# --------------------------
# Build ROS1 workspace
# --------------------------
WORKDIR /root/voxblox_ws/ros1_ws
RUN noetic && \
    rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys="numpy_eigen grpc yaml_cpp_catkin catkin_boost_python_buildtool" && \
    catkin config --blacklist minkindr_python && \
    catkin build

# --------------------------
# Source everything at startup
# --------------------------
WORKDIR /root/voxblox_ws
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc && \
    echo "source /root/voxblox_ws/ros1_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "source /root/voxblox_ws/ros_bridge_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
