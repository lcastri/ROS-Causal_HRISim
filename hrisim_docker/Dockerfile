FROM nvidia/cudagl:11.4.2-base-ubuntu20.04

# Set the timezone
ENV TZ=Europe/London
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install essential packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \ 
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    locales lsb-release build-essential software-properties-common gnupg curl wget \
    apt-transport-https git tmux terminator nano htop net-tools iputils-ping sshpass iproute2 \
    xserver-xorg-video-dummy x11-* xinit x11vnc xterm xvfb xauth xorg \
    && rm -rf /var/lib/apt/lists/*

# Install ROS
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-catkin ros-noetic-desktop-full ros-noetic-vision-msgs ros-noetic-cv-bridge ros-noetic-rosbridge-server \
    python3-catkin-tools python3-venv python3-pip python3-rosdep ipython3 python3-rosinstall python-is-python3 \
    && rosdep init && rosdep update

# Create a new workspace
RUN mkdir -p ~/ros_ws
WORKDIR /root/ros_ws

# Download tiago_public-noetic.rosinstall and clone required repositories
RUN wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall && \
    rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall

# Initialize rosdep and update
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init && rosdep update

# Install dependencies
RUN DEBIAN_FRONTEND=noninteractive rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys \
    "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver ros-noetic-control-msgs"

# Fix tiago_controller_configuration package.xml
COPY hrisim_docker/tiago_controller_conf_package.xml /root/ros_ws/src/tiago_robot/tiago_controller_configuration/package.xml

# Install tmux and TMuLE
RUN DEBIAN_FRONTEND=noninteractive pip install tmule

# Install gazebo dependencies for animated actors
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN DEBIAN_FRONTEND=noninteractive apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-pkgs
    
# Clone and install PetriNetPlans
RUN cd ~/ros_ws
RUN git clone -b noetic_devel https://github.com/francescodelduchetto/PetriNetPlans.git
RUN DEBIAN_FRONTEND=noninteractive apt install -y flex libfl-dev libfl2
RUN mkdir -p ~/ros_ws/PetriNetPlans/PNP/build && cd ~/ros_ws/PetriNetPlans/PNP/build && cmake .. && \
    make install

RUN cd ~/ros_ws/src && \
    ln -s ~/ros_ws/PetriNetPlans/PNPros/ROS_bridge/pnp_ros . && \
    ln -s ~/ros_ws/PetriNetPlans/PNPros/ROS_bridge/pnp_msgs .
RUN echo "export PNP_HOME=~/ros_ws/src/pnp_ros/" >> ~/.bashrc

# install fpcmci
RUN DEBIAN_FRONTEND=noninteractive pip install --upgrade setuptools wheel
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \ 
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends default-jdk
ENV JAVA_HOME "/lib/jvm/java-11-openjdk-amd64/bin/java"
RUN cd ~/ros_ws && \
    git clone -b v1.4 https://github.com/pwollstadt/IDTxl.git
RUN cd ~/ros_ws/IDTxl && pip install .
RUN DEBIAN_FRONTEND=noninteractive pip install fpcmci --use-pep517

# Installing pkgs for postprocessing
RUN DEBIAN_FRONTEND=noninteractive pip install shapely

# Clone and install pedsim
RUN cd ~/ros_ws/src/ && \
    git clone https://github.com/srl-freiburg/pedsim_ros.git && \
    cd pedsim_ros && \
    git submodule update --init --recursive

# Clone and install roscausal
# RUN echo "ahahaha"
# RUN cd ~/ros_ws/src/ && \
#     git clone -b dev https://github.com/lcastri/roscausal.git
RUN rm -rf ~/ros_ws/src/roscausal && \
    cd ~/ros_ws/src/ && \
    git clone -b dev https://github.com/lcastri/roscausal.git

# Copy the entrypoint.sh script into the image and give execute permissions to the script
COPY hrisim_docker/entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh
COPY hrisim_docker/robot_entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/robot_entrypoint.sh

# files has been modified to allow single agent scenario and to expose robot_radius
COPY HRISim/hrisim_pedsim/modified_scripts/simulator.cpp /root/ros_ws/src/pedsim_ros/pedsim_simulator/src
COPY HRISim/hrisim_pedsim/modified_scripts/agent.cpp /root/ros_ws/src/pedsim_ros/pedsim_simulator/src/element
COPY HRISim/hrisim_pedsim/modified_scripts/config.h /root/ros_ws/src/pedsim_ros/pedsim_simulator/include/pedsim_simulator
COPY HRISim/hrisim_pedsim/modified_scripts/keyboard_teleop.py /root/ros_ws/src/pedsim_ros/pedsim_simulator/scripts
COPY HRISim/hrisim_pedsim/modified_scripts/simulate_diff_drive_ped.cpp /root/ros_ws/src/pedsim_ros/pedsim_simulator/src
COPY HRISim/hrisim_pedsim/modified_scripts/CMakeLists.txt /root/ros_ws/src/pedsim_ros/pedsim_simulator/
COPY HRISim/hrisim_pedsim/modified_scripts/spawn_pedsim_agents.py /root/ros_ws/src/pedsim_ros/pedsim_gazebo_plugin/scripts
COPY HRISim/hrisim_pedsim/modified_scripts/actor_model.sdf /root/ros_ws/src/pedsim_ros/pedsim_gazebo_plugin/models
COPY HRISim/hrisim_pedsim/modified_scripts/AgentState.msg /root/ros_ws/src/pedsim_ros/pedsim_msgs/msg

# Copy files into the image
RUN mkdir -p ~/ros_ws/src/HRISim/
COPY HRISim /root/ros_ws/src/HRISim
COPY HRISim/hrisim_gazebo/modified_scripts/local_planner.yaml /root/ros_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_tiago/config/base/teb
COPY HRISim/hrisim_pedsim/scenarios /root/ros_ws/src/pedsim_ros/pedsim_simulator/scenarios
COPY HRISim/hrisim_pedsim/launch/mysimulator.launch /root/ros_ws/src/pedsim_ros/pedsim_simulator/launch/
COPY HRISim/hrisim_pedsim/worlds /root/ros_ws/src/pedsim_ros/pedsim_gazebo_plugin/worlds
COPY HRISim/hrisim_pedsim/launch/gazebo_plugin.launch /root/ros_ws/src/pedsim_ros/pedsim_gazebo_plugin/launch/
COPY HRISim/hrisim_twist_mux/modified_scripts/twist_mux.launch /opt/ros/noetic/share/twist_mux/launch

# Copy maps into the image
RUN mkdir -p ~/.pal/tiago_maps/configurations/
COPY HRISim/hrisim_gazebo/tiago_maps/ /root/.pal/tiago_maps/configurations/

# Create .gazebo folders
RUN mkdir -p ~/.gazebo/models
COPY HRISim/hrisim_gazebo/models /root/.gazebo/models
RUN echo "export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc

# Create a shared folder with the host machine
RUN mkdir -p ~/shared

# Set the entrypoint to run the script
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]