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
COPY tiago_docker/tiago_controller_conf_package.xml /root/ros_ws/src/tiago_robot/tiago_controller_configuration/package.xml

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build -DCATKIN_ENABLE_TESTING=0 -j 1"

# Set up ros_ws
RUN echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc

# Copy the entrypoint.sh script into the image and give execute permissions to the script
COPY tiago_docker/entrypoint.sh /usr/local/bin/
COPY tiago_docker/robot_entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/robot_entrypoint.sh

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
RUN DEBIAN_FRONTEND=noninteractive pip install jpype1
RUN DEBIAN_FRONTEND=noninteractive pip install mpmath
RUN cd ~/ros_ws/IDTxl && pip install .
RUN echo pipp
RUN cd ~/ros_ws/ && \
    git clone -b causal-hri https://github.com/lcastri/fpcmci.git && \
    cd ~/ros_ws/fpcmci && pip install --use-pep517 .

# Clone and install pedsim
RUN cd ~/ros_ws/src/ && \
    git clone https://github.com/srl-freiburg/pedsim_ros.git && \
    cd pedsim_ros && \
    git submodule update --init --recursive
# files has been modified to allow single agent scenario and to expose robot_radius
COPY mytiago_pedsim/modified_scripts/simulator.cpp /root/ros_ws/src/pedsim_ros/pedsim_simulator/src
COPY mytiago_pedsim/modified_scripts/agent.cpp /root/ros_ws/src/pedsim_ros/pedsim_simulator/src/element
COPY mytiago_pedsim/modified_scripts/config.h /root/ros_ws/src/pedsim_ros/pedsim_simulator/include/pedsim_simulator
COPY mytiago_pedsim/modified_scripts/keyboard_teleop.py /root/ros_ws/src/pedsim_ros/pedsim_simulator/scripts
COPY mytiago_pedsim/modified_scripts/simulate_diff_drive_ped.cpp /root/ros_ws/src/pedsim_ros/pedsim_simulator/src
COPY mytiago_pedsim/modified_scripts/CMakeLists.txt /root/ros_ws/src/pedsim_ros/pedsim_simulator/
COPY mytiago_pedsim/modified_scripts/spawn_pedsim_agents.py /root/ros_ws/src/pedsim_ros/pedsim_gazebo_plugin/scripts
COPY mytiago_pedsim/modified_scripts/actor_model.sdf /root/ros_ws/src/pedsim_ros/pedsim_gazebo_plugin/models

# Copy files into the image
RUN mkdir -p ~/ros_ws/src/myTIAGo/
COPY mytiago_tmule /root/ros_ws/src/myTIAGo/mytiago_tmule
COPY mytiago_rviz /root/ros_ws/src/myTIAGo/mytiago_rviz
COPY mytiago_gazebo /root/ros_ws/src/myTIAGo/mytiago_gazebo
COPY mytiago_gazebo/modified_scripts/local_planner.yaml /root/ros_ws/src/pal_navigation_cfg_public/pal_navigation_cfg_tiago/config/base/teb
COPY mytiago_plans /root/ros_ws/src/myTIAGo/mytiago_plans
COPY mytiago_actions /root/ros_ws/src/myTIAGo/mytiago_actions
COPY mytiago_conditions /root/ros_ws/src/myTIAGo/mytiago_conditions
COPY mytiago_twist_mux /root/ros_ws/src/myTIAGo/mytiago_twist_mux
COPY scripts /root/ros_ws/src/myTIAGo/scripts
COPY mytiago_pedsim /root/ros_ws/src/myTIAGo/mytiago_pedsim
COPY mytiago_pedsim/scenarios /root/ros_ws/src/pedsim_ros/pedsim_simulator/scenarios
COPY mytiago_pedsim/launch/mysimulator.launch /root/ros_ws/src/pedsim_ros/pedsim_simulator/launch/
COPY mytiago_pedsim/worlds /root/ros_ws/src/pedsim_ros/pedsim_gazebo_plugin/worlds
COPY mytiago_pedsim/launch/gazebo_plugin.launch /root/ros_ws/src/pedsim_ros/pedsim_gazebo_plugin/launch/
COPY mytiago_recording /root/ros_ws/src/myTIAGo/mytiago_recording
COPY mytiago_twist_mux/modified_scripts/twist_mux.launch /opt/ros/noetic/share/twist_mux/launch
COPY mytiago_twist_mux/modified_scripts/twist_mux.launch /opt/ros/noetic/share/twist_mux/launch
COPY mytiago_causal_discovery /root/ros_ws/src/myTIAGo/mytiago_causal_discovery

# Copy maps into the image
RUN mkdir -p ~/.pal/tiago_maps/configurations/
COPY mytiago_gazebo/tiago_maps/ /root/.pal/tiago_maps/configurations/

# Create .gazebo folders
RUN mkdir -p ~/.gazebo/models
COPY mytiago_gazebo/models /root/.gazebo/models
RUN echo "export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc

# Create a shared folder with the host machine
RUN mkdir -p ~/shared


# Set the entrypoint to run the script
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]