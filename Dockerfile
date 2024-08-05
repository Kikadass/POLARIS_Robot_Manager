FROM osrf/ros:noetic-desktop-full

# Dependencies from POLARIS_GEM_e2 README.md
RUN apt-get update && \
    apt-get install -y git \
                       bash \
                       vim \
                       ros-noetic-ackermann-msgs \
                       ros-noetic-geometry2 \
                       ros-noetic-hector-gazebo \
                       ros-noetic-hector-models \
                       ros-noetic-jsk-rviz-plugins \
                       ros-noetic-ros-control \
                       ros-noetic-ros-controllers \
                       ros-noetic-velodyne-simulator

# HOME can be overwritten by --build-arg HOME=$HOME in the docker build command
ARG HOME=/root

# Clone the POLARIS_GEM_e2 in the expected directory
RUN mkdir -p ${HOME}/gem_ws/src 
WORKDIR ${HOME}/gem_ws/src
RUN git clone https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git

# Compile simulator
WORKDIR ${HOME}/gem_ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash && catkin_make

# Add setup to .bashrc to simplify usage 
RUN echo "source  /opt/ros/noetic/setup.bash" >>  ${HOME}/.bashrc
RUN echo "source ${HOME}/gem_ws/devel/setup.bash" >>  ${HOME}/.bashrc
RUN echo "alias launchGazeboWithGem='roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:=\"true\"'" >>  ${HOME}/.bashrc
RUN echo "alias launchRobotStatusManager='roslaunch robot_status_manager robot_status_manager.launch'" >>  ${HOME}/.bashrc

RUN echo "alias displaySensorInfo='roslaunch gem_gazebo gem_sensor_info.launch'" >>  ${HOME}/.bashrc
RUN echo "alias moveAroundTheMap='rosrun gem_pure_pursuit_sim pure_pursuit_sim.py'" >>  ${HOME}/.bashrc
RUN echo "alias goToProject='cd ${HOME}/gem_ws/src/POLARIS_Robot_Manager'" >>  ${HOME}/.bashrc

# Setup simple track environment
CMD source devel/setup.bash && roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"
