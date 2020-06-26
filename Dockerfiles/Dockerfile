# Kudos to DOROWU and AHMET
FROM dorowu/ubuntu-desktop-lxde-vnc
LABEL maintainer "r.banyi@gmail.com"

# Install dirmngr
RUN apt-get update && apt-get install -y dirmngr

# Adding keys for ROS

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

#Set the ROS distro
ENV ROS_DISTRO kinetic

# Adding keys for ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installing ROS
RUN apt-get update && apt-get install -y ros-kinetic-desktop-full \
		wget git nano
RUN rosdep init && rosdep update

# Update Gazebo 7
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y gazebo7 libignition-math2-dev

RUN apt-get install -y ros-kinetic-gazebo-ros-control 
RUN apt-get install -y ros-kinetic-ros-controllers

# Install the racecar simulator
ENV SIM_WS /opt/ros/sim_ws
RUN mkdir -p $SIM_WS/src
RUN git clone https://github.com/mit-racecar/racecar_simulator.git
RUN mv racecar_simulator $SIM_WS/src
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; cd $SIM_WS; catkin_make;'

# Make a racecar workspace chained to the sim repo
RUN mkdir -p /racecar_ws/src
RUN /bin/bash -c 'source $SIM_WS/devel/setup.bash; cd racecar_ws; catkin_make;'

# ROS Gazebo
RUN git clone https://github.com/mit-racecar/racecar_gazebo.git
RUN mv racecar_gazebo $SIM_WS/src
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; cd $SIM_WS; catkin_make;'

# Installing Atom Editor
RUN add-apt-repository ppa:webupd8team/atom && apt update && apt install -y atom

RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc && source /root/.bashrc"
