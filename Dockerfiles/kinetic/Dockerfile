FROM dorowu/ubuntu-desktop-lxde-vnc:xenial
LABEL maintainer "rbanyi@gmail.com"

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

RUN apt-get install -y \
    ros-kinetic-gazebo-ros-control \
    ros-kinetic-ros-controllers \
    ros-kinetic-tf2-geometry-msgs \
    ros-kinetic-ackermann-msgs \
    ros-kinetic-joy \
    ros-kinetic-map-server \
    build-essential

# Setup workspace
ENV SIM_WS /home/ubuntu/racecar_ws
RUN mkdir -p $SIM_WS
RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; cd $SIM_WS; catkin_make;'

# Installing Atom Editor
RUN add-apt-repository ppa:webupd8team/atom && apt update && apt install -y atom

RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc && source /root/.bashrc"
