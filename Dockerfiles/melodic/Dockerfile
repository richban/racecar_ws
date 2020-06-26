FROM dorowu/ubuntu-desktop-lxde-vnc
LABEL maintainer "r.banyi@gmail.com"

#Install dirmngr
RUN apt-get update && apt-get install -y dirmngr

# Adding keys for ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Installing ROS
RUN apt-get update && apt-get install -y ros-melodic-desktop-full \
		wget git nano
RUN apt-get install python-rosdep
RUN rosdep init && rosdep update

# Update Gazebo 9
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt-get update && apt-get install -y gazebo9 libignition-math2-dev

RUN apt-get install -y \
    ros-melodic-gazebo-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-tf2-geometry-msgs \
    ros-melodic-ackermann-msgs \
    ros-melodic-joy \
    ros-melodic-map-server \
    build-essential

# Setup workspace
ENV SIM_WS /home/ubuntu/racecar_ws
RUN mkdir -p $SIM_WS
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash; cd $SIM_WS; catkin_make;'

# Installing Atom Editor
RUN add-apt-repository ppa:webupd8team/atom && apt update && apt install -y atom

RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc && source /root/.bashrc"
