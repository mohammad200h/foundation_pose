
FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu22.04

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update
RUN apt-get install -y software-properties-common

RUN add-apt-repository universe

##### python3.11 ######
RUN apt-get update
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get install -y python3.11
RUN apt-get install -y python3-pip
###### ROS2 Jazzy #######
RUN locale

RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN locale

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update &&  apt install -y ros-dev-tools

RUN apt update

RUN apt upgrade -y

RUN apt install -y --fix-missing ros-humble-desktop

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && echo $ROS_DISTRO"
###### Additional ROS2 Packages ########
RUN apt-get update && apt-get install -y ros-humble-ament-cmake

#####Installing terminator ######
RUN apt-get update

#RUN apt-get update && apt-get install -y terminator --fix-missing

#RUN mkdir /etc/xdg/terminator/
##########Setting up Sudo #######

ARG UNAME=ubuntu
ARG UID=1000
ARG GID=1000
ARG PASS=ros

RUN apt-get update && apt-get install sudo -y
RUN sudo -i

RUN groupadd -g $GID -o $UNAME
RUN sudo useradd -m -u $UID -g $GID -p $(openssl passwd -1 $PASS) -o -s  /bin/bash $UNAME
RUN usermod -aG sudo $UNAME

###### WorkDir #######

USER $UNAME
WORKDIR home/$UNAME/

###### Shell #######
CMD /bin/bash
