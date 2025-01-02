
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

###### Installing package dependencies ######
RUN pip install torch==2.5.1 torchaudio==2.5.1 torchvision==0.20.1

RUN pip install aiohttp==3.10.10 albumentations==1.4.18 cv_bridge
RUN pip install Cython==3.0.10 cycler dearpygui==1.11.1
RUN pip install einops==0.8.0 glfw==2.7.0 gdown
RUN pip install h5py hickle==5.0.3 imageio==2.35.1
RUN pip install joblib==1.4.2 kornia==0.5.10 kiwisolver
RUN pip install loguru==0.7.2 matplotlib==3.9.2 Ninja
RUN pip install "numpy<2" omegaconf==2.3.0 open3d==0.18.0
RUN pip install openai==1.51.2 opencv_contrib_python==4.10.0.84 opencv_python==4.10.0.82
RUN pip install opencv_python_headless==4.10.0.84 pandas Pillow==11.0.0
RUN pip install progressbar33==2.4 psutil PyAutoGUI==0.9.54
RUN pip install PyOpenGL==3.1.0 pyparsing pyrender==0.1.45
RUN pip install PySide6==6.8.0.1 pytest==8.2.2 pytinyrenderer==0.0.14
RUN pip install pytorch_lightning==2.4.0 PyYAML==6.0.2 Requests==2.32.3
RUN pip install ruamel.base==1.0.0 ruptures==1.1.9 scikit_learn==1.5.2
RUN pip install scipy==1.14.1 setuptools==72.1.0 six
RUN pip install tqdm==4.66.5 transformations trimesh==4.4.7
RUN pip install ultralytics==8.3.31 warp==1.0.4 warp_lang==1.3.1
RUN pip install websocket_client==1.8.0 yacs==0.1.8
RUN pip install "git+https://github.com/facebookresearch/pytorch3d.git@stable"
RUN pip install pybind11==2.10.1
RUN pip install trimesh
RUN pip install imageio
RUN pip install joblib
RUN pip install pandas
pip install kornia
pip install omegaconf
pip install h5py
pip install ultralytics
pip install ninja
sudo apt install ninja-build


#####Installing terminator ######
RUN apt-get update

RUN  apt-get install -y terminator

RUN mkdir /etc/xdg/terminator/
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
