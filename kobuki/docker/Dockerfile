ARG base_img

FROM ${base_img}
MAINTAINER Nikolay Dema <ndema2301@gmail.com>

### ---------------------------------------------------------------------- #
RUN apt update && apt install python3-pip tmux -y
RUN pip3 install opencv-python opencv-contrib-python scipy
RUN apt install ros-$ROS_DISTRO-domain-bridge -y
RUN apt install nano vim -y