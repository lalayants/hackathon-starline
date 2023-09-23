FROM nickodema/hsl_2022:nvidia

RUN apt update && \
    apt install ros-humble-gazebo-ros-pkgs ros-humble-cartographer ros-humble-cartographer-ros ros-humble-navigation2 ros-humble-nav2-bringup -y && \
    apt install ros-humble-turtlebot3-msgs ros-humble-dynamixel-sdk ros-humble-hls-lfcd-lds-driver -y

RUN apt install python3-pip -y && pip3 install pandas
RUN apt install tmux git vim -y
WORKDIR /workspace