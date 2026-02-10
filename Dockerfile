FROM osrf/ros:humble-desktop
RUN apt-get update && apt-get install -y vim && apt-get install tree
RUN apt install -y python3-pip
RUN apt-get install dos2unix
RUN pip3 install setuptools==58.2.0
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc