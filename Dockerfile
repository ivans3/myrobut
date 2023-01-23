FROM ubuntu:bionic

RUN apt update

RUN apt install -y curl gnupg2 lsb-release locales

RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

RUN apt update

#install tzdata first so it doesnt block
RUN apt-get install -y tzdata
RUN ln -fs /usr/share/zoneinfo/US/Pacific-New /etc/localtime && dpkg-reconfigure -f noninteractive tzdata

RUN apt-get install -y ros-eloquent-ros-base
RUN apt-get update
RUN apt-get install -y python3-colcon-common-extensions git vim python-rosdep

RUN rosdep init
RUN rosdep update --include-eol-distros

#Note: only needed to build interface (`ament_cmake`-type packages...)on armhf-side Only:
RUN git clone https://github.com/eProsima/Fast-CDR.git && mkdir Fast-CDR/build && cd Fast-CDR/build && cmake .. && cmake --build . --target install

#For client-side:
RUN apt-get install -y python3-sdl2

#To use SDL_Image.so instead of PIL for image loading:
RUN apt-get update && apt-get install -y libsdl-image1.2-dev

RUN touch /root/.Xauthority

