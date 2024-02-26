FROM nvidia/cudagl:11.3.0-devel-ubuntu20.04

ENV DEBIAN_FRONTEND="noninteractive"
ENV QT_X11_NO_MITSHM=1

# Setup basic packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    curl \
    vim \
    nano \
    lsb-release \
    ca-certificates \
    libjpeg-dev \
    libpng-dev \
    libglfw3-dev \
    libglm-dev \
    libx11-dev \
    libomp-dev \
    libegl1-mesa-dev \
    pkg-config \
    wget \
    zip \
    pip \
    unzip &&\
    rm -rf /var/lib/apt/lists/*

# Install ROS Noetic
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros-focal.list > /dev/null && \
    apt update && \
    apt install -y ros-noetic-desktop && \
        /bin/bash -c "source /opt/ros/noetic/setup.bash"
    
RUN apt-key adv --fetch-keys 'http://packages.osrfoundation.org/gazebo.key' && \
        apt update && \
        apt install -y python3-rosdep

RUN if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then \
        rosdep init; \
    fi

WORKDIR /opt/ros/noetic
RUN rosdep update

WORKDIR /opt/

# Install libusb without udev to avoid conflicts with docker
RUN apt-get remove -y libusb-1.0-0-dev
RUN /bin/bash -c  "wget https://github.com/libusb/libusb/releases/download/v1.0.24/libusb-1.0.24.tar.bz2 &&\
tar xf libusb-1.0.24.tar.bz2 &&\
cd libusb-1.0.24 &&\
./configure --disable-udev --prefix='$PWD/install_dir' &&\
make -j &&\
make install"
ENV LD_LIBRARY_PATH="/opt/install_dir/lib":${LD_LIBRARY_PATH}

# Install Turtlebot drivers
RUN apt-get update && apt-get install -y --no-install-recommends \
    liborocos-kdl-dev \
    ros-noetic-ecl-* \
    ros-noetic-joy \
    libeigen3-dev \
    python3-opencv \
    libftdi-dev &&\
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN /bin/bash -c 'wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \ 
		  apt-get update && apt-get install -y libgazebo11 gazebo11-common gazebo11 ros-noetic-gazebo-ros ros-noetic-gazebo-dev ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control'
    
RUN /bin/bash -c 'mkdir -p /opt/turtlebot_ws/src && \
		  cd /opt/turtlebot_ws/ && \
		  source /opt/ros/noetic/setup.bash && \
		  catkin_make && \
		  cd src/ && \
		  git clone https://github.com/yujinrobot/kobuki.git && \
		  git clone https://github.com/yujinrobot/yujin_ocs.git && \
		  git clone https://github.com/yujinrobot/kobuki_msgs.git && \
		  git clone https://github.com/yujinrobot/kobuki_core.git && \
		  cd yujin_ocs && \
		  mkdir save && \
		  mv yocs_cmd_vel_mux save && \
		  mv yocs_controllers save && \
		  mv yocs_velocity_smoother save && \
		  rm -rf yocs* && \
		  cd save  && \
		  mv * .. && \
		  cd .. && rmdir save && \
		  cd /opt/turtlebot_ws && \
		  rosdep install --from-paths src --ignore-src -r -y && \
		  catkin_make'

# Install tools
RUN apt-get update && apt-get install -y \
    ros-noetic-usb-cam \
    ros-noetic-image-pipeline \
    tmux \
    jq &&\
    rm -rf /var/lib/apt/lists/*

# Install dependencies (and GNM)
COPY ./ /opt/placenav
WORKDIR /opt/placenav
RUN --mount=type=cache,target=/root/.cache \
    pip install -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cu113

WORKDIR /opt/
RUN --mount=type=cache,target=/root/.cache \
    pip install -e "git+https://github.com/robodhruv/drive-any-robot.git@main#egg=drive-any-robot&subdirectory=train"

# Install PlaceNav as package
# Installing as editable allows mounting the
# package on top of the installed package,
# allowing for easy development
WORKDIR /opt/placenav
RUN pip install -e .

# Build the placenav visualization messages
WORKDIR /opt/placenav/src/placenav_viz_msgs
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Update entrypoint
WORKDIR /opt/placenav
COPY docker/turtlebot/entrypoint.sh /opt/entrypoint.sh
ENTRYPOINT ["/opt/entrypoint.sh", "bash"]