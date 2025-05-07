FROM ubuntu:18.04
SHELL ["/bin/bash", "-c"]

RUN apt-get update -y \
    && DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt-get install -y \
    python3-dev \
    python3-numpy \
    python3-pip \
    curl \
    libpcl-dev \
    libi2c-dev \
    screen \
    vim \
    mc \
    aptitude \
    git \
    gstreamer1.0-gtk3 \
    build-essential \
    cmake \
    libgtk2.0-dev \
    pkg-config \
    libswscale-dev \
    libtbb2 \
    libtbb-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libv4l-dev \
    v4l-utils \
    qv4l2 \
    v4l2ucp \
    libdc1394-22-dev \
    lsb-release \
    sudo \
    wget \
    net-tools \
    && apt-get clean

RUN mkdir -p /home/ballsbot/projects

# opencv with gstreamer support
RUN cd /home/ballsbot/projects \
    && git clone https://github.com/opencv/opencv_contrib && cd opencv_contrib && git checkout 4.5.5 && cd .. \
    && git clone https://github.com/opencv/opencv && cd opencv && git checkout 4.5.5 \
    && mkdir release && cd release/ \
    && cmake -D WITH_CUDA=ON \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        -D WITH_GSTREAMER=ON \
        -D WITH_LIBV4L=ON \
        -D BUILD_opencv_python2=OFF \
        -D BUILD_opencv_python3=ON \
        -D BUILD_TESTS=OFF \
        -D BUILD_PERF_TESTS=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D INSTALL_C_EXAMPLES=OFF \
        -D INSTALL_PYTHON_EXAMPLES=OFF \
        -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
        -D CMAKE_INSTALL_PREFIX=/usr/local .. \
    && make -j4 \
    && make install

# pcl 1.10
# RUN cd /home/ballsbot/projects \
#     && wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.10.1.tar.gz \
#     && tar xzf pcl-1.10.1.tar.gz \
#     && cd pcl-pcl-1.10.1 && mkdir build && cd build \
#     && cmake .. && make -j4 && make install

# ros
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    && apt-get update -y && apt-get install -y ros-melodic-ros-base python3-rosdep \
    && source /opt/ros/melodic/setup.bash \
    && mkdir -p /home/ballsbot/catkin_ws/src \
    && cd /home/ballsbot/catkin_ws/ \
    && catkin_make \
    && rosdep init && rosdep update --include-eol-distros

# lidar
COPY ./docker/ydlidar_ros/ /home/ballsbot/catkin_ws/src/ydlidar_ros/
RUN source /opt/ros/melodic/setup.bash \
    && cd /home/ballsbot/catkin_ws/ \
    && catkin_make \
    && source devel/setup.bash \
    && roscd ydlidar_ros/startup \
    && chmod 777 ./* \
    && sh initenv.sh

# python
RUN pip3 install pip -U \
    && pip3 install \
    packaging \
    notebook \
    adafruit-pca9685 \
    Adafruit_PureIO==1.0.1 \
    ipywidgets \
    jupyterlab \
    Pillow \
    smbus \
    Cython \
    flake8 \
    numpy \
    tqdm
RUN pip3 install matplotlib -U

# imu
RUN cd /home/ballsbot/projects \
    && git clone https://github.com/jetsonhacks/RTIMULib \
    && cd RTIMULib/Linux/python/ \
    && python3 setup.py build \
    && sudo python3 setup.py install

# main project
RUN mkdir -p /home/ballsbot/projects/ballsbot
COPY ./ /home/ballsbot/projects/ballsbot/
RUN cd /home/ballsbot/projects/ballsbot/python_packages/ballsbot_localization/ \
    && rm -rf ballsbot_localization.cpp build && python3 setup.py build_ext \
    && python3 setup.py install -f
RUN cd /home/ballsbot/projects/ballsbot/python_packages/ballsbot_manipulator_geometry/ \
    && rm -rf ballsbot_manipulator_geometry.cpp build && python3 setup.py build_ext \
    && python3 setup.py install -f
RUN cd /home/ballsbot/projects/ballsbot/python_packages/python-pcl-ndt/ \
    && rm -rf python_pcl_ndt.cpp build && python3 setup.py build_ext \
    && python3 setup.py install -f

# jupyter
RUN mkdir /home/ballsbot/notebooks \
    && cp /home/ballsbot/projects/ballsbot/python/notebooks/* /home/ballsbot/notebooks/

RUN cd /home/ballsbot \
    && ln -s /home/ballsbot/projects/ballsbot/data/ballsbot_config_override.py
RUN ln -s /usr/local/include/opencv4 /usr/include/opencv4 \
    && grep -vF ballsbot_detection /home/ballsbot/projects/ballsbot/scripts/make-ros-modules.sh > /home/ballsbot/projects/ballsbot/scripts/make-ros-modules.fixed.sh \
    && chmod 755 /home/ballsbot/projects/ballsbot/scripts/make-ros-modules.fixed.sh \
    && /home/ballsbot/projects/ballsbot/scripts/make-ros-modules.fixed.sh

WORKDIR /home/ballsbot/projects/ballsbot

CMD [ "bash", "-c", "sleep inf"]