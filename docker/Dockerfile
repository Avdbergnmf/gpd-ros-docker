# Based on https://github.com/DLopezMadrid/pcl-docker
# Adapted for GPD-ROS use (GPD Lib: https://github.com/atenpas/gpd, ROS Wrapper: https://github.com/atenpas/gpd_ros/)

# You will need to have the nvidia GPU drivers and nvidia CUDA drivers in your host to take full advantage of this image

FROM nvidia/opengl:1.0-glvnd-runtime-ubuntu18.04

ENV CONTAINER_USER="gpd"

# ================================================ NVIDIA CUDA Installation ================================================
# Example from https://gitlab.com/nvidia/container-images/cuda/-/blob/master/dist/10.2/ubuntu18.04-x86_64/base/Dockerfile

RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list && \
    apt-get purge --autoremove -y curl && \
    rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 10.2.89
ENV CUDA_PKG_VERSION 10-2=$CUDA_VERSION-1
# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-$CUDA_PKG_VERSION \
    cuda-compat-10-2 \
    && ln -s cuda-10.2 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.2 brand=tesla,driver>=396,driver<397 brand=tesla,driver>=410,driver<411 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441"

# ======== Installing basic dependencies ========
RUN apt-get update \
    && apt-get install -y \
        build-essential \
        apt-utils \
        wget \
        unzip \
        git

# ======== Installing productivity tools ========
RUN apt-get install -y \
    sudo \
    vim \
    terminator \
    dbus \
    dbus-x11 \
    gdb

# ======== Installing CMake ========
WORKDIR /tmp

RUN apt-get update \
    && apt-get install -y \
        libssl-dev \
        libgl1-mesa-dev

ENV CMAKE_VERSION="3.10.2"
RUN wget https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/cmake-${CMAKE_VERSION}.tar.gz \
    && tar -xvf cmake-${CMAKE_VERSION}.tar.gz \
    && cd cmake-${CMAKE_VERSION} \
    && ./bootstrap -- -DCMAKE_BUILD_TYPE:STRING=Release \
    && make -j$(nproc) \
    && make install

# ======== Installing VTK ========
WORKDIR /tmp

RUN apt-get update && apt-get install -y \
    libxt-dev 

RUN wget https://www.vtk.org/files/release/8.2/VTK-8.2.0.tar.gz \
    && tar -xf VTK-8.2.0.tar.gz \
    && cd VTK-8.2.0 && mkdir build && cd build \
    && cmake .. -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES \
                -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) \
    && make install

# ======== Installing PCL library ========
WORKDIR /tmp
ENV PCL_VERSION="1.11.1"

# PCL dependencies
RUN apt-get install -y \
    libeigen3-dev \
    libflann-dev \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    libboost-all-dev \
    libusb-1.0-0-dev \
    libusb-dev \
    libopenni-dev \
    libopenni2-dev \
    libpcap-dev \
    libpng-dev \
    mpi-default-dev \
    openmpi-bin \
    openmpi-common \
    libqhull-dev \
    libgtest-dev

RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-${PCL_VERSION}.tar.gz \
    && tar -xf pcl-${PCL_VERSION}.tar.gz \
    && cd pcl-pcl-${PCL_VERSION} \
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
                -DVTK_RENDERING_BACKEND=OpenGL2 \
    && make -j$(nproc) \
    && make install

# we need this here else we get a input dialog in which we need to define our geo area for the TZ
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y pcl-tools

RUN unset PCL_VERSION

# ======== Installing OpenCV library ========
RUN apt-get install -y \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

WORKDIR /tmp
ENV OPENCV_VERSION="4.5.2"
RUN wget https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.tar.gz \
    && tar -xf ${OPENCV_VERSION}.tar.gz \
    && cd opencv-${OPENCV_VERSION} \
    && mkdir build && cd build \
    && cmake -D DCMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local .. \
    && make -j$(nproc) \
    && make install

# ======== user config ==========

#Add new sudo user
RUN useradd -m $CONTAINER_USER && \
        echo "$CONTAINER_USER:$CONTAINER_USER" | chpasswd && \
        usermod --shell /bin/bash $CONTAINER_USER && \
        usermod -aG sudo $CONTAINER_USER && \
        echo "$CONTAINER_USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$CONTAINER_USER && \
        chmod 0440 /etc/sudoers.d/$CONTAINER_USER && \
        # Replace 1000 with your user/group id
        usermod  --uid 1000 $CONTAINER_USER && \
        groupmod --gid 999 $CONTAINER_USER

# ======== Installing GPD library ======== <<<
# Based on https://github.com/atenpas/gpd#install
WORKDIR /home/$CONTAINER_USER

# I want the source to be accessible, as I need to refer to it when using gpd_ros
ENV GPD_VERSION="2.0.0"

# Note: the additional flags are necessary as per: https://github.com/atenpas/gpd/issues/88, additionally changed folder name to just gpd to make it easier to find.
RUN wget https://github.com/atenpas/gpd/archive/${GPD_VERSION}.tar.gz \
    && tar -xf ${GPD_VERSION}.tar.gz \
    && mv gpd-${GPD_VERSION} gpd \
    && cd gpd \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=DEBUG -DCMAKE_CXX_FLAGS_DEBUG="-O3 -march=native -mtune=intel -msse4.2 -mavx2 -mfma -flto -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++17" .. \ 
    && make -j$(nproc) \
    && make install

# ======== Installing ROS Core ======== <<<
# Based on: https://github.com/osrf/docker_images/tree/df19ab7d5993d3b78a908362cdcd1479a8e78b35/ros/melodic/ubuntu/bionic

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends dirmngr \
    && rm -rf /var/lib/apt/lists/*
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list
# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO melodic

###### One option is to install it in steps like done in the official docker images
# #### CORE
# RUN apt-get update && apt-get install -y --no-install-recommends ros-melodic-ros-core=1.4.1-0* \
#     && rm -rf /var/lib/apt/lists/*
# #### BASE
# RUN apt-get update && apt-get install -y --no-install-recommends ros-melodic-ros-base=1.4.1-0* \
#     && rm -rf /var/lib/apt/lists/*
# #### ROBOT (Specifically need eigen_conversions from this)
# RUN apt-get update && apt-get install -y --no-install-recommends ros-melodic-robot=1.4.1-0* \
#     && rm -rf /var/lib/apt/lists/*
# #### DESKTOP (For rviz)
# RUN apt-get update && apt-get install -y --no-install-recommends ros-melodic-desktop=1.4.1-0* \
#     && rm -rf /var/lib/apt/lists/*
#### PERCEPTION (specifically need pcl_conversions from this)

# However, to keep things simple, I'll just install the full desktop install 
RUN apt-get update && apt-get install -y --no-install-recommends ros-melodic-desktop-full \
    && rm -rf /var/lib/apt/lists/*
# Additionally, we need the object recognition packages
RUN apt-get update && apt-get install -y --no-install-recommends ros-melodic-object-recognition-* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

RUN unset ENV DEBIAN_FRONTEND=noninteractive

# setup entrypoint
WORKDIR /home/$CONTAINER_USER
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]