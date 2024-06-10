FROM ros:noetic-perception
ARG USERNAME=seamate-docker
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
ENV SHELL /bin/bash

# Install build dependencies
RUN apt-get clean && \
    apt-get update && \
    apt-get install -y wget software-properties-common python3-catkin-tools libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev libfmt-dev psmisc ros-noetic-tf-conversions libboost-all-dev python3-pybind11 libzip-dev python3-pip libspdlog-dev ros-noetic-rviz && \
    rm -rf /var/lib/apt/lists/

RUN wget ceres-solver.org/ceres-solver-2.1.0.tar.gz
RUN tar zxf ceres-solver-2.1.0.tar.gz
RUN mkdir ceres-bin
WORKDIR /ceres-bin
RUN cmake ../ceres-solver-2.1.0
RUN make -j4
RUN make install

RUN pip3 install numpy --upgrade
RUN pip3 install scikit-learn seaborn tqdm tabulate

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
