# Use NVIDIA's CUDA 11.7.1 base image with Ubuntu 22.04
FROM nvidia/cuda:11.7.1-devel-ubuntu22.04
LABEL maintainer="Om Tiwari"

# Set non-interactive mode for apt-get
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y --no-install-recommends git curl wget git zsh tmux vim g++


# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && locale-gen en_US.UTF-8

# Set locale environment variables
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Add ROS 2 GPG key and repository to sources list
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2-latest.list
    
# ==========> INSTALL ROS humble <=============
# Install ROS 2 Humble Desktop
RUN apt-get update && apt-get install -y ros-humble-desktop

# =========> INSTALL OpenPCDet <=============
RUN apt update && apt install -y python3-pip
RUN pip install torch==2.0.0+cu117 torchvision==0.15.0+cu117 torchaudio==2.0.0+cu117 --extra-index-url https://download.pytorch.org/whl/cu117

#RUN pip install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu117
RUN pip3 install spconv-cu117
RUN apt update && apt install -y python3-setuptools
# Install Python 3.10 and pip
RUN apt-get update && apt-get install -y python3.10 python3-pip

RUN apt-get update && apt-get install -y sudo

# Set the default shell to bash
ENV SHELL=/bin/bash


