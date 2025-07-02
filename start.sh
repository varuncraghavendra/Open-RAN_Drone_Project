# In px4_sitl_img

FROM ubuntu:22.04

ENV TZ=Asia/Dubai
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install base dependencies (excluding python3-pyserial and NuttX packages)
RUN apt-get update && apt-get install -y \
    python3-pip \
    vim \
    tmux \
    tmuxinator \
    iputils-ping \
    libegl-mesa0 \
    git \
    lsb-core \
    sudo \
    wget \
    curl \
    ca-certificates \
    python3-jinja2 \
    python3-empy \
    python3-numpy \
    python3-toml \
    python3-dev \
    ninja-build \
    exiftool \
    cmake \
    build-essential \
    genromfs \
    libeigen3-dev \
    libxml2-utils \
    python3-setuptools \
    python3-wheel \
    python3-venv \
    xxd \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* /var/cache/apt/archives/*

# Install Python modules
RUN pip install pyserial
RUN pip uninstall -y em && pip install empy==3.3.4

# Clone PX4 Autopilot repo
RUN git clone https://github.com/annaa95/PX4-Autopilot.git /app/PX4-Autopilot
RUN cd /app/PX4-Autopilot && git fetch && git checkout v1.14.3/aa-dev-branch && git submodule update --init --recursive

# Patch ubuntu.sh to remove nuttx() function entirely
RUN sed -i '/^nuttx() {/,/^}/d' /app/PX4-Autopilot/Tools/setup/ubuntu.sh

# Run PX4 SITL setup script (now without nuttx section)
RUN /bin/bash /app/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx

# Pre-build PX4 SITL target
RUN cd /app/PX4-Autopilot && DONT_RUN=1 make px4_sitl_default
