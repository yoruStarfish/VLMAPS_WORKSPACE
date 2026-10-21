
# ============================================================
# 1. Base Image: ROS2 Humble Desktop
# ============================================================
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV TERM=xterm-256color

SHELL ["/bin/bash", "-c"]

# ============================================================
# 2. System dependencies
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    can-utils \
    wget \
    git \
    iproute2 \
    build-essential \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    libasio-dev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    usbutils \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# 3. ROS2 Sensor and Navigation Dependencies
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-velodyne-driver \
    ros-humble-velodyne-pointcloud \
    ros-humble-velodyne-laserscan \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# 4. Install Miniforge
# ============================================================
RUN curl -fsSL -o /tmp/miniforge.sh \
    "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname -s)-$(uname -m).sh" \
    && bash /tmp/miniforge.sh -b -p /opt/miniforge3 \
    && rm /tmp/miniforge.sh

ENV PATH="/opt/miniforge3/bin:${PATH}"

# ============================================================
# 5. Create Conda Environment
# ============================================================
COPY environment.yaml /tmp/environment.yaml

RUN conda env create -f /tmp/environment.yaml \
    && conda clean -afy \
    && rm /tmp/environment.yaml

# ============================================================
# 6. Configure ROS2 and Conda
# ============================================================
RUN sed -i \
    's/#force_color_prompt=yes/force_color_prompt=yes/g' \
    /root/.bashrc \
    && echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /opt/miniforge3/etc/profile.d/conda.sh' >> /root/.bashrc \
    && echo 'export TERM=xterm-256color' >> /root/.bashrc

# ============================================================
# 7. ROS2 Workspace
# ============================================================
WORKDIR /workspace

# Source workspace automatically if already built
RUN echo 'if [ -f /workspace/install/setup.bash ]; then' \
    >> /root/.bashrc \
    && echo '    source /workspace/install/setup.bash' \
    >> /root/.bashrc \
    && echo 'fi' >> /root/.bashrc

# Use NVIDIA PRIME rendering instead of forcing Mesa software rendering.
ENV LIBGL_ALWAYS_SOFTWARE=0
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# Update both TF libraries together; tf2 >= 0.25.24 fixes the RViz TF deadlock.
RUN apt-get update \
    && apt-get install -y --only-upgrade ros-humble-tf2 ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# 8. Default Command
# ============================================================
CMD ["/bin/bash"]
