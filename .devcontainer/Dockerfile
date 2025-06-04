# 1) Base: Gazebo 11 on Ubuntu 20.04 (focal)
FROM gazebo:gzserver11-focal

# 2) Suppress interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# 3) ARG for versions
ARG NODE_MAJOR_VERSION=18
ARG NLOHMANN_JSON_VERSION=v3.11.3

# 4) Install absolutely essential prerequisites for managing APT sources
#    This layer should be very stable.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    apt-transport-https \
    && rm -rf /var/lib/apt/lists/*

# 5) Define ALL APT sources (each in its own layer for better caching if one definition changes)
#    Layer 5.1: Base Ubuntu sources
COPY .devcontainer/sources.list /etc/apt/sources.list
#    Layer 5.2: Node.js source
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_${NODE_MAJOR_VERSION}.x nodistro main" > /etc/apt/sources.list.d/nodesource.list
#    Layer 5.3: ROS Noetic source
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
#    Note: OSRF (Gazebo) sources are typically included in the base gazebo image. If not, add them here.

# 6) Perform ONE main apt-get update.
#    This layer is the primary one that might break due to external repo changes.
RUN apt-get update

# 7) Install packages in logical groups, from more stable to less stable (or by function)
#    Group 7.1: Core build tools and system utilities (very stable dependencies)
RUN apt-get install -y --no-install-recommends \
    git \
    build-essential \
    cmake \
    pkg-config \
    sudo \
    software-properties-common \
    x11-apps \
    tmux
#    Group 7.2: Python base and common libraries (stable)
RUN apt-get install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-venv \
    python3-numpy
#    Group 7.3: Gazebo and its direct dependencies (fairly stable once versions are set)
RUN apt-get install -y --no-install-recommends \
    gazebo11 libgazebo11-dev \
    libgl1-mesa-dev libglu1-mesa freeglut3 \
    libeigen3-dev libopencv-dev \
    libprotobuf-dev protobuf-compiler
#    Group 7.4: ROS specific packages (stable for a given ROS distro)
RUN apt-get install -y --no-install-recommends \
    ros-noetic-gazebo-msgs \
    python3-empy \
    python3-jinja2
#    Group 7.5: Node.js and other specific libraries (Node.js version tied to ARG)
RUN apt-get install -y --no-install-recommends \
    nodejs \
    libzmq3-dev \
    astyle

# 8) Clean up APT lists ONCE after all installs are done
RUN rm -rf /var/lib/apt/lists/*

# --- Add nlohmann/json header manually ---
RUN mkdir -p /usr/local/include/nlohmann && \
    curl -Lo /usr/local/include/nlohmann/json.hpp https://github.com/nlohmann/json/releases/download/${NLOHMANN_JSON_VERSION}/json.hpp

# 9) Install Python packages from requirements.txt
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# 10) Create a non-root 'vscode' user
RUN useradd --create-home --shell /bin/bash -G sudo vscode && \
    echo "vscode ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/vscode_nopasswd

# 11) Auto-source Gazebo and ROS setup for vscode
RUN { \
        echo "source /opt/ros/noetic/setup.bash"; \
        echo "source /usr/share/gazebo/setup.sh"; \
        echo "export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH:-\$HOME/workspace/.gazebo/models:\$HOME/.gazebo/models}"; \
        echo "export GAZEBO_PLUGIN_PATH=\${GAZEBO_PLUGIN_PATH:-\$HOME/workspace/gazebo_plugin/build}"; \
    } >> /home/vscode/.bashrc && \
    chown vscode:vscode /home/vscode/.bashrc

# 12) Pre-create log and .gazebo/models folders (as vscode user)
USER vscode
RUN mkdir -p /home/vscode/logs && \
    mkdir -p /home/vscode/.gazebo/models && \
    mkdir -p /home/vscode/workspace/.gazebo/models

# 13) Copy startup and utility scripts, then set permissions
USER root
COPY .devcontainer/start_gazebo.sh /usr/local/bin/start_gazebo.sh
COPY .devcontainer/rebuild_plugin.sh /usr/local/bin/rebuild_plugin.sh
RUN chmod +x /usr/local/bin/start_gazebo.sh /usr/local/bin/rebuild_plugin.sh && \
    chown vscode:vscode /usr/local/bin/start_gazebo.sh /usr/local/bin/rebuild_plugin.sh

# 14) Switch to 'vscode' user and set working directory
USER vscode
WORKDIR /home/vscode/workspace

# 15) Expose ports
EXPOSE 11345/tcp
EXPOSE 5555/tcp
EXPOSE 28642/tcp

# 16) Default command
CMD ["tail", "-f", "/dev/null"]