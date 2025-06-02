# 1) Base: Gazebo 11 on Ubuntu 20.04 (focal) for amd64
FROM gazebo:gzserver11-focal

# 2) Suppress interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# 3) Install Python 3, NumPy, and Gazebo GUI (gzclient) plus basic X11 deps
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3 \
      python3-pip \
      gazebo11 \
      x11-apps \
      libgl1-mesa-dev \
      libglu1-mesa \
      freeglut3 && \
    pip3 install numpy && \
    rm -rf /var/lib/apt/lists/*

# 4) Create a non-root 'vscode' user
RUN useradd --create-home --shell /bin/bash vscode

# 5) Auto-source Gazebo setup and set GAZEBO_MODEL_PATH for both root and vscode
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/home/vscode/.gazebo/models:\$GAZEBO_MODEL_PATH" >> /root/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> /home/vscode/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/home/vscode/.gazebo/models:\$GAZEBO_MODEL_PATH" >> /home/vscode/.bashrc && \
    chown vscode:vscode /home/vscode/.bashrc

# 6) Pre-create the custom-model folder (for your checked-in model.sdf)
USER root
RUN mkdir -p /home/vscode/.gazebo/models/tesseract_nano && \
    chown -R vscode:vscode /home/vscode/.gazebo

# 7) Switch to 'vscode' user and set working directory
USER vscode
WORKDIR /home/vscode

# 8) Default to bash login shell so ~/.bashrc is sourced
CMD [ "bash", "-l" ]
