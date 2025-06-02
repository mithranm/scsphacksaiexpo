# 1. Base: Official Gazebo 11 server image (Ubuntu 20.04 "focal" for amd64)
FROM gazebo:gzserver11-focal

# 2. Suppress interactive prompts in apt-get
ENV DEBIAN_FRONTEND=noninteractive

# 3. Install Python 3 and NumPy (for our generator script)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      python3 python3-pip && \
    pip3 install numpy && \
    rm -rf /var/lib/apt/lists/*

# 4. Create an unprivileged 'vscode' user (UID 1000)
RUN useradd --create-home --shell /bin/bash vscode

# 5. Ensure Gazebo is sourced in every new shell for both root and vscode
RUN echo "source /usr/share/gazebo/setup.sh" >> /root/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> /home/vscode/.bashrc && \
    chown vscode:vscode /home/vscode/.bashrc

# 6. Create the folder for custom models (so the Python generator can write there)
USER root
RUN mkdir -p /home/vscode/.gazebo/models/tesseract_nano && \
    chown -R vscode:vscode /home/vscode/.gazebo

# 7. Switch to the unprivileged user for subsequent work
USER vscode
WORKDIR /home/vscode

# 8. Open bash as a login shell (so ~/.bashrc runs automatically)
CMD [ "bash", "-l" ]
