# ...existing code...

# Remove malformed ROS sources list if it exists
RUN rm -f /etc/apt/sources.list.d/ros-latest.list

# 6) Perform ONE main apt-get update.
RUN apt-get update

# ...existing code...