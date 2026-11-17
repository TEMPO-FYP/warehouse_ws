#!/bin/bash
set -e

# Use ROS_DISTRO from environment (set by base image), fall back to jazzy if not set
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# Source ROS 2 setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Install system dependencies
apt-get update && apt-get install -y \
    gnupg \
    curl \
    libpcap-dev

# Add Clearpath rosdep sources so rosdep can resolve clearpath package dependencies
wget https://raw.githubusercontent.com/clearpathrobotics/public-rosdistro/master/rosdep/50-clearpath.list \
    -O /etc/ros/rosdep/sources.list.d/50-clearpath.list
rosdep update

# Navigate to workspace root
cd /root/warehouse_ws

# Install ROS 2 dependencies for workspace packages
echo "Installing ROS 2 dependencies..."
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Install MoveIt Task Constructor and Python dependencies
echo "Installing MoveIt Task Constructor and Python dependencies..."
apt-get install -y \
    ros-$ROS_DISTRO-moveit-task-constructor-core \
    ros-$ROS_DISTRO-moveit-task-constructor-msgs \
    ros-$ROS_DISTRO-moveit-task-constructor-visualization \
    python3-scipy \
    python3-sklearn

# Install Python dependencies for LLM planner
echo "Installing LLM Planner Python dependencies..."
pip install --no-cache-dir sentence-transformers groq google-genai

# Fix PCL warning if PCL is installed
echo "Fixing PCL warnings..."
if ls -d /usr/include/pcl* 1> /dev/null 2>&1; then
    find /usr/include/pcl* -path "*/sample_consensus/impl/sac_model_plane.hpp" -exec sed -i 's/^\(\s*\)PCL_ERROR ("\[pcl::SampleConsensusModelPlane::isSampleGood\] Sample points too similar or collinear!\\n");/\1\/\/ PCL_ERROR ("[pcl::SampleConsensusModelPlane::isSampleGood] Sample points too similar or collinear!\\n");/' {} \;
fi

# Build workspace packages
echo "Building packages..."
colcon build --symlink-install
source install/setup.bash

echo "Workspace setup completed!"