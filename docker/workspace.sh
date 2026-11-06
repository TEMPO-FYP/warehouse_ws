#!/bin/bash
set -e

# Set ROS 2 distribution as a variable
ROS_DISTRO="jazzy"

# Source ROS 2 setup
source /opt/ros/$ROS_DISTRO/setup.bash

# Install system dependencies
apt-get update && apt-get install -y \
    gnupg \
    curl \
    libpcap-dev

# Navigate to workspace root
cd /root/warehouse_ws

# Install ROS 2 dependencies for workspace packages
echo "Installing ROS 2 dependencies..."
rosdep update
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
