#!/bin/bash
set -e

# Set ROS 2 distribution as a variable
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

# Navigate to workspace root
cd /root/warehouse_ws

# Install ROS 2 dependencies for workspace packages
echo "Installing ROS 2 dependencies..."
rosdep update
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Re-source ROS 2 setup so hooks from newly installed vendor packages (e.g. gz_sim_vendor) are loaded
source /opt/ros/$ROS_DISTRO/setup.bash

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
