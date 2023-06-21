#!/bin/bash
echo "Using the default rosinstall file!"
CONFIG_FILE=workspace.rosinstall
ROSINSTALLFILE=$(cd $(dirname $CONFIG_FILE) && pwd -P)/$CONFIG_FILE

# Install dependencies
echo "Installing dependencies..."
if [ $ROS_PYTHON_VERSION -eq 3 ]
then
  sudo apt-get install -y python-is-python3 python3-wstool python3-catkin-tools
else
  sudo apt-get install -y python-wstool python-catkin-tools
fi

# Initialize catkin workspace
echo "Initializing catkin workspace from $ROSINSTALLFILE ... (strg + c to cancel)"
sleep 5

# remove old workspace if exists
cd ..
if [ -d "catkin_ws" ]
then
  echo "Removing old catkin workspace..."
  rm -rf catkin_ws
fi

mkdir -p catkin_ws/src
catkin init --workspace catkin_ws
cd catkin_ws
wstool init src $ROSINSTALLFILE

# Create simbolic link of the packages in src
echo "Creating simbolic link of the packages..."
cd src
ln -s ../../controller
ln -s ../../planning
ln -s ../../waypoint_msgs
cd ..

# Install required ros-dependencies with rosdep
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
echo "Building packages..."
catkin build
return_value=$?
if [ ${return_value} -eq 1 ]
then
  echo "Build failed"
  exit 1
fi
echo
echo "Remember to source the build files!"
echo "Please run:"
echo "    source catkin_ws/devel/setup.bash"
