# STL-AEP
Guiding Efficient 3D Exploration with Temporal Logic

# Install instructions

## Install ROS Melodic (ros-desktop-full)

## Install catkin_tools

## Setup ROS workspace

# Install catkin tools

# Setup ROS workspace

# wstool init src

# Update .rosinstall

# rosdep install --from-paths src --ignore-src -r -y

# sudo apt install ros-melodic-octomap-*

// # sudo apt install python-pip

// # pip install rtree

// # sudo apt install libspatialindex-dev

# Install Mavros and Mavlink

# Install dependencies
sudo apt-get update
sudo apt-get install cmake python-pip python-rosinstall python-rosinstall-generator python-wstool
sudo apt-get install build-essential python-catkin-tools libprotobuf-dev libprotoc-dev protobuf-compiler
sudo apt-get install libeigen3-dev libgstreamer1.0-* libimage-exiftool-perl python-jinja2 libgeographic-dev geographiclib-tools ros-melodic-mav*

pip install numpy toml future

cd ~/catkin_ws
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# Add this to your .bashrc file found at ~/.bashrc:
# Set the plugin path so Gazebo finds our model and sim export
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ws/src/px4/build/posix_sitl_default/build_gazebo
# Set the model path so Gazebo finds the airframes
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/px4/Tools/sitl_gazebo/models:~/catkin_ws/src/rpl_uav/simulation/models
# Set path to sitl_gazebo repository
export SITL_GAZEBO_PATH=~/catkin_ws/src/px4/Tools/sitl_gazebo
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/catkin_ws/src/px4/Tools/sitl_gazebo/Build/msgs/:~/catkin_ws/src/px4/build/posix_sitl_default/build_gazebo
# So ROS finds these packages
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/catkin_ws/src/px4:~/catkin_ws/src/px4/Tools/sitl_gazebo

# Build the UAV
cd ~/catkin_ws/src/px4
make posix_sitl_default gazebo

# Build workspace
cd ~/catkin_ws/
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build