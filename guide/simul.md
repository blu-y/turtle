# dev tools
sudo apt install ros-dev-tools

# ignition gazebo
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress

# package install
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator

# launch
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py  
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
