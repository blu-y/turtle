```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

```
git add .
git commit -m "commit message"
git branch -M main
git push origin main
```
`alias push="git add . && git commit -m 'update' && git branch -M main && git push origin main" `

##### Testing Turtlebot4
`ros2 run turtlebot4_tests ros_tests`

##### teleop
```
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
source /opt/ros/humble/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

##### Simulation
```

sudo apt install ros-dev-tools

sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress

sudo apt update
sudo apt install ros-humble-turtlebot4-simulator

ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true

```
