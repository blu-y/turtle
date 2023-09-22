```
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

```
ssh-keygen -t rsa
cd ~/.ssh
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


### SLAM
```
ros2 launch turtlebot4_navigation slam.launch.py
```
for custom parameter, add `params:=/full/path/to/slam.yaml`
##### Saving the map
```
ros2 run nav2_map_server map_saver_cli -f "map_name" --ros-args -p map_subscribe_transient_local:=true -r __ns:=/namespace
```
##### SLAM from existing map
```
ros2 launch turtlebot4_navigation localization.launch.py map:=/path/to/map.yaml
```
### Navigation
```
ros2 launch turtlebot4_navigation localization.launch.py map:=office.yaml
```
another terminal, launch
```
ros2 launch turtlebot4_navigation nav2.launch.py
```
and the other terminal, launch
```
ros2 launch turtlebot4_viz view_robot.launch.py
```
set approximate initail pose with `2D Pose Estimate`
and use `Nav2 Goal` to make goal position
