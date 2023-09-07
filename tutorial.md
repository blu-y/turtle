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
