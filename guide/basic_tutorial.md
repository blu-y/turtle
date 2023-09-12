## Turtlebot4 Basic Tutorial

### Keyboard Teleoperation
ROS2 통신을 통해 PC의 키보드로 같은 ROS2 네트워크에 연결된 터틀봇을 움직일 수 있다. `teleop_twist_keyboard`패키지를 설치한다.  
```
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
```
설치가 완료되면 다음 명령으로 노드를 실행시킨다.  
```
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
`source /opt/ros/humble/setup.bash`는 ROS의 설정을 갱신시켜주는 역할을 한다. ROS 관련 작업을 할 때 실행시켜주어야 하며 [이전](https://blu-y.github.io/turtle/guide/basic_setup#robot)과정에서 해당 부분을 `~/.bashrc`에 추가하였을 경우 입력해주지 않아도 된다.  
