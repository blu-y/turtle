## Turtlebot4 Basic Tutorial 2

### Generating a map

#### SLAM
다음 명령을 통해 SLAM 을 실행할 수 있다.
```
sudo apt install ros-humble-turtlebot4-desktop
sudo apt install ros-humble-turtlebot4-navigation
ros2 launch turtlebot4_navigation slam.launch.py
```

아래 명령을 통해 asynchronous SLAM이나 custom SLAM parameters로 실행도 가능하다.
```
ros2 launch turtlebot4_navigation slam.launch.py sync:=false # asynchronous SLAM
ros2 launch turtlebot4_navigation slam.launch.py params:=/full/path/to/slam.yaml # custom SLAM parameters
```

#### Rviz2
`view_robot` launch 파일을 이용하여 Rviz2로 visualization 할 수 있다.
```
ros2 launch turtlebot4_viz view_robot.launch.py
```

#### Save map
서비스 콜을 이용하여 현재까지 작성된 맵을 저장할 수 있다.
```
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'map_name'"
```
맵을 저장하게 되면 맵이 저장된 `map_name.pgm` 파일과 여러 설정들이 저장된 `map_name.yaml` 파일이 저장된다.
설정파일을 통해 여러 parameter를 조절할 수 있다.

### Navigation
SLAM은 새로운 지도를 작성하거나, 알려지지않은(또는 dynamic한) 환경에서 유용하다. 지도에서 새로운 부분이 발견되거나 변경점이 발견되면 맵을 업데이트한다.  
Localization은 이미 작성된 지도를 사용하여 현재 로봇에서 인식된 센서를 토대로 현재 위치와 자세를 계산한다.  

#### Nav2
다음 명령을 통해 navigation을 실행할 수 있다. `map_name.yaml`에 저장된 map의 yaml 파일을 이용하여 실행한다.
```
ros2 launch turtlebot4_navigation localization.launch.py map:=map_name.yaml
```
다른 터미널을 열어 Nav2를 실행한다.
```
ros2 launch turtlebot4_navigation nav2.launch.py
```
다른 터미널을 열어 Rviz2를 실행하여 visualize한다.
```
ros2 launch turtlebot4_viz view_robot.launch.py
```
Rviz2에서 여러 Navigation 툴을 이용할 수 있다.  
`2D Pose Estimate` : 대략적인 initial pose를 입력해주면 현재 위치를 추정할 수 있다.
`Publish Point`: Map 상에 point를 publish한다. `/clicked_point` 토픽으로 publish 된다. (`ros2 topic echo /clicked_point`로 확인)
`Nav2 Goal`: 원하는 goal pose를 설정해 주면 Nav2가 로봇을 goal pose로 이동시키려고 시도한다. 사용하기 전 initial pose를 설정해주어야 한다.
