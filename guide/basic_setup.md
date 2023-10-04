## Turtlebot4 Basic Setup
### Note
#### PC / RPi / Create
터틀봇은 구동부인 Create3와 여러 센서가 장착된 Raspberry Pi 컴퓨터로 이루어져 있다. 편의상 우리가 사용하는 User PC는 `PC`, 라즈베리 파이는 `RPi`, Create3는 `Create`라고 작성하였다.
### PC
1. ROS2 설치 : [Humble(22.04)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)  
   `Note: Ubuntu, ROS2 설치 시 오류 (최신 NVIDIA 그래픽카드 관련)`
2. 터틀봇 패키지 설치 :
   ```
   sudo apt update && sudo apt install ros-humble-turtlebot4-desktop
   ```

### Robot

1. 터틀봇 이미지(22.04) 설치  
   [링크](http://download.ros.org/downloads/turtlebot4/)에서 터틀봇 이미지를 다운받아 터틀봇의 RPi에 장착된 sd카드에 설치한다. PC에 터틀봇 sd카드를 장착한 뒤 다음 명령을 통해 설치한다.

   ```
   sudo apt install dcfldd
   sudo fdisk -l
   ```
   위 명령어를 통해 설치된 SD 카드를 확인한다. SD의 이름은 `/dev/mmcblk0`이나 `/dev/sda`같은 꼴로 되어있다. 다음 명령을 따라 이미지를 SD카드에 설치한다.
   ```
   wget https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/scripts/sd_flash.sh
   bash sd_flash.sh /path/to/downloaded/image
   ```
2. 터틀봇 AP 연결  
   터틀봇을 동봉된 독에 올려놓으면 시작된다. 터틀봇을 처음 시작하면 터틀봇에 장착된 RPi는 AP(Access Point) 모드로 시작한다. PC의 와이파이(5GHz를 지원하여야 함)를 통해 `Turtlebot4` 와이파이에 연결한다. 비밀번호도 `Turtlebot4`이다.
3. RPi에 SSH 접속  
   터틀봇 와이파이에 접속되었다면 SSH를 통해 터틀봇의 RPi에 접속할 수 있다. 접속 시 비밀번호는 `turtlebot4`이다.

   ```
   ssh ubuntu@10.42.0.1
   ```
4. RPi를 와이파이에 연결  
   ssh를 통해 연결된 RPi에 다음 명령어를 사용하면 터틀봇 설정창에 진입한다.

   ```
   turtlebot4-setup
   ```
   여러 대의 터틀봇은 한 와이파이에 구동하기 위해선 서로 네트워크가 분리되어야 한다. 분리하기 위해 ROS에서는 namespace와 domain 두 방법이 있는데 namespace는 topic앞에 `/namespace1/ip` 처럼 접두사가 붙고 domain은 아예 다른 네트워크에서 ROS를 실행한 것처럼 사용할 수 있다. 본 실습에서는 서로의 로봇이 혼선되지 않기 위하여 domain을 사용한다.
   `ROS Setup`에 들어가 `Domain`에 부여된 숫자를 입력한 후 `save`한다.
   컴퓨터와 같은 와이파이에 연결하기 위하여 `Wi-Fi Setup`에 들어가 `Wi-Fi Mode`를 `Client`로 변경한 후, `SSID`와 `Password`에 사용할 와이파이 이름과 비밀번호를 입력한 후 `save`한다.
   설정이 완료되었다면 `Apply Settings`를 통해 설정을 적용한다.
5. RPi IP 확인  
   PC에서도 같은 Domain ID를 설정하여야 한다.

   ```
   source /opt/ros/humble/setup.bash
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   export ROS_DOMAIN_ID=<부여받은_Domain_ID>
   ```
   위 명령을 `~/.bashrc`에 추가한 후 `source ~/.bashrc`명령을 통해 적용하면 터미널 창이 실행될 때마다 Domain ID가 적용될 것이다.
   설정을 완료하면 터틀봇이 재부팅되며 터틀봇 화면에서 IP 주소를 확인할 수 있다. PC에서 다음 명령을 실행해 보자.
   ```
   $ ros2 topic echo /ip
   data: 192.168.28.24
   ```
   IP를 확인하면 이제는 해당 IP로 터틀봇에 SSH 접속할 수 있다.
   ```
   ssh ubuntu@192.168.28.24
   ```
6. Create 웹서버 접속  
   새롭게 연결되면 구동부인 Create3는 웹브라우저에서 IP와 8080 포트를 통해 Create 웹서버에 접속할 수 있다. `192.168.28.24:8080`
7. Create3 펌웨어 업그레이드  
   Humble로 터틀봇을 업글레이드 해주었기 때문에 Galactic으로 설치되어 있는 Create의 펌웨어 또한 업그레이드 시켜주어야 한다. [링크](https://iroboteducation.github.io/create3_docs/releases/overview/)에서 Humble용 최신 펌웨어를 받아, Create 웹서버의 `Upgrade`탭에서 업그레이드를 진행해 준다. Humble로 설정하여야 한다.
8.  블루투스 컨트롤러 연결  
   동봉된 컨트롤러는 터틀봇과 미리 페어링되어 있어 켜면 바로 연결될 것이다. 만약 연결되지 않는다면 [링크 참조](https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html#turtlebot-4-controller-setup)

#### 수동 연결  
   와이파이가 변경되면 터틀봇에 연결할 수 없기 때문에 이더넷 케이블을 통해 연결하여야 한다.  
   이더넷 케이블로 PC와 터틀봇을 연결한 후 컴퓨터의 IP를 수동으로 설정해주어야 한다.  
   `IP: 192.168.185.5`, `Subnet Mask: 255.255.255.0`, `Gateway: 192.168.185.1`로 PC의 IP를 수동으로 설정해 준 후 터틀봇의 유선 정적 IP인 `192.168.185.3`을 통해 SSH 접속한 후 와이파이의 이름과 비밀번호를 다시 설정할 수 있다.
   ```
   ssh ubuntu@192.168.185.3
   turtlebot4-setup
   ```


#### 터틀봇 4 테스트
   ssh로 접속 후 다음 명령으로 터틀봇 4 테스트를 진행할 수 있다
   ```
   source /opt/ros/humble/setup.bash
   ros2 run turtlebot4_tests ros_tests
   ```
