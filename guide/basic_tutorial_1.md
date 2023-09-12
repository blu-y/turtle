## Turtlebot4 Basic Tutorial 1

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
`source /opt/ros/humble/setup.bash`는 ROS의 설정을 갱신시켜주는 역할을 한다. ROS 관련 작업을 할 때 실행시켜주어야 하며 [이전](https://blu-y.github.io/turtle/guide/basic_setup#robot)과정에서 해당 부분을 `~/.bashrc`에 추가하였을 경우 새 터미널을 열 때 자동으로 실행되며 `source ~/.bashrc` 명령을 통해 한번에 실행된다.  

#### Topic(Command Velocity)
Teleop 노드는 `/cmd_vel` 토픽을 통해 로봇에 속도를 전달한다. `/cmd_vel` 토픽은 [geometry_msgs/Twist](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html) 형식의 메세지로 로봇에 linear, angular 속도를 전달한다.  
다음 명령을 통해 노드를 생성하지 않아도 `/cmd_vel` 토픽을 발행할 수 있다. Tab 키를 적절히 활용하면 쉽게 작성할 수 있다.  
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
`linear.x`의 값을 수정하여 로봇을 앞뒤로 움직일 수 있으며, `angular.z` 값을 수정하여 좌우로 회전할 수 있다.  

### Creating Node
위 예제에선 `teleop_twist_keyboard` 패키지의 `teleop_twist_keyboard` 노드를 사용해봤다. 이번엔 패키지와 노드를 만드는 방법에 대해 설명한다.  

#### 노드 생성
다음 명령을 통해 workspace로 사용할 폴더를 만든 후 `turtlebot4_python_tutorials` 패키지와 `turtlebot4_first_python_node`를 만든다.  
```
mkdir ~/turtlebot4_ws/src -p=
cd ~/turtlebot4_ws/src
ros2 pkg create --build-type ament_python --node-name turtlebot4_first_python_node turtlebot4_python_tutorials
```

#### 노드 작성(python)
`~/turtlebot4_ws/src/turtlebot4_python_tutorials/turtlebot4_python_tutorials/turtlebot4_first_python_node.py`에 있는 파이썬 파일을 수정하여 노드를 작성할 수 있다.  
이번엔 Create3의 Lightring을 버튼을 눌러 변경하는 노드를 작성해 본다.  

##### Dependencies
패키지를 실행할 때 필요한 다른 패키지들을 dependency로 설정한다. 이번 패키지에서는 ROS2 통신을 python으로 실행하기 위한 `rclpy` 패키지와 Create3를 제어하기 위해 필요한 `irobot_create_msgs` 패키지가 필요하다.  
Dependency를 설정하기 위해선 패키지를 생성할 때 만들어진 `package.xml` 파일에 정의해주어야 한다.  
`package.xml` 파일을 열어 `<buildtool_depend>ament_cmake</buildtool_depend>` 아래에 다음을 추가해준다.  
```
<depend>rclpy</depend>
<depend>irobot_create_msgs</depend>
```

##### Python 파일 작성
`~/turtlebot4_ws/src/turtlebot4_python_tutorials/turtlebot4_python_tutorials/turtlebot4_first_python_node.py` 를 작성한다.  
```
from irobot_create_msgs.msg import InterfaceButtons, LightringLeds

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class TurtleBot4FirstNode(Node):
    lights_on_ = False

    def __init__(self):
        super().__init__('turtlebot4_first_python_node')

        # Subscribe to the /interface_buttons topic
        self.interface_buttons_subscriber = self.create_subscription(
            InterfaceButtons,
            '/interface_buttons',
            self.interface_buttons_callback,
            qos_profile_sensor_data)

        # Create a publisher for the /cmd_lightring topic
        self.lightring_publisher = self.create_publisher(
            LightringLeds,
            '/cmd_lightring',
            qos_profile_sensor_data)

    # Interface buttons subscription callback
    def interface_buttons_callback(self, create3_buttons_msg: InterfaceButtons):
        # Button 1 is pressed
        if create3_buttons_msg.button_1.is_pressed:
            self.get_logger().info('Button 1 Pressed!')
            self.button_1_function()

    # Perform a function when Button 1 is pressed
    def button_1_function(self):
        # Create a ROS 2 message
        lightring_msg = LightringLeds()
        # Stamp the message with the current time
        lightring_msg.header.stamp = self.get_clock().now().to_msg()

        # Lights are currently off
        if not self.lights_on_:
            # Override system lights
            lightring_msg.override_system = True

            # LED 0
            lightring_msg.leds[0].red = 255
            lightring_msg.leds[0].blue = 0
            lightring_msg.leds[0].green = 0

            # LED 1
            lightring_msg.leds[1].red = 0
            lightring_msg.leds[1].blue = 255
            lightring_msg.leds[1].green = 0

            # LED 2
            lightring_msg.leds[2].red = 0
            lightring_msg.leds[2].blue = 0
            lightring_msg.leds[2].green = 255

            # LED 3
            lightring_msg.leds[3].red = 255
            lightring_msg.leds[3].blue = 255
            lightring_msg.leds[3].green = 0

            # LED 4
            lightring_msg.leds[4].red = 255
            lightring_msg.leds[4].blue = 0
            lightring_msg.leds[4].green = 255

            # LED 5
            lightring_msg.leds[5].red = 0
            lightring_msg.leds[5].blue = 255
            lightring_msg.leds[5].green = 255
        # Lights are currently on
        else:
            # Disable system override. The system will take back control of the lightring.
            lightring_msg.override_system = False

        # Publish the message
        self.lightring_publisher.publish(lightring_msg)
        # Toggle the lights on status
        self.lights_on_ = not self.lights_on_


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4FirstNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```
다음 명령을 통해 `turtlebot4_python_tutorials` 패키지를 빌드한다.  
```
cd ~/turtlebot4_ws
colcon build --symlink-install --packages-select turtlebot4_python_tutorials
source install/local_setup.bash
```
다음 명령을 통해 빌드된 노드를 실행할 수 있다.  
```
ros2 run turtlebot4_python_tutorials turtlebot4_first_python_node
```