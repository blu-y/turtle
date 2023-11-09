
```
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2


class LiDARSubscriber(Node) :
   def __init__(self) :
     super().__init__('lidar_subscriber')
     self.lidar_sub = self.create_subscription(
        LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)

   def lidar_cb(self, msg) :
     self.lidar = msg.ranges
     self.lidar = np.array(self.lidar)
     self.lidar = np.resize(self.lidar, (20, 720))
     cv2.imshow('img', self.lidar)
     cv2.waitKey(1)
     #print(self.lidar)
     
def main(args=None) :
  rclpy.init(args=args)
  node = LiDARSubscriber()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__' :
  main()

```
