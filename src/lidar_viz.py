#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import cv2
import matplotlib.cm as cm

class LiDARSubscriber(Node) :
   def __init__(self) :
     super().__init__('lidar_subscriber')
     self.lidar_sub = self.create_subscription(
        LaserScan, '/scan', self.lidar_cb, qos_profile_sensor_data)
     self.length = 0
     self.res = 40
     self.cmap = cm.get_cmap('jet', 256)

   def lidar_cb(self, msg) :
     self.lidar = msg.ranges
     if self.length == 0:
       self.inc = msg.angle_increment
       self.m = msg.range_max
       self.length = len(self.lidar)
       self.ind = np.arange(self.length)
       self.size = int(self.m*2*self.res)
     self.lidar = np.array(self.lidar)
     self.lidar[self.lidar==np.inf] = 0

     self.viz = np.zeros((self.size, self.size, 3))*255
     # x coordinate
     x_c = np.cos(self.inc*self.ind)*self.lidar*self.size/self.m+self.size/2
     y_c = -np.sin(self.inc*self.ind)*self.lidar*self.size/self.m+self.size/2
     for i in range(self.length):
       cv2.circle(self.viz,(int(x_c[i]),int(y_c[i])),3,self.cm(self.lidar[i]),-1)
     self.lidar = np.resize(self.lidar, (20, self.length))
     cv2.imshow('img', self.lidar)
     cv2.imshow('viz', self.viz)
     cv2.waitKey(1)
     #print(self.lidar)

   def cm(self, value):
     color = self.cmap(value / self.m)
     rgb_color = tuple([int(255 * c) for c in color[:3]])
     return rgb_color
     
def main(args=None) :
  rclpy.init(args=args)
  node = LiDARSubscriber()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__' :
  main()
