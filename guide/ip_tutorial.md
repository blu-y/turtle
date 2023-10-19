## Image Processing Tutorial

### Rviz2
`view_robot` launch
```
ros2 launch turtlebot4_viz view_robot.launch.py
```
Find camera topic `/oakd/rgb/preview/image_raw`


### CV_Bridge
In python use cv_bridge to make msg to array

make `image_subscriber.py` file
```
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge


class ImageSubscriber(Node) :
   def __init__(self) :
     super().__init__('image_subscriber')
     self.bridge = CvBridge() 
     self.image_sub = self.create_subscription(
        Image, '/oakd/rgb/preview/image_raw', self.image_cb, qos_profile_sensor_data)
     self.image = []

   def image_cb(self, msg) :
     self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
     cv2.imshow('img', self.image)
     cv2.waitKey(1)
     
def main(args=None) :
  rclpy.init(args=args)
  node = ImageSubscriber()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__' :
  main()
```
Run Python file
```
python3 image_subscriber.py
```

### Basic Image Processing

#### Array-like image
image as an array (coordinage, color)

#### Color Space
RGB, HSV, YUV

#### Histogram
다음 

#### Filter, Edge

#### DL
Yolo, RCNN, ViT, SwinT

#### OpenCV

##### I/O Image, Gray Scaling
Import image, grayscale, show and save.
```
import cv2
import numpy as np

def main(image_path):
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('Original', img)
    cv2.imshow('Grayscale', gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    filename = 'gray_' + image_path
    cv2.imwrite(filename, gray)

if __name__ == "__main__":
    path_to_image = "test1.png"
    main(path_to_image)
```

##### Canny Edge & Hough Line Detector
Detect edge, detect lines.
```
import cv2
import numpy as np

def main(image_path):
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    canny = cv2.Canny(gray, 50, 150, apertureSize=3)
    lines = cv2.HoughLines(canny, 1, np.pi / 180, 100)
    if lines is not None:
        for rho, theta in lines[:, 0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000 * (-b))
            y1 = int(y0 + 1000 * (a))
            x2 = int(x0 - 1000 * (-b))
            y2 = int(y0 - 1000 * (a))
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)

    cv2.imshow('Canny', canny)
    cv2.imshow('Hough Lines', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    path_to_image = "test1.png"
    main(path_to_image)
```

##### Harris Cornor Detector
Detect corners.
```
import cv2
import numpy as np

def main(image_path):
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray, blockSize=2, ksize=3, k=0.04)
    dst = cv2.dilate(dst, None)
    img[dst > 0.1 * dst.max()] = [0, 0, 255]
    cv2.imshow('Harris Corners', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    path_to_image = "test1.png"
    main(path_to_image)
```

##### SIFT
Matching two images.
```
import cv2
import numpy as np

def main(image_path1, image_path2):
    # Step 1: Read the two images
    img1 = cv2.imread(image_path1, cv2.IMREAD_GRAYSCALE)  # queryImage
    img2 = cv2.imread(image_path2, cv2.IMREAD_GRAYSCALE)  # trainImage

    sift = cv2.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)
    index_params = dict(algorithm=1, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    matches = flann.knnMatch(des1, des2, k=2)

    good_matches = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good_matches.append(m)
    img_matches = np.empty((max(img1.shape[0], img2.shape[0]), img1.shape[1] + img2.shape[1], 3), dtype=np.uint8)

    cv2.drawMatches(img1, kp1, img2, kp2, good_matches, img_matches, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imshow('Matches', img_matches)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    path_to_image_1 = "test1.png"
    path_to_image_2 = "test2.png"
    main(path_to_image_1, path_to_image_2)
```