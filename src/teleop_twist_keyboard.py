#!/usr/bin/env python3
import sys
import threading
from geometry_msgs.msg import Twist
import rclpy
import termios
import tty
settings = termios.tcgetattr(sys.stdin)

msg = """
---------------------------
Moving around:
   u(↖)  i(↑)  o(↗)
   j(←)   k    l(→)

anything else : stop

CTRL-C to quit
"""

moveBindings = {'i':(1,0,0,0), 'o':(1,0,0,-1), 'j':(0,0,0,1), 'l':(0,0,0,-1), 'u':(1,0,0,1),}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rclpy.init()
    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()
    speed = 0.5; turn = 1.0; x = 0; y = 0; z = 0; th = 0; status = 0
    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            else:
                x = 0; y = 0; z = 0; th = 0
                if (key == '\x03'):
                    break
            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
            pub.publish(twist)
    except Exception as e:
        print(e)
    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
        rclpy.shutdown()
        spinner.join()

if __name__ == '__main__':
    main()
