#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class ManualControl:
    def __init__(self):
        rospy.init_node('manual_control', anonymous=False)
        self.pub = rospy.Publisher('/autonomy/pathfollowing', Twist, queue_size=1)
        
        self.speed = 0.0
        self.angle = 0.0
        self.speed_step = 0.1
        self.angle_step = 0.2
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        print("\n=== Manual Control ===")
        print("W/S: Forward/Backward")
        print("A/D: Left/Right")
        print("X: Stop")
        print("Q: Quit")
        print("======================\n")
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key == 'w':
                    self.speed = min(self.speed + self.speed_step, 1.0)
                elif key == 's':
                    self.speed = max(self.speed - self.speed_step, -1.0)
                elif key == 'a':
                    self.angle = min(self.angle + self.angle_step, 1.57)
                elif key == 'd':
                    self.angle = max(self.angle - self.angle_step, -1.57)
                elif key == 'x':
                    self.speed = 0.0
                    self.angle = 0.0
                elif key == 'q':
                    break
                
                msg = Twist()
                msg.linear.x = self.speed
                msg.angular.z = self.angle
                self.pub.publish(msg)
                
                print(f"\rSpeed: {self.speed:.2f} | Angle: {self.angle:.2f}  ", end='')
                
                rate.sleep()
        except Exception as e:
            print(e)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            msg = Twist()
            self.pub.publish(msg)
            print("\nStopped")

def main():
    try:
        controller = ManualControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
