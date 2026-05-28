#!/usr/bin/env python3
"""
sim_odom_bridge — converts /odom (from simulator) to the topics
expected by the Web UI: /tf_simple/pose and /tf_simple/yaw.
Same role as boat_tf.py in the real USV, but reads from the simulator.
"""

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64


def quat_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class SimOdomBridge:
    def __init__(self):
        rospy.init_node('sim_odom_bridge', anonymous=False)
        self.pub_pose = rospy.Publisher('/tf_simple/pose', Point, queue_size=1)
        self.pub_yaw  = rospy.Publisher('/tf_simple/yaw',  Float64, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self._cb)
        rospy.loginfo("[SimBridge] Siap: /odom → /tf_simple/pose + /tf_simple/yaw")

    def _cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.pub_pose.publish(Point(x=p.x, y=p.y, z=0.0))
        self.pub_yaw.publish(Float64(data=yaw))

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    SimOdomBridge().spin()
