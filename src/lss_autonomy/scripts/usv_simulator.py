#!/usr/bin/env python3
"""
USV Simulator Node
Simulates USV movement and odometry
"""

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf.transformations import quaternion_from_euler
import tf2_ros


class USVSimulator:
    """Simulates USV movement based on control commands"""
    
    def __init__(self):
        rospy.init_node('usv_simulator', anonymous=False)
        
        # USV parameters
        self.usv_length = 1.6
        self.max_surge = 1.0  # m/s
        self.max_yaw_rate = 1.0  # rad/s
        
        # Simulation state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Control inputs
        self.surge = 0.0
        self.yaw_rate = 0.0
        
        # ROS interface
        self.sub_cmd_vel = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # Simulation parameters
        self.dt = 0.05  # 20 Hz
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.update_simulation)
        
        rospy.loginfo("USV Simulator started at position (%.2f, %.2f, %.2f)", self.x, self.y, self.yaw)
    
    def cmd_vel_callback(self, msg):
        """Receive control commands"""
        self.surge = np.clip(msg.linear.x, -self.max_surge, self.max_surge)
        self.yaw_rate = np.clip(msg.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
    
    def update_simulation(self, event):
        """Update USV position and publish odometry"""
        # Simple kinematic model (unicycle)
        self.x += self.surge * np.cos(self.yaw) * self.dt
        self.y += self.surge * np.sin(self.yaw) * self.dt
        self.yaw += self.yaw_rate * self.dt
        
        # Normalize yaw to [-pi, pi]
        while self.yaw > np.pi:
            self.yaw -= 2 * np.pi
        while self.yaw < -np.pi:
            self.yaw += 2 * np.pi
        
        # Publish odometry
        self.publish_odom()
        
        # Publish TF
        self.publish_tf()
    
    def publish_odom(self):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation
        quat = quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation = Quaternion(*quat)
        
        # Velocity
        odom.twist.twist.linear.x = self.surge
        odom.twist.twist.angular.z = self.yaw_rate
        
        # Covariance (not used in simulation)
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        
        self.pub_odom.publish(odom)
    
    def publish_tf(self):
        """Publish TF from map to base_link"""
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        
        # Translation
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        # Rotation
        quat = quaternion_from_euler(0, 0, self.yaw)
        transform.transform.rotation = Quaternion(*quat)
        
        self.tf_broadcaster.sendTransform(transform)


if __name__ == '__main__':
    try:
        simulator = USVSimulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
