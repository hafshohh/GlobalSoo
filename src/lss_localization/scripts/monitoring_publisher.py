#!/usr/bin/env python3
"""
Telemetry Publisher Node
========================
Aggregates MAVROS sensor data into a single JSON topic for the web dashboard.

Published topic:
  /telemetry/nav_status  (std_msgs/String)  — JSON with:
      sog, cog, heading, ground_speed, roll, pitch, yaw

Subscribed topics (MAVROS):
  /mavros/global_position/compass_hdg   (std_msgs/Float64)
  /mavros/global_position/raw/gps_vel   (geometry_msgs/TwistStamped)
  /mavros/imu/data                      (sensor_msgs/Imu)
  /mavros/global_position/global        (sensor_msgs/NavSatFix)
  /mavros/vfr_hud                       (mavros_msgs/VFR_HUD)
"""

import rospy
import json
import math
import numpy as np
from std_msgs.msg import String, Float64, Float32
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

try:
    from mavros_msgs.msg import VFR_HUD
    HAS_VFR_HUD = True
except ImportError:
    HAS_VFR_HUD = False


class TelemetryPublisher:
    def __init__(self):
        rospy.init_node('telemetry_publisher', anonymous=False)

        # ----- Internal state -----
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.sog = 0.0
        self.cog = 0.0
        self.ground_speed = 0.0
        self.vel_east = 0.0
        self.vel_north = 0.0

        # ----- Publisher -----
        self.pub = rospy.Publisher(
            '/telemetry/nav_status', String, queue_size=10)

        # ----- Source selection -----
        source = rospy.get_param('~source', 'mavros')

        if source == 'sim':
            rospy.loginfo("Telemetry publisher: source=sim (/odom + /usv/roll)")
            rospy.Subscriber('/odom', Odometry, self.odom_cb)
            rospy.Subscriber('/usv/roll', Float32, self.roll_cb)
        else:
            rospy.loginfo("Telemetry publisher: source=mavros")
            rospy.Subscriber(
                '/mavros/global_position/compass_hdg',
                Float64, self.compass_cb)
            rospy.Subscriber(
                '/mavros/global_position/raw/gps_vel',
                TwistStamped, self.gps_vel_cb)
            rospy.Subscriber(
                '/mavros/local_position/velocity_local',
                TwistStamped, self.local_vel_cb)
            rospy.Subscriber('/mavros/imu/data', Imu, self.imu_cb)
            if HAS_VFR_HUD:
                rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, self.vfr_hud_cb)

        self.rate = rospy.get_param('~rate', 10)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.rate), self.publish_telemetry)

        rospy.loginfo("Telemetry publisher started at %d Hz", self.rate)

    # ------------------------------------------------------------------ #
    #  Callbacks                                                          #
    # ------------------------------------------------------------------ #

    def odom_cb(self, msg):
        """Simulation: extract telemetry from /odom (Odometry)."""
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.pitch = math.degrees(pitch)
        self.yaw = math.degrees(yaw)

        # Heading: ROS yaw=0 → East; compass heading 0=North CW
        self.heading = (90.0 - math.degrees(yaw)) % 360.0

        # Body-frame velocities (surge u, sway v) → world-frame for COG/SOG
        u = msg.twist.twist.linear.x
        v = msg.twist.twist.linear.y
        vx_world = u * math.cos(yaw) - v * math.sin(yaw)  # East
        vy_world = u * math.sin(yaw) + v * math.cos(yaw)  # North

        self.sog = math.sqrt(vx_world ** 2 + vy_world ** 2)
        self.ground_speed = self.sog
        if self.sog > 0.05:
            self.cog = math.degrees(math.atan2(vx_world, vy_world)) % 360.0

    def roll_cb(self, msg):
        """Simulation: roll angle from /usv/roll (Float32, radians)."""
        self.roll = math.degrees(msg.data)

    def compass_cb(self, msg):
        """Compass heading in degrees 0-360."""
        self.heading = msg.data

    def gps_vel_cb(self, msg):
        """GPS velocity — compute SOG and COG from velocity components."""
        vx = msg.twist.linear.x  # East
        vy = msg.twist.linear.y  # North
        vz = msg.twist.linear.z  # Up

        self.vel_east = vx
        self.vel_north = vy

        # SOG = horizontal speed magnitude
        self.sog = math.sqrt(vx ** 2 + vy ** 2)

        # COG = direction of travel (degrees, 0 = North, CW positive)
        if self.sog > 0.1:  # Only meaningful above threshold
            cog_rad = math.atan2(vx, vy)  # atan2(east, north)
            self.cog = math.degrees(cog_rad) % 360
        # else keep previous COG

    def local_vel_cb(self, msg):
        """Local velocity fallback — compute ground speed."""
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.ground_speed = math.sqrt(vx ** 2 + vy ** 2)

    def imu_cb(self, msg):
        """IMU orientation quaternion → roll, pitch, yaw."""
        q = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.roll = math.degrees(roll)
        self.pitch = math.degrees(pitch)
        self.yaw = math.degrees(yaw)

    def vfr_hud_cb(self, msg):
        """VFR HUD gives a ready-made groundspeed and heading."""
        self.ground_speed = msg.groundspeed
        self.heading = msg.heading

    # ------------------------------------------------------------------ #
    #  Helpers                                                            #
    # ------------------------------------------------------------------ #

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw) in radians."""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    # ------------------------------------------------------------------ #
    #  Publisher                                                          #
    # ------------------------------------------------------------------ #

    def publish_telemetry(self, event):
        """Publish an aggregated JSON string at fixed rate."""
        # Use ground_speed from VFR_HUD if available, else SOG
        gs = self.ground_speed if self.ground_speed > 0 else self.sog

        data = {
            'sog': round(self.sog, 2),
            'cog': round(self.cog, 1),
            'heading': round(self.heading, 1),
            'ground_speed': round(gs, 2),
            'roll': round(self.roll, 1),
            'pitch': round(self.pitch, 1),
            'yaw': round(self.yaw, 1),
        }

        msg = String()
        msg.data = json.dumps(data)
        self.pub.publish(msg)


def main():
    try:
        node = TelemetryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
