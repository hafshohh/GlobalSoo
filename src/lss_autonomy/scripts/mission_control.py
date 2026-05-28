#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, String, Bool, Float32
from tf.transformations import euler_from_quaternion


class PathFollower:
    def __init__(self):
        self.path_idx = 0
        self.path_idx_before = 0
        self.path = None
        self.current_pos = Point()
        self.current_yaw = 0.0
        self.waypoint_threshold = 2.0

        rospy.Subscriber('odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        self.current_pos.x = msg.pose.pose.position.x
        self.current_pos.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def set_path(self, path):
        self.path = path
        self.path_idx = 0
    
    def is_arrived_last_WP(self):
        if self.path is None or len(self.path.poses) == 0:
            return False
        return self.path_idx >= len(self.path.poses)
    
    def update(self):
        if self.path is None or len(self.path.poses) == 0:
            return 0.0
        
        if self.is_arrived_last_WP():
            return 0.0
        
        target = self.path.poses[self.path_idx].pose.position
        
        dx = target.x - self.current_pos.x
        dy = target.y - self.current_pos.y
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance < self.waypoint_threshold:
            self.path_idx += 1
            rospy.loginfo(f"Reached waypoint {self.path_idx}/{len(self.path.poses)}")
            if self.is_arrived_last_WP():
                return 0.0
            target = self.path.poses[self.path_idx].pose.position
            dx = target.x - self.current_pos.x
            dy = target.y - self.current_pos.y
        
        desired_yaw = np.arctan2(dy, dx)
        error = desired_yaw - self.current_yaw
        
        while error > np.pi:
            error -= 2 * np.pi
        while error < -np.pi:
            error += 2 * np.pi
        
        return error


class MissionManagerNode:
    def __init__(self):
        rospy.init_node('mission_manager_node', anonymous=False)
        
        self.path_subscriber = rospy.Subscriber('/waypoints/mission_1/in', Path, self.cb_path, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/autonomy/mission_manager/state_cmd', String, self.cmd_cb, queue_size=10)
        self.obstacle_sub = rospy.Subscriber('/autonomy/obstacle_status', Bool, self.obstacle_cb, queue_size=10)
        self.speed_sub = rospy.Subscriber('/autonomy/mission_manager/speed', Float64, self.speed_cb, queue_size=10)
        
        # NEW: Subscribe to planned paths and control signals from path planner
        self.sub_global_path = rospy.Subscriber('/planned_path_global', Path, self.global_path_callback, queue_size=1)
        self.sub_desired_heading = rospy.Subscriber('/desired_heading', Float32, self.desired_heading_callback, queue_size=1)
        
        self.err_pub = rospy.Publisher('/autonomy/mission_manager/error', Float64, queue_size=1)
        self.trg_pub = rospy.Publisher('/autonomy/mission_manager/target', Float64, queue_size=1)
        self.state_pub = rospy.Publisher('/autonomy/mission_manager/state', String, queue_size=1)
        self.mode_pub = rospy.Publisher('/autonomy/mission_manager/mode', String, queue_size=1)
        
        self.is_started = False
        self.obstacle_detected = False
        self.speed = 0.0
        self.path_follower = PathFollower()
        self.error = 0.0
        
        # NEW: State variables for path planner integration
        self.global_path = []
        self.desired_heading = 0.0
        
        self.timer = rospy.Timer(rospy.Duration(1.0 / 15.0), self.update)
    
    def obstacle_cb(self, msg):
        self.obstacle_detected = msg.data
        if self.obstacle_detected:
            rospy.logwarn_throttle(2.0, "Obstacle detected - switching to avoidance mode")
    
    def speed_cb(self, msg):
        self.speed = msg.data
        rospy.loginfo(f"Speed updated to: {self.speed}")
        
    def cmd_cb(self, msg):
        cmd = msg.data
        if cmd == 'start':
            self.is_started = True
        elif cmd == 'stop':
            self.is_started = False
        elif cmd == 'restart':
            self.restart()
            
        state_msg = String()
        state_msg.data = 'running' if self.is_started else 'stopped'
        self.state_pub.publish(state_msg)

    def restart(self):
        self.path_follower.path_idx = 1
        self.path_follower.path_idx_before = 0
        rospy.loginfo("Mission Restart")
    
    def update(self, event):
        if not self.is_started:
            return
        
        # Priority: Obstacle avoidance > Path following
        # Saat obstacle terdeteksi, mission_control DIAM (tidak publish ke motor)
        # obstacle_avoidance.py yang langsung mengendalikan motor via /autonomy/pathfollowing
        if self.obstacle_detected: # changed
            mode_msg = String()
            mode_msg.data = 'vision'
            self.mode_pub.publish(mode_msg)
            return
        
        # Publish GPS mode
        mode_msg = String()
        mode_msg.data = 'gps'
        self.mode_pub.publish(mode_msg)

        # Kontrol motor didelegasikan ke path_planner (D* Lite + ILOS)
        # mission_control hanya publish error untuk monitoring
        self.error = self.path_follower.update()
        err = Float64()
        err.data = self.error
        self.err_pub.publish(err)
    
    def cb_path(self, msg):
        self.path_follower.set_path(msg)
        self.path_follower.path_idx_before = 0
        rospy.loginfo('Path Received')
    
    def global_path_callback(self, msg):
        """NEW: Receive planned path from D* Lite path planner"""
        try:
            self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            if len(self.global_path) > 0:
                rospy.logdebug("Mission: Global path updated - {} waypoints".format(len(self.global_path)))
        except Exception as e:
            rospy.logwarn("Global path callback error: {}".format(e))
    
    def desired_heading_callback(self, msg):
        """NEW: Receive desired heading from ILOS path follower"""
        try:
            self.desired_heading = msg.data
        except Exception as e:
            rospy.logwarn("Desired heading callback error: {}".format(e))


def main():
    try:
        node = MissionManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()



# if self.obstacle_detected and self.obstacle_cmd is not None:
#             self.pub.publish(self.obstacle_cmd)