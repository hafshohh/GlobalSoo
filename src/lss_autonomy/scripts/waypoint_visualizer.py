#!/usr/bin/env python3
"""
Visualization Markers for Waypoints
Publishes waypoint markers to RViz
"""

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path


class WaypointVisualizer:
    """Visualize waypoints as markers in RViz"""
    
    def __init__(self):
        rospy.init_node('waypoint_visualizer', anonymous=False)
        
        self.pub_markers = rospy.Publisher('/waypoints_markers', MarkerArray, queue_size=10)
        self.sub_waypoints = rospy.Subscriber('/waypoints', Path, self.waypoints_callback, queue_size=1)
        self.sub_global_path = rospy.Subscriber('/planned_path_global', Path, self.global_path_callback, queue_size=1)
        
        self.waypoints = []
        self.global_path = []
        
        rospy.loginfo("Waypoint Visualizer started")
    
    def waypoints_callback(self, msg):
        """Receive waypoints and publish markers"""
        self.waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.publish_waypoint_markers()
    
    def global_path_callback(self, msg):
        """Receive global path"""
        self.global_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    
    def publish_waypoint_markers(self):
        """Publish waypoint markers for RViz"""
        marker_array = MarkerArray()
        
        # Waypoint markers (blue spheres)
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = wp[0]
            marker.pose.position.y = wp[1]
            marker.pose.position.z = 0.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            # Blue color
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(marker)
            
            # Text label
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = "waypoint_labels"
            text_marker.id = 100 + i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = wp[0]
            text_marker.pose.position.y = wp[1]
            text_marker.pose.position.z = 1.0
            
            text_marker.scale.x = 1.0
            text_marker.scale.y = 1.0
            text_marker.scale.z = 1.0
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = "WP{}".format(i)
            text_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(text_marker)
        
        self.pub_markers.publish(marker_array)
    
    def run(self):
        """Main loop"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    try:
        visualizer = WaypointVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
