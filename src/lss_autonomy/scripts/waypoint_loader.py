#!/usr/bin/python3
import rospy
import yaml
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool


def global_pos_to_local(lat_, long_):
    """Convert GPS coordinates to local XY coordinates"""
    lat_top = -7.286040
    long_top = 112.794951
    lat_bottom = -7.287745
    long_bottom = 112.797922

    lat_center = (lat_top + lat_bottom) / 2
    long_center = (long_top + long_bottom) / 2

    LON_TO_METER = 113321
    LAT_TO_METER = 111000

    y = LON_TO_METER * (long_center - long_)
    x = LAT_TO_METER * (lat_center - lat_)

    return y * -1, x * -1


def local_to_global(x, y):
    """Convert local XY coordinates to GPS lat/lon"""
    lat_top = -7.286040
    long_top = 112.794951
    lat_bottom = -7.287745
    long_bottom = 112.797922

    lat_center = (lat_top + lat_bottom) / 2
    long_center = (long_top + long_bottom) / 2

    LON_TO_METER = 113321
    LAT_TO_METER = 111000

    # Reverse of global_pos_to_local
    lon = long_center - (-x / LON_TO_METER)
    lat = lat_center - (-y / LAT_TO_METER)

    return lat, lon


class WaypointLoader:
    def __init__(self):
        rospy.init_node('waypoint_loader', anonymous=False)
        
        self.path_pub = rospy.Publisher('/waypoints/mission_1/in', Path, queue_size=10, latch=True)
        self.status_pub = rospy.Publisher('/waypoints/loader/status', String, queue_size=10)
        
        self.yaml_file = rospy.get_param('~waypoints_file', 
                                          os.path.join(os.path.dirname(__file__), '../config/waypoints.yaml'))
        
        # Subscribe to path from web UI to save to YAML
        self.path_sub = rospy.Subscriber('/waypoints/save_request', Path, self.save_waypoints_cb)

        self.load_and_publish(self.yaml_file, delay=True)
    
    def load_and_publish(self, yaml_file, delay=False):
        try:
            with open(yaml_file, 'r') as f:
                data = yaml.safe_load(f)

            waypoints = data.get('waypoints', [])

            if not waypoints or (len(waypoints) == 1 and waypoints[0]['lat'] == 0.0):
                rospy.logwarn("No valid waypoints in YAML file")
                return

            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"

            for wp in waypoints:
                lat = wp['lat']
                lon = wp['lon']

                x, y = global_pos_to_local(lat, lon)

                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0

                path_msg.poses.append(pose)
                rospy.loginfo(f"Waypoint: lat={lat}, lon={lon} -> x={x:.2f}, y={y:.2f}")

            # Delay hanya saat node baru launch (tunggu subscriber siap)
            if delay:
                rospy.sleep(1.0)
            self.path_pub.publish(path_msg)
            
            status_msg = String()
            status_msg.data = f"Loaded {len(waypoints)} waypoints"
            self.status_pub.publish(status_msg)
            
            rospy.loginfo(f"Published {len(waypoints)} waypoints to /waypoints/mission_1/in")
            
        except Exception as e:
            rospy.logerr(f"Failed to load waypoints: {e}")
    
    def save_waypoints_cb(self, msg):
        """Save waypoints from Path message to YAML file"""
        try:
            waypoints = []
            for pose in msg.poses:
                x = pose.pose.position.x
                y = pose.pose.position.y
                lat, lon = local_to_global(x, y)
                waypoints.append({'lat': lat, 'lon': lon})
            
            data = {'waypoints': waypoints}
            
            with open(self.yaml_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            
            rospy.loginfo(f"Saved {len(waypoints)} waypoints to {self.yaml_file}")

            # Auto-reload dan publish tanpa delay (subscriber sudah siap)
            self.load_and_publish(self.yaml_file, delay=False)
            
            status_msg = String()
            status_msg.data = f"Saved & published {len(waypoints)} waypoints"
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            rospy.logerr(f"Failed to save waypoints: {e}")


def main():
    try:
        loader = WaypointLoader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
