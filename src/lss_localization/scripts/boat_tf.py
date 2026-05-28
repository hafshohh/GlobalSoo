# #!/usr/bin/env python

# import rospy
# import tf2_ros
# import geometry_msgs.msg
# import numpy as np
# from sensor_msgs.msg import NavSatFix
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Float64, Float64MultiArray
# from geometry_msgs.msg import Point


# class BoatLocalization:
#     def __init__(self):
#         rospy.init_node('boat_localization', anonymous=False)
        
#         # Map boundaries (D8 ITS)
#         self.lat_top = -7.286040
#         self.long_top = 112.794951
#         self.lat_bottom = -7.287745
#         self.long_bottom = 112.797922
        
#         self.lat_center = (self.lat_top + self.lat_bottom) / 2
#         self.long_center = (self.long_top + self.long_bottom) / 2
        
#         self.LON_TO_METER = 113321
#         self.LAT_TO_METER = 111000
        
#         self.heading = 0.0
        
#         # Publishers
#         self.pose_pub = rospy.Publisher('/tf_simple/pose', Point, queue_size=10)
#         self.yaw_pub = rospy.Publisher('/tf_simple/yaw', Float64, queue_size=10)
#         self.odom_pub = rospy.Publisher('/asv/odom', Odometry, queue_size=10)
#         self.map_corner_pub = rospy.Publisher('/map/corners', Float64MultiArray, queue_size=10)
        
#         self.br = tf2_ros.TransformBroadcaster()
        
#         # Subscribers
#         rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
#         rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.compass_callback)
        
#         # Publish map corners once
#         self.publish_map_corners()
    
#     def publish_map_corners(self):
#         msg = Float64MultiArray()
#         msg.data = [self.lat_top, self.long_top, self.lat_bottom, self.long_bottom]
#         self.map_corner_pub.publish(msg)
    
#     def global_to_local(self, lat, lon):
#         y = self.LON_TO_METER * (self.long_center - lon)
#         x = self.LAT_TO_METER * (self.lat_center - lat)
#         return -y, -x
    
#     def get_quaternion(self, yaw):
#         qx = 0.0
#         qy = 0.0
#         qz = np.sin(yaw / 2)
#         qw = np.cos(yaw / 2)
#         return [qx, qy, qz, qw]
    
#     def compass_callback(self, msg):
#         angle = msg.data - 90.0
#         if -angle <= -180:
#             angle = 360 - angle
#         else:
#             angle = -angle
        
#         self.heading = angle * np.pi / 180
#         self.yaw_pub.publish(self.heading)
    
#     def gps_callback(self, msg):
#         # Validate GPS fix
#         if msg.status.status < 0:
#             rospy.logwarn_throttle(5, "GPS fix not available")
#             return
        
#         # Validate coordinates are reasonable
#         if abs(msg.latitude) > 90 or abs(msg.longitude) > 180:
#             rospy.logwarn_throttle(5, "Invalid GPS coordinates")
#             return
        
#         x, y = self.global_to_local(msg.latitude, msg.longitude)
        
#         # Publish simple pose
#         pose_msg = Point()
#         pose_msg.x = x
#         pose_msg.y = y
#         pose_msg.z = 0
#         self.pose_pub.publish(pose_msg)
        
#         # TF: map -> asv/odom
#         t = geometry_msgs.msg.TransformStamped()
#         t.header.stamp = rospy.Time.now()
#         t.header.frame_id = "map"
#         t.child_frame_id = "asv/odom"
#         t.transform.translation.x = x
#         t.transform.translation.y = y
#         t.transform.translation.z = 0.0
#         q = self.get_quaternion(self.heading)
#         t.transform.rotation.x = q[0]
#         t.transform.rotation.y = q[1]
#         t.transform.rotation.z = q[2]
#         t.transform.rotation.w = q[3]
#         self.br.sendTransform(t)
        
#         # TF: asv/odom -> asv/base_link
#         t2 = geometry_msgs.msg.TransformStamped()
#         t2.header.stamp = rospy.Time.now()
#         t2.header.frame_id = "asv/odom"
#         t2.child_frame_id = "asv/base_link"
#         t2.transform.translation.x = 0
#         t2.transform.translation.y = 0
#         t2.transform.translation.z = 0.0
#         q2 = self.get_quaternion(0)
#         t2.transform.rotation.x = q2[0]
#         t2.transform.rotation.y = q2[1]
#         t2.transform.rotation.z = q2[2]
#         t2.transform.rotation.w = q2[3]
#         self.br.sendTransform(t2)
        
#         # TF: asv/base_link -> asv/laser_link
#         t3 = geometry_msgs.msg.TransformStamped()
#         t3.header.stamp = rospy.Time.now()
#         t3.header.frame_id = "asv/base_link"
#         t3.child_frame_id = "asv/laser_link"
#         t3.transform.translation.x = 0
#         t3.transform.translation.y = 0
#         t3.transform.translation.z = 0.0
#         q3 = self.get_quaternion(np.pi / 2)
#         t3.transform.rotation.x = q3[0]
#         t3.transform.rotation.y = q3[1]
#         t3.transform.rotation.z = q3[2]
#         t3.transform.rotation.w = q3[3]
#         self.br.sendTransform(t3)
        
#         # Publish Odometry
#         odom = Odometry()
#         odom.header.stamp = rospy.Time.now()
#         odom.header.frame_id = "asv/odom"
#         odom.child_frame_id = "asv/base_link"
#         odom.pose.pose.position.x = x
#         odom.pose.pose.position.y = y
#         odom.pose.pose.position.z = 0
#         odom.pose.pose.orientation.x = q[0]
#         odom.pose.pose.orientation.y = q[1]
#         odom.pose.pose.orientation.z = q[2]
#         odom.pose.pose.orientation.w = q[3]
#         self.odom_pub.publish(odom)


# def main():
#     try:
#         node = BoatLocalization()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass


# if __name__ == '__main__':
#     main()


#!/usr/bin/python3

import rospy

# Because of transformations
#import tf_conversions
#from squaternion import Quaternion

import tf2_ros
from tf import TransformBroadcaster
import geometry_msgs.msg
import os

from geometry_msgs.msg import Point

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

import numpy as np
import rospkg

heading_asv = 0

rospy.init_node('tf2_asv_broadcaster')
  

pose_tf_pub = rospy.Publisher('/tf_simple/pose', Point, queue_size=10)
yaw_tf_pub = rospy.Publisher('/tf_simple/yaw', Float64, queue_size=10)
odom_tf_pub = rospy.Publisher('/asv/odom', Odometry, queue_size=10)
map_corner_pub = rospy.Publisher('/map/corners',Float64MultiArray, queue_size=10)

import numpy as np # Scientific computing library for Python




def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def global_pos_to_local(lat_, long_):
    #dir_package = rospkg.get_path('boat_tf')
    #file = os.open(dir_package + '/data/reed_canal.txt')
    #lat_top = float(file[0].split(',')[0].strip())
    #long_top = float(file[0].split(',')[1].strip())
    #lat_bottom = float(file[1].split(',')[0].strip())
    #long_bottom = float(file[1].split(',')[1].strip())
    #width = float(file[2].split(' ')[0].strip())
    #height = float(file[2].split(' ')[1].strip())

    #d8
    lat_top = -7.286040
    long_top = 112.794951
    lat_bottom = -7.287745
    long_bottom = 112.797922
    #reed
    #lat_top =  29.153113537
    #long_top = -81.019343158 
    #lat_bottom = 29.149281665
    #long_bottom = -81.013523513
    #nben
    # lat_top = 27.375589550
    # long_top = -82.455675285
    # lat_bottom = 27.373069704
    # long_bottom = -82.451911558
    map_corner_msg = Float64MultiArray()
    map_corner_msg.data = [lat_top,long_top,lat_bottom,long_bottom]
    map_corner_pub.publish(map_corner_msg)

    lat_center = (lat_top + lat_bottom) / 2
    long_center = (long_top + long_bottom) / 2

    print(lat_center, long_center)

    LON_TO_METER = 113321
    LAT_TO_METER = 111000

    offsetX = 0
    offsetY = 0

    y = LON_TO_METER * (long_center - long_) - offsetY
    x = LAT_TO_METER * (lat_center - lat_) - offsetX

    return y * -1, x * -1

def cb_magnetic_field(mag : MagneticField):
    vec = mag.magnetic_field
    angle = np.arctan2(vec.y,vec.x)
    angle *= 180/np.pi
    floatmsg = Float64()
    floatmsg.data = angle
    cb_compass_mavros(floatmsg)
    pass
def cb_compass_mavros(angle_):
    global heading_asv, yaw_tf_pub
    # heading_asv = -angle_.data
    tmp_0 = angle_.data  - 90.0
    tmp_a = 0
    if -tmp_0 <= -180:
        tmp_a = 360 - tmp_0
        # heading_asv *= -1
    else:
        tmp_a = -tmp_0
    


    print("HEADING",tmp_a)
    heading_asv = tmp_a * np.pi / 180
    yaw_tf_pub.publish(heading_asv)


def handle_robot_pose(msg, robot_name):
    global heading_asv, pose_tf_pub

    print("get topic")
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'asv/odom'
    t.child_frame_id = 'asv/base_link'
    # t.transform.translation.x, t.transform.translation.y = global_pos_to_local(msg.latitude, msg.longitude)
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0.0
    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, np.pi * heading_asv / 180)
    q = get_quaternion_from_euler(0,0,0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    tmp_point = Point()
    tmp_point.x, tmp_point.y= global_pos_to_local(msg.latitude, msg.longitude)
    tmp_point.z = 0

    pose_tf_pub.publish(tmp_point)

    # //first, we'll publish the transform over tf
    # geometry_msgs::TransformStamped odom_trans;
    # odom_trans.header.stamp = current_time;
    # odom_trans.header.frame_id = "odom";
    # odom_trans.child_frame_id = "base_link";

    # odom_trans.transform.translation.x = x;
    # odom_trans.transform.translation.y = y;
    # odom_trans.transform.translation.z = 0.0;
    # odom_trans.transform.rotation = odom_quat;

    # //send the transform
    # odom_broadcaster.sendTransform(odom_trans);

    odom_t = geometry_msgs.msg.TransformStamped()
    
    odom_t.header.stamp = rospy.Time.now()
    odom_t.header.frame_id = "map"
    odom_t.child_frame_id = 'asv/odom'
    odom_t.transform.translation.x, odom_t.transform.translation.y = global_pos_to_local(msg.latitude, msg.longitude)
    odom_t.transform.translation.z = 0.0
    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, np.pi * heading_asv / 180)
    q = get_quaternion_from_euler(0,0,heading_asv)
    odom_t.transform.rotation.x = q[0]
    odom_t.transform.rotation.y = q[1]
    odom_t.transform.rotation.z = q[2]
    odom_t.transform.rotation.w = q[3]

    br.sendTransform(odom_t)


    lidar_t = geometry_msgs.msg.TransformStamped()
    lidar_t.header.stamp = rospy.Time.now()
    lidar_t.header.frame_id = "asv/base_link"
    lidar_t.child_frame_id = 'asv/laser_link'
    lidar_t.transform.translation.x = 0
    lidar_t.transform.translation.y = 0
    lidar_t.transform.translation.z = 0.0
    #q = tf_conversions.transformations.quaternion_from_euler(0, 0, np.pi * heading_asv / 180)
    q = get_quaternion_from_euler(0,0,np.pi/2)#np.pi/2)
    lidar_t.transform.rotation.x = q[0]
    lidar_t.transform.rotation.y = q[1]
    lidar_t.transform.rotation.z = q[2]
    lidar_t.transform.rotation.w = q[3]
    
    br.sendTransform(lidar_t)

    print(rospy.Time.now())

    odom_msg = Odometry()
    odom_msg.header.stamp.secs = rospy.Time.now().secs
    odom_msg.header.stamp.nsecs = rospy.Time.now().nsecs
    odom_msg.header.frame_id = 'asv/odom'
    odom_msg.child_frame_id = 'asv/base_link'
    odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y = global_pos_to_local(msg.latitude, msg.longitude)
    
    odom_msg.pose.pose.position.z = 0
    odom_msg.pose.pose.orientation.x = q[0]
    odom_msg.pose.pose.orientation.y = q[1]
    odom_msg.pose.pose.orientation.z = q[2]
    odom_msg.pose.pose.orientation.w = q[3]

    odom_tf_pub.publish(odom_msg)

    

robot_name = 'asv'
rospy.Subscriber('/mavros/global_position/global',
                    NavSatFix,
                    handle_robot_pose,
                    robot_name)
rospy.Subscriber('/mavros/global_position/compass_hdg',
       Float64,
       cb_compass_mavros)      
# rospy.Subscriber('/mavros/imu/mag',MagneticField,cb_magnetic_field)
if __name__ == '__main__':
    rospy.spin()