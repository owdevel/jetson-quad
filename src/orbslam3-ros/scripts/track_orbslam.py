#!/usr/bin/env python3
"""
Python3 setup with catkin_virtualenv
"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped,Point

def callback(data):
    add_point = Point()
    add_point.x = data.pose.position.x
    add_point.y = data.pose.position.y
    add_point.z = data.pose.position.z
    rospy.loginfo('Publishing Marker Point')
    marker.points.append(add_point)
    # Publish the Marker
    pub_point.publish(marker)
    #rospy.sleep(5)


marker = Marker()
marker.header.frame_id = "/map"
marker.type = marker.LINE_STRIP
marker.action = marker.ADD

# marker scale
marker.scale.x = 0.03
marker.scale.y = 0.03
marker.scale.z = 0.03

# marker color
marker.color.a = 1.0
marker.color.r = 1.0
marker.color.g = 0.0
marker.color.b = 0.0

# marker orientaiton
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
"""
# marker position
marker.pose.position.x = 0.0
marker.pose.position.y = 0.0
marker.pose.position.z = 0.0
"""
# marker line points
marker.points = []
rospy.loginfo('Marker created')

rospy.init_node('position_tracker')

pub_point = rospy.Publisher('orbslam_marker', Marker, queue_size=100)
print("Publisher created....")

rospy.Subscriber("/orbslam_pose", PoseStamped, callback)
print("Subcriber created....")
rospy.spin()