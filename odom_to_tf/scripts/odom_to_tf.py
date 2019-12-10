#!/usr/bin/env python  
import roslib
roslib.load_manifest('odom_to_tf')
import rospy

import tf
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg

def odometryCb(msg):
    br = tf2_ros.TransformBroadcaster()
    print base_link
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
    t.header.frame_id = odometry_frame
    t.child_frame_id = base_link
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w
    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('odom_to_tf_broadcaster')
    base_link = "/base_link"
    odometry_topic = "/odom"
    odometry_frame = "/odom"
    if rospy.has_param('~base_link'):
      base_link = rospy.get_param('~base_link')
    if rospy.has_param('~odometry_frame'):
      odometry_frame = rospy.get_param('~odometry_frame')
    rospy.Subscriber(odometry_topic,
                     Odometry,
                     odometryCb
                     )
    rospy.spin()