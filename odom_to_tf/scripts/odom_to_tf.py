#!/usr/bin/env python  
import roslib
roslib.load_manifest('odom_to_tf')
import rospy

import tf
import tf2_ros
from nav_msgs.msg import Odometry
import geometry_msgs.msg
from sensor_msgs.msg import Imu

class OdomToTF:
  def __init__(self):
    self.last_imu_orientation = None
    self.base_link = "/base_link"
    self.odometry_frame = rospy.get_param('~odom_frame_id', 'odom')
    self.base_link = rospy.get_param('~base_frame_id', 'base_link')
    self.override_stamp = rospy.get_param('~overridestamp', default=False)
    rospy.Subscriber("odom", Odometry, self.odometryCb)  
    rospy.Subscriber("imu", Imu, self.imuCb)

  def odometryCb(self, msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    if override_stamp:
      t.header.stamp = rospy.Time.now()
    else:
      t.header.stamp = msg.header.stamp
    t.header.frame_id = odometry_frame
    t.child_frame_id = base_link
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z
    if self.last_imu_orientation == None:
      t.transform.rotation.x = msg.pose.pose.orientation.x
      t.transform.rotation.y = msg.pose.pose.orientation.y
      t.transform.rotation.z = msg.pose.pose.orientation.z
      t.transform.rotation.w = msg.pose.pose.orientation.w
    else:
      t.transform.rotation.x = self.last_imu_orientation.x
      t.transform.rotation.x = self.last_imu_orientation.x
      t.transform.rotation.x = self.last_imu_orientation.x
      t.transform.rotation.x = self.last_imu_orientation.x
      
      
    br.sendTransform(t)
    
  def imuCb(self, msg):
    self.last_imu_orientation = msg.orientation

if __name__ == '__main__':
    last_quaternion = None
    rospy.init_node('odom_to_tf_broadcaster')
    rospy.spin()