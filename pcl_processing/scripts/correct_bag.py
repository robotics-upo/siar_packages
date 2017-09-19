#!/usr/bin/env python

#PKG = 'sensor_msgs'
import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import fileinput
import os

def fix(inbag, outbag, camera):
  print "Trying to migrating file: %s"%inbag
  inbag_ = rosbag.Bag(inbag, 'r')                                           
  outbag_ = rosbag.Bag(outbag, 'w')
  
  topic_image = camera + "/depth_registered/image_raw/compressedDepth"
  topic_info = camera + "/depth_registered/camera_info"
  
  for (topic, msg, t) in inbag_.read_messages():
    if msg._type == 'sensor_msgs/CameraInfo':
      msg.header.frame_id = 'front_depth_optical_frame'
      msg_info = msg
      break
  for (topic, msg, t) in inbag_.read_messages():
    if topic == topic_image:
      msg_info.header = msg.header
      
      outbag_.write(topic_info, msg_info, t, raw=False)
      outbag_.write(topic, msg, t, raw=False)
    else:
      outbag_.write(topic, msg, t, raw=False)
  outbag_.close()

if __name__ == '__main__':
  import sys
  if len(sys.argv) >= 3:
    fix(sys.argv[1], sys.argv[2], sys.argv[3])
  else:
    print "usage: %s <in_bag> <out_bag> <camera>" % sys.argv[0]