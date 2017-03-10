#!/usr/bin/env python

PKG = 'manhole_labeler'
import roslib; roslib.load_manifest(PKG)
import rospy
import fileinput
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
import numpy as np
import sys

class FakeDetector:    

  # Export images to files:
  def export_rgb_image(msg, file_):
    nparr = np.fromstring(msg.data,np.uint8)
    gray_image = cv2.imdecode(nparr, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    detect_manhole(gray_image, min_rad)
    downsampled = cv2.resize(gray_image, None, fx=0.25, fy=0.25)
    #cv2.imshow('img',downsampled)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    file_.write(np.array2string( downsampled).replace('[','').replace(']',''))
    
  def export_depth_image(msg, file_):
    #print len(msg.data)
    nparr = np.fromstring(msg.data, np.uint8)
    gray_image = cv2.imdecode(nparr[12:], cv2.CV_LOAD_IMAGE_GRAYSCALE) # CompressedDepth format adds 12 bits of garbage at the begining
    #print gray_image
    downsampled = cv2.resize(gray_image, None, fx=0.25, fy=0.25)
    file_.write(np.array2string( downsampled).replace('[','').replace(']',''))

  def rgb_callback(self, img):
    seq = img.header.seq
    print "RGB Callback. Seq: %d"%seq
    bool_msg = Bool(False)
    id_msg = Int32(-1)
    pose_msg = Pose2D(float('nan'), float('nan'), float('nan'))
    
    for i in range(len(self.detected_vector)):
      if self.detected_vector[i][0] <= seq and self.detected_vector[i][1] >= seq:
        #  Prepare all data for publishing
        if len(self.detected_vector[i]) > 2:
          id_msg.data = self.detected_vector[i][2]
        bool_msg.data = True
        
        if len(self.detected_vector[i]) > 4:
          print 'Hereeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee'
          pose_msg.x = self.detected_vector[i][3]
          pose_msg.y = self.detected_vector[i][4]
        break
    self.bool_pub.publish(bool_msg)
    self.id_pub.publish(id_msg)
    self.pose_pub.publish(pose_msg)
    
  def depth_callback(self, img):
    print "Depth Callback"
    #seq = img.header.seq
    #bool_msg = Bool(False)
    #for i in range(len(self.detected_vector)):
      #if self.detected_vector[i][0] < seq and self.detected_vector[i][1] > seq:
        ##bool_msg.data = True
        #break
    #self.bool_pub.publish(bool_msg)
        
  def __init__(self, camera, filename):
    self.load_vector(filename)
    np.set_printoptions(precision=3, threshold=10000, linewidth=10000)
    rgb_image = camera + "/rgb/image_raw/compressed"
    rgb_info = camera + "/rgb/camera_info"
    rospy.Subscriber(rgb_image, CompressedImage, self.rgb_callback)
    
    # Setup publisher
    self.bool_pub = rospy.Publisher('manhole',Bool, queue_size=2)
    self.id_pub = rospy.Publisher('manhole_id', Int32, queue_size=2)
    self.pose_pub = rospy.Publisher('position', Pose2D, queue_size=2)
    # Spin until ctrl + c
    rospy.spin()
    
  def load_vector(self, filename):
    self.detected_vector = np.loadtxt(filename)
    print "Loaded_vector"
    print self.detected_vector
      
if __name__ == '__main__':
  if len(sys.argv) > 2:
    rospy.init_node(sys.argv)
    detector = FakeDetector(sys.argv[1], sys.argv[2])
  else:
    print "usage: %s <camera> <vector_file>" % sys.argv[0]
    sys.exit()