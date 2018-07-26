#!/usr/bin/env python
# license removed for brevity
import rospy
from tf import TransformListener
import tf
from rssi_get.msg import Nvip_status
from libelium_waspmote_gas_node.msg import GasMeasure
#from tf import TransformError
#from tf import TfError

def callback(data):
  #if tf_listener.frameExists("/base_link") and tf_listener.frameExists("/odom"):
  try:
    t = tf_listener.getLatestCommonTime("/odom", "/base_link")
    pos, ori = tf_listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
    #if  tf_listener.frameExists("/map"):
      #t = tf_listener.getLatestCommonTime("/map", "/base_link")
      #pos_2, ori_2 = tf_listener.lookupTransform("/map", "/base_link",  t)
      #text_file.write("%f %f %f %f %d\n"%(pos[0], pos[1], pos_2[0], pos_2[1], data.rssi))
    #else:
      #text_file.write("%f %f 0.0 0.0 %d\n"%(pos[0], pos[1], data.rssi))
    #print pos
    
    text_file.write("%f %f %f %d\n"%(t.secs + t.nsecs * 1e-9, pos[0], pos[1], data.rssi))

    print("%f %f %f %d\n"%( t.secs + t.nsecs * 1e-9, pos[0], pos[1], data.rssi))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print("rssi callback: tf trouble")
    
def gas_callback(data):
  try:
    t = tf_listener.getLatestCommonTime("/odom", "/base_link")
    pos, ori = tf_listener.lookupTransform("/odom", "/base_link", rospy.Time(0))
    gas_text_file.write("%f %f %f %f %f %f %f %f %f %f\n"%(t.secs + t.nsecs * 1e-9, pos[0], pos[1], data.O2_conc, data.H2S_conc, data.CO_conc, data.CH4_conc, data.temperature, data.pressure, data.RH))
    print("Gas measure: %f %f %f %f %f %f %f %f %f %f\n"%(t.secs + t.nsecs * 1e-9, pos[0], pos[1], data.O2_conc, data.H2S_conc, data.CO_conc, data.CH4_conc, data.temperature, data.pressure, data.RH))
  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print("gas_callback: tf trouble")
    
if __name__ == '__main__':
  rospy.init_node('gather_data')
  tf_listener = TransformListener()
  rospy.Subscriber("/rssi_nvip_2400", Nvip_status, callback)
  rospy.Subscriber("/gas_info", GasMeasure, gas_callback)
  text_file = open('rssi.txt', 'w')
  gas_text_file = open('gas.txt', 'w')
  gas_text_file.write("% t x y o2 h2s co ch4 temp pressure RH\n");
  rospy.spin()
  text_file.close()
  