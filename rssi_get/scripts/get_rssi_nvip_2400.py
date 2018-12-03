#!/usr/bin/env python
# license removed for brevity
import os
os.environ['http_proxy']='' # nVIP2400 has no proxy
import urllib2
import rospy
import requests
import re
from rssi_get.msg import Nvip_status


def talker():
    pub = rospy.Publisher('rssi_nvip_2400', Nvip_status, queue_size=10)
    rospy.init_node('rssi_nvip_getter', anonymous = False)
    ip = "192.168.168.47"
    if rospy.has_param('~ip'):
      ip = rospy.get_param('~ip')
      
    password = "admin"
    if rospy.has_param('~password'):
		password = rospy.get_param('~password')
	      

    device = 'nVIP2400'
    if rospy.has_param('~device'):
      device = rospy.get_param('~device') # Can be: nVIP2400 or pX2
      
    webname = {'nVIP2400': '/cgi-bin/webif/status-wlan.sh?interval=-1', 'pX2': '/cgi-bin/webif/status-wlan.sh?interval=-1'}
    url = "http://admin:" + password + "@" + ip + webname[device]
  
    print "Connecting to: ", url
    rate_d = 0.33
    if rospy.has_param('~rate'):
      rate_d = rospy.get_param('~rate')
    rate = rospy.Rate(rate_d) 
    while not rospy.is_shutdown():
      info = ["", "" , "", "", "", ""]
      #s_ = requests.session() Not necessary. For closing connections
      #s_.keep_alive = False
      #r = requests.post(url, "", headers={'Connection':'close'})
      try:
        req = requests.get(url, timeout=2)
      except Exception:
        print "Could not make the request"
      else:
        if req.status_code == 200:
          text = req.text
          #print text         # DEBBBUGGGGG
          text_elem = 0
          cont = -1
          cont_elem = 0
          have_data = -1
          in_table = 0
          cont_row = -1
          for item in text.split("\n"):
            if "</table>" in item:
              in_table = 0
            if in_table == 1:
              if "<tr" in item:
                cont_row = cont_row + 1
              else:
                info[cont_row] = info[cont_row] + item + os.linesep 
            if "Connection Status" in item or "Connection Info" in item:
              in_table = 1
          i = 1
          # get the data from the interesting areas
          for s in info:
            print s
            if len(s) > 20:
              result_list = re.findall(r"(([0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2})+", s)
              #print "Result list: ", result_list              # DEBBBUGGGGG
              if (len(result_list) > 0):
                st = Nvip_status(result_list[0][0], 0 ,0 ,0 ,0)
                result_list = re.findall(">[0-9\-]+<", s)
                #print result_list                                  # DEBBBUGGGGG
                st.rssi = int(result_list[2][1:-1]) 
                result_list = re.findall("[0-9\.\-]+\sM", s)
                st.tx_rate = float(result_list[0][0:-1]) 
                st.rx_rate = float(result_list[1][0:-1])
                result_list = re.findall("[0-9]+%<", s)
                st.rssi_perc = int(result_list[0][0:-2])
                pub.publish(st)
      #del req
      rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass


      
    
    
  
