#ifndef __ARM_FIREWALL_HPP__
#define __ARM_FIREWALL_HPP__

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "math.h"
#include "siar_driver/SiarArmCommand.h"
#include "siar_driver/SiarStatus.h"
#include "siar_arm.hpp"
#include <queue>


using namespace std;

static class ArmFirewall {

  bool checkJointLimits(const std::array<uint16_t, 5> joint_values)
  {
    bool ret_val = (joint_values[0]<805 && joint_values[0]>208);
    ret_val &= (joint_values[1]<1568 && joint_values[1]>220);
    ret_val &= (joint_values[2]<2020 && joint_values[2]>191); // Limits from the Arm Reference Value spreadsheet of Carlos Marques
    ret_val &= (joint_values[3]<958 && joint_values[3]>30);
    ret_val &= (joint_values[4]<1000 && joint_values[4]>30);
    
    return ret_val;
  }

  bool checkTemperatureAndStatus(const std::array<uint16_t,5> &herculex_temperature, const std::array<uint16_t,5> &herculex_status) {
    bool ret_val = true;
    for(int i = 0; i<5; i++)
    {
      if (herculex_temperature[i]<0 && herculex_temperature[i]>50)
      {
        ROS_ERROR("TEMPERATURE OF THE %d LINK IS OUT OF RANGE: %d", i, herculex_temperature[i]);
        ret_val = false;
      }
      if (herculex_status[i]!=1)
      {
        ROS_ERROR("%d LINK STATUS: %d", i, herculex_status[i]);
        ret_val = false;
      }
    }
    return ret_val;  
  }
  
};



#endif  







