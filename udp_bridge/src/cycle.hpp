#ifndef __CYCLE_H__
#define __CYCLE_H__

#include <ros/ros.h>
  
class Cycle
{
protected:
  int m_count;
  ros::Time m_start;
  ros::Duration m_expected_cycle_time;
  bool is_init;
  
public:

  Cycle():is_init(false)
  {
    m_count = 0;
    m_start = ros::Time::now();
  }
  
  Cycle(double frequency):is_init(false)
  {
    init(frequency);
  }
  
  void init(double frequency)
  {
    m_count = 0;
    m_start = ros::Time::now();
    m_expected_cycle_time = ros::Duration(1.0 / frequency);
    is_init = true;
  }
  
  bool newCycle(void)
  {
    if (!is_init) {
      return false;
    }
    ros::Time expected_end = m_start + m_expected_cycle_time;
    ros::Time actual = ros::Time::now();
    
    if(actual > expected_end)
    {
      m_count++;
      m_start = expected_end;
      if(actual > m_start + m_expected_cycle_time)
        m_start = actual;
      return true;
    }
    else
      return false;
  }
   
  int getCycleCount(void)
  {
    return m_count;
  }
};


#endif








