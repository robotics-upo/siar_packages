#ifndef __COMMAND_EVALUATOR_CPP__
#define __COMMAND_EVALUATOR_CPP__

#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/math/special_functions/sign.hpp>
#include <math.h>

#include "siar_footprint.hpp"

namespace siar_controller {
  
  struct RobotCharacteristics {
    double a_max;
    double v_max;
    double theta_dot_max;
    double a_max_theta;
  };
  
  class CommandEvaluator {
  public:
    CommandEvaluator(double w_dist, double w_safe, double T, const RobotCharacteristics &model, double delta_T = 0.1);
    
    //! @brief Simulates the trajectory during T and generates a cost according to the cost map
    //! @retval -1.0 --> Collision
    double evualateTrajectory(const geometry_msgs::Twist &v_ini, const geometry_msgs::Twist &v_command, const geometry_msgs::Twist &operator_command, const nav_msgs::OccupancyGrid &alt_map);
    
    //! @brief Destructor
    ~CommandEvaluator();
    
    inline RobotCharacteristics getCharacteristics() const {return m_model;}
    
  protected:
    double m_T; // Lookahead time
    double m_delta_T; // Timestep
    double m_divRes, origin_x, origin_y;
    int width;
    
    double m_w_dist, m_w_safe; // Different weights. Respectively: Distance to commanded velocity, safety, collision penalty
    
    RobotCharacteristics m_model;
    
    SiarFootprint *footprint;
    
    inline double computeNewVelocity(double &v, double &w, double dt, const geometry_msgs::Twist &com){
        v = std::min(com.linear.x, v + m_model.a_max * dt * boost::math::sign(com.linear.x - v));
        w = std::min(com.angular.z, w + m_model.a_max_theta * dt * boost::math::sign(com.angular.z - w));
      }
    
    //! brief Applies the footprint in the altitude map
    //! @param x X coord
    //! @param y Y coord
    //! @param th Theta angle
    //! @param alt_map The altitude map
    //! @param collision (Out parameter) Will become true if a collision is detected
    int applyFootprint(double x, double y, double th, nav_msgs::OccupancyGrid alt_map, bool &collision);
    
    inline int point2index(double x, double y)
        {
                return ((int)((x - origin_x)*m_divRes))*width + (int)((y - origin_y)*m_divRes);
        }
  };

CommandEvaluator::CommandEvaluator(double w_dist, double w_safe, double T, const RobotCharacteristics &model, double delta_T):footprint(NULL)
{
  m_w_dist = w_dist;
  m_w_safe = w_safe;
  m_T = T;
  m_delta_T = delta_T;
  m_model = model;
}

CommandEvaluator::~CommandEvaluator()
{
  delete footprint;
}

double CommandEvaluator::evualateTrajectory(const geometry_msgs::Twist& v_ini, const geometry_msgs::Twist& v_command, 
                                            const geometry_msgs::Twist& operator_command, const nav_msgs::OccupancyGrid& alt_map)
{
  double dt = m_delta_T;
  int steps = m_T / m_delta_T;
  double x=0.0, y=0.0, th=0.0;


  double lv = v_ini.linear.x;
  double av = v_ini.angular.z;
  
  int cont_footprint = 0;
  
  // Initialize the footprint if needed:
  if (footprint == NULL) {
    footprint = new SiarFootprint(alt_map.info.resolution); // TODO: CONFIGURABLE FOR VARIABLE WIDTH PROTOTYPE
    m_divRes = 1.0 / alt_map.info.resolution;
    origin_x = -alt_map.info.height * alt_map.info.resolution /2.0;
    origin_y = -alt_map.info.width * alt_map.info.resolution /2.0;
  }

  //int ini = floor(steps/2.0 + 0.5);
  bool collision = false;
  for(unsigned int i=0; i <= steps && !collision; i++)
  {
    lv = computeNewVelocity(lv, av, dt, v_command);

    // Integrate the model
    double lin_dist = lv * dt;
    th = th + (av * dt);
    //normalization just in case
//     th = normalizeAngle(th, -M_PI, M_PI);
    x = x + lin_dist*cos(th); // Euler 1
    y = y + lin_dist*sin(th); 
    
    cont_footprint += applyFootprint(x, y, th, alt_map, collision);

  }
  
  double ret = cont_footprint * m_w_safe + m_w_dist * sqrt(pow(x - operator_command.linear.x * m_T, 2.0) + y*y);
  if (collision)
    return -1.0;
    
  return ret;
}

// TODO: test the conversion between footprint coords and map coords
int CommandEvaluator::applyFootprint(double x, double y, double th, nav_msgs::OccupancyGrid alt_map, bool &collision)
{
  int ret_val = 0;
  
  FootprintType fp = footprint->getFootprint(x, y, th);
  
  int i_ini = x/alt_map.info.resolution + alt_map.info.origin.position.x;
  int j_ini = y/alt_map.info.resolution + alt_map.info.origin.position.y;
  
  collision = false;
  
  int index;
  for (unsigned int i = 0; i < fp.size() && !collision; i++) {
    index = point2index(fp.at(i).x, fp.at(i).y);
    if (alt_map.data[index] == 127) 
      collision = true; // Collision detected!
    ret_val += abs(alt_map.data[index]); // TODO: check index and coordinate transform
    
  }
  
  return ret_val;
}


  
}


#endif // __COMMAND_EVALUATOR_CPP__