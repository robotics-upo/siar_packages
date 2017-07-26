#ifndef _ASTAR_HPP_
#define _ASTAR_HPP_

#include <unordered_map>
#include <unordered_set>

#include "siar_planner/AStarState.hpp"
#include "siar_planner/AStarNode.h"
#include "siar_planner/AStarModel.hpp"

#include "visualization_msgs/Marker.h"

#include <functions/functions.h>

#include <ros/ros.h>

class AStar
{
public:
  AStar(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  double getPath(AStarState start, AStarState goal, std::list<AStarNode>& path);      
  
  AStarModel &getModel() {return m;}
  
  visualization_msgs::Marker getPathMarker(const std::list< AStarNode >& path);
  
  visualization_msgs::Marker getGraphMarker();
  
protected:
  AStar();
  
  bool allow_relaxation;

  void addEdge(int id0, int id1, double cost);
  
  int addNode(AStarState st, double comm_x = 0.0, double comm_ang = 0.0);
  void addEdge(int id0, int id1);
  void clear();
  
  void expandNode(int base_id, int relaxation_mode = 0);
  
  double heuristic_cost(const AStarNode &a, const AStarNode &b) const;
  
  int getBestNode(int mode = 0);
  
  //! @brief Searchs for a state in the graph and returns its ID if found
  int getCellID(const AStarState &st) const;

  std::unordered_map<int, AStarNode> nodes;
  
  bool isGoal(int goal_id, const AStarState &st);
  
  bool isSameCell(const AStarState& s1, const AStarState& s2) const;
  
  int K, n_iter, n_rounds;
  double delta_t, cellsize_m, cellsize_rad, cost_weight;
  
  std::unordered_set<int> closedSet;
  std::unordered_set<int> openSet;
  
  AStarModel m;
  double goal_gap_m, goal_gap_rad;
  
  // Random numbers
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
  
  // Sets to test
  bool file_test_set_init;
  std::vector <geometry_msgs::Twist> test_set_forward, test_set_backward;
  std::vector <geometry_msgs::Twist> file_test_set_forward, file_test_set_backward;
  double wheel_decrease;
  bool optimize;
  double limit_exploration;
  double last_wheel;
  
  
  //! @brief Retrieves the file_test_set_forward and backward from file (the filename is specified in the configuration)
  bool initializeTestSetFromFile(const std::string& velocityset_filename, int n_interpol = 1, double mult = 1.0);
  
  //! @brief Adds a velocity to the forward test, the same velocity with opposite angular velocity. Then does the same to backward test but changing the sign of v.x
  void addVelocityToTestSets(const geometry_msgs::Twist &v, std::vector<geometry_msgs::Twist> &set_forward, std::vector<geometry_msgs::Twist> &set_backward);
  bool checkExplorationBounds(const AStarState& st);
};

AStar::AStar(ros::NodeHandle &nh, ros::NodeHandle &pnh):m(nh, pnh), gen(rd()), dis(0,1)
{
  if (!pnh.getParam("K", K)) { // Number of random commands
    K = 4;
  }
  if (!pnh.getParam("n_iter", n_iter)) {
    n_iter = 1000;
  }
  if (!pnh.getParam("n_rounds", n_rounds)) {
    n_rounds = 6;
  }
  if (!pnh.getParam("delta_t", delta_t)) {
    delta_t = 0.5;
  }
  if (!pnh.getParam("wheel_decrease", wheel_decrease)) {
    wheel_decrease = 0.05;
  }
  if (!pnh.getParam("cellsize_m", cellsize_m)) {
    cellsize_m = 0.2;
  }
  if (!pnh.getParam("cellsize_rad", cellsize_rad)) {
    cellsize_rad = 0.2;
  }
  
  if (!pnh.getParam("allow_relaxation", allow_relaxation)) {
    allow_relaxation = true;
  }
  
  if (!pnh.getParam("last_wheel", last_wheel)) {
    last_wheel = 0.1;
  }
  
  if (!pnh.getParam("goal_gap_m", goal_gap_m)) {
    goal_gap_m = cellsize_m;
  }
  
  if (!pnh.getParam("goal_gap_rad", goal_gap_rad)) {
    goal_gap_rad = cellsize_rad;
  }
  if (!pnh.getParam("cost_weight", cost_weight)) {
    cost_weight = 1e-2;
  }
  if (!pnh.getParam("optimize", optimize)) {
    optimize = true;
    
  }
  if (!pnh.getParam("limit_exploration", limit_exploration)) {
    
    limit_exploration = 1.3;
  }
  
  
  ROS_INFO("n_iter = %d \t K: %d \t Cost weight: %f", n_iter, K, cost_weight);
  
  file_test_set_init = false;
  std::string velocity_file;
  if (pnh.getParam("velocity_file", velocity_file)) {
    
    file_test_set_init = initializeTestSetFromFile(velocity_file);
   ROS_INFO("Velocity file: %s", velocity_file.c_str());
    if (file_test_set_init)
      ROS_INFO("Initialized the file test set");
  }
}


inline
void AStar::clear()
{
  nodes.clear();
}

inline
int AStar::addNode(AStarState st, double comm_x, double comm_ang)
{
  int id = getCellID(st);
  if (id < 0) {
    AStarNode &n = nodes[nodes.size()];
    n.id = nodes.size() - 1;
    n.st = st; 
    id = n.id;
    n.command_ang = comm_ang;
    n.command_lin = comm_x;
  }
  
  return id;
}


inline
void AStar::addEdge(int id0, int id1, double cost)
{
  nodes[id0].neighbors[id1] = cost;  
}


double AStar::getPath(AStarState start, AStarState goal, std::list<AStarNode>& path)
{
  nodes.clear(); // Incremental algorithm --> the graph is generated in each calculation
  path.clear();
  openSet.clear();
  closedSet.clear();
  
  if (!m.isInit()) {
    ROS_ERROR("AStar::getPath --> The model has not been initialized --> could not calculate a path");
    return -1.0;
  }
  
  int start_id = addNode(start);
  int goal_id = addNode(goal);
  double ret_val = -1.0;
  
  AStarNode& start_node = nodes.at(start_id);
  AStarNode& goal_node = nodes.at(goal_id);
  
  
  for (auto it = nodes.begin(); it != nodes.end(); ++it) {
    it->second.gScore = std::numeric_limits<double>::infinity();
    it->second.fScore = std::numeric_limits<double>::infinity();
    it->second.parent = NULL;
  }
  
  start_node.gScore = 0;
  start_node.fScore = heuristic_cost(start_node,goal_node);  
  
  double original_m_w_ = m.getMinWheel();
  
  
  openSet.insert(start_id);
  int relax = 0;
  while (allow_relaxation && relax < n_rounds) {
    int cont = 0;
    while (cont < n_iter && !openSet.empty()) {
      
      // Get node ID with minimum cost
      int current_id = getBestNode(relax);
      
      if (current_id < 0) {
        std::cerr << "ID < 0 --> Openset empty" << std::endl;
        break;
      }
      
      AStarNode* current_node = &nodes[current_id];    
      
      if (isGoal(goal_id, current_node->st) && (current_node->fScore < ret_val || ret_val < 0.0)) {
        // Retrieve path
        path.clear();
        ret_val = current_node->fScore;
        path.push_front(*current_node);
        while (current_node->parent != NULL) {
          current_node = current_node->parent;
          path.push_front(*current_node);
        }
      } else if (!isGoal(goal_id, current_node->st)) {
        expandNode(current_id, relax);
      }
      // Expands the graph from the current node --> Draws edges according to the inputs  
      
      openSet.erase(current_id);          
      closedSet.insert(current_id);
      for (auto it = current_node->neighbors.begin(); it != current_node->neighbors.end(); ++it) {
        int neighbor_id = it->first;
        double neighbor_cost = it->second;
        
        if (closedSet.count(neighbor_id)>0) {
          continue;
        }

        double tentative_gScore = current_node->gScore + neighbor_cost;
        AStarNode &neighbor_node = nodes.at(neighbor_id);
        if (openSet.count(neighbor_id)==0) {
          openSet.insert(neighbor_id);
        } else if (tentative_gScore >= neighbor_node.gScore) {
          continue;
        }
        neighbor_node.parent = current_node;
        neighbor_node.gScore = tentative_gScore;
        neighbor_node.fScore = cost_weight * tentative_gScore + heuristic_cost(neighbor_node, goal_node);
        
      }
      cont++;
    }
    relax++; // If no solution is found --> try with relaxation (if configured so)
    if (allow_relaxation)
    {
      if (ret_val > 0.0) {
        cost_weight *= 5; 
        ROS_INFO("Iteration %d. Solution found --> new cost weight: %f. Min cost: %f", relax, cost_weight, ret_val);
        if (!optimize) {
          return ret_val;
        }
      } else {
        double m_w = m.getMinWheel() - ((relax > 1)?wheel_decrease:0);
        n_iter += 100;
        m_w = (m_w < last_wheel)?last_wheel:m_w;
        m.setMinWheel(m_w);
        ROS_INFO("Iteration %d. Cont = %d. No solution --> decreasing min wheel to %f", relax, cont, m_w);
      }
      // Open the nodes for relaxation!
      for (auto it = closedSet.begin(); it != closedSet.end(); ) {
        int id = *it;
        it++;
        closedSet.erase(id);
        openSet.insert(id);
      }
      ROS_INFO("Openset size = %d",(int) openSet.size());
    }
    
  } 
  m.setMinWheel(original_m_w_); // Restore the original min wheel
  return ret_val;
}


double AStar::heuristic_cost(const AStarNode& a, const AStarNode& b) const
{
  return a.st.state.distance(b.st.state);
}

void AStar::expandNode(int base_id, int relaxation_mode)
{
  AStarNode &n = nodes[base_id];
  
  test_set_backward = file_test_set_backward;
  test_set_forward = file_test_set_forward;
  
  for (int i = 0; i < K; i++) {
    geometry_msgs::Twist command = m.generateRandomCommand();
    addVelocityToTestSets(command, test_set_forward, test_set_backward);
  }
  
//   std::copy(test_set_backward.begin(), test_set_backward.end(),
//           std::back_insert_iterator<std::vector<geometry_msgs::Twist> >(test_set_forward));
  
  for (auto it = test_set_forward.begin(); it != test_set_forward.end(); it++) {
    AStarState st = n.st;
    geometry_msgs::Twist &command = *it;
    
    double cost = m.integrate(st, command, delta_t, relaxation_mode >= 1, false); // If relaxation_mode >= 1 --> allow two wheels
    
    if (cost < 0.0) {
      // Collision
      // Update??
//       ROS_INFO("Detected collision. State: %s.\t Command: %f, %f", st.state.toString().c_str(), command.linear.x, command.angular.z);
    } else {
      // Add the vertex to the graph
      int neighID = addNode(st, command.linear.x, command.angular.z);
      addEdge(base_id, neighID, cost);
    }
  }
}

int AStar::getBestNode(int mode) 
{
  double min = std::numeric_limits<double>::infinity();
  int ret_val = -1;
  double candidate;
  for (auto it = openSet.begin(); it != openSet.end(); ++it) {
    if (mode == 0) {
      candidate = nodes.at(*it).fScore;
      if (m.isCollision(nodes[*it].st))
        continue;
    }
    else {
      candidate = nodes.at(*it).fScore;
    }
    if (candidate < min) {
      // Check if it exceeds the exploration bounds:
      if (checkExplorationBounds(nodes[*it].st)) {
	min = candidate;
	ret_val = *it;
	
      }
    }
  }
  
  return ret_val;
}


int AStar::getCellID(const AStarState& st) const
{
  int ret_val = -1;
  
  for (int i = 0; i < nodes.size() && ret_val < 0; i++) {
    const AStarState &curr_state = nodes.at(i).st;
    
    if ( isSameCell(curr_state, st) ) {
      ret_val = i;
    }
  }
  
  return ret_val;
}

bool AStar::isSameCell(const AStarState& s1, const AStarState& s2) const
{
  bool ret_val = false;
  
  if (s1.state.size() == s2.state.size()) {
    double cellsize;
    ret_val = true;
    for (unsigned int i = 0; i < s1.state.size() && ret_val; i++) {
      if (i < 2) {
        cellsize = cellsize_m;
      } else
        cellsize = cellsize_rad;
      
      ret_val = fabs(round(s1.state[i] / cellsize) - round(s2.state[i]/cellsize)) < cellsize;
    }
  }
  
  return ret_val;
}

visualization_msgs::Marker AStar::getGraphMarker()
{
  visualization_msgs::Marker m;
  m.header.frame_id = this->m.getFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "astar";
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.id = 0;
  m.points.clear();
  m.type = visualization_msgs::Marker::LINE_LIST;
  // LINE_LIST markers use x scale only (for line width)
  m.scale.x = cellsize_m / 3.0;
  // Points are green
  visualization_msgs::Marker::_color_type color;
  color.r = 1.0;
  color.b = 0;
  color.g = 0;
  color.a = 1.0;
  double color_step = 1.0/(double)nodes.size();
  geometry_msgs::Point p1;
  geometry_msgs::Point p2;
  for (unsigned int i = 0; i < nodes.size();i++) {
    
    auto new_color = color;
    new_color.r -= color_step;
    new_color.b += color_step;
    
    p1.x = nodes[i].st.state[0];
    p1.y = nodes[i].st.state[1];
    
    for (auto it = nodes[i].neighbors.begin(); it != nodes[i].neighbors.end(); ++it) {
      m.points.push_back(p1);
      m.colors.push_back(color);
      
      p2.x = nodes[it->first].st.state[0];
      p2.y = nodes[it->first].st.state[1];
      
      m.points.push_back(p2);
      m.colors.push_back(new_color);
    }
   
    color = new_color;
    
  }
  
  return m;
}

bool AStar::isGoal(int goal_id, const AStarState &st) {
  bool ret_val = true;
  
  functions::RealVector test = st.state;
  functions::RealVector goal = nodes[goal_id].st.state;
  
  if (test.size() < 3 || goal.size() < 3) {
    ret_val = false;
  } else
    ret_val = (fabs(test[0] - goal[0]) < goal_gap_m) && (fabs(test[1]-goal[1]) < goal_gap_m) &&
            (fabs(test[2] - goal[2]) < goal_gap_rad);
    
  
  return ret_val;
}

visualization_msgs::Marker AStar::getPathMarker(const std::list< AStarNode >& path) 
{
  visualization_msgs::Marker ret;
  
  int cont = 0;
  AStarState st;
  for (auto it = path.begin(); it != path.end(); it++, cont++) {
    if (cont > 0) {
      st = (--it)->st;
      it++;
    
      geometry_msgs::Twist command;
      command.linear.x = it->command_lin;
      command.angular.z = it->command_ang;
    
      m.integrate(ret, st, command, 1.0, true);
    }
  }
  
  return ret;
}


// Test sets:

bool AStar::initializeTestSetFromFile(const std::string& velocityset_filename, int n_interpol, double mult)
{
  file_test_set_init = true;
  file_test_set_forward.clear();
  file_test_set_backward.clear();
  
  std::vector<std::vector <double> > M;
  
  geometry_msgs::Twist v, v_ant, v_new;
  v.linear.x = v.linear.y = v.linear.z = 0.0;
  v.angular.x = v.angular.y = v.angular.z = 0.0;
  
  bool loaded = functions::getMatrixFromFile(velocityset_filename, M);
  if (loaded) 
  {
    for ( auto vec : M )
    {
      if (vec.size() < 2)
        continue;
      
      v.linear.x = vec.at(0) * mult;
      v.angular.z = vec.at(1) * mult;
      
      ROS_INFO("File test set. Forward Command %d. vx = %f. v_theta = %f", (int)file_test_set_forward.size(), v.linear.x, v.angular.z);
      addVelocityToTestSets(v, file_test_set_forward, file_test_set_backward);
      
      if (file_test_set_forward.size() > 1) {
        // Perform n_interpols
        v_new = v_ant;
        double inc_x = (v.linear.x - v_ant.linear.x) / (double)(n_interpol + 1);
        double inc_z = (v.angular.z - v_ant.angular.z) / (double)(n_interpol + 1);
        for (int i = 1; i <= n_interpol; i++) {
          v_new.linear.x += inc_x;
          v_new.angular.z += inc_z;
          addVelocityToTestSets(v_new, file_test_set_forward, file_test_set_backward);
        }
      }
      
      v_ant = v;
      
    }
  }
  return loaded;
}

void AStar::addVelocityToTestSets(const geometry_msgs::Twist& v1, std::vector< geometry_msgs::Twist >& set_forward, std::vector< geometry_msgs::Twist >& set_backward)
{
  geometry_msgs::Twist v = v1;
  // First forward velocities
  set_forward.push_back(v);
  if (fabs(v.angular.z) > 1e-10) {
    v.angular.z *= -1.0;
    set_forward.push_back(v);
  }
  // Then backward ones
  v.linear.x *= -1.0;
  set_backward.push_back(v);
  if (fabs(v.angular.z) > 1e-10) {
    v.angular.z *= -1.0;
    set_backward.push_back(v);
  }
}

bool AStar::checkExplorationBounds(const AStarState &st)
{
  bool ret_val = true;
  
  if (limit_exploration > 0.0) {
//     ROS_INFO("Check exploration bounds: m.getWorldMaxX = %f\t st.state[0] = %f", m.getWorldMaxY(), st.state[1]);
//     ROS_INFO("Check exploration bounds: m.getWorldMaxY = %f\t st.state[1] = %f\t limit_exploration = %f", m.getWorldMaxY(), st.state[1], limit_exploration);
    ret_val = fabs(st.state[0]) < m.getWorldMaxX() * limit_exploration;
    ret_val &= fabs(st.state[1]) < m.getWorldMaxY() * limit_exploration;
  }
    
  return ret_val;
}


#endif
