#ifndef _ASTAR_HPP_
#define _ASTAR_HPP_

#include <unordered_map>
#include <unordered_set>

#include "siar_planner/AStarState.hpp"
#include "siar_planner/AStarNode.h"
#include "siar_planner/AStarModel.hpp"

#include "visualization_msgs/Marker.h"

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
  
  int K, n_iter;
  double delta_t, cellsize_m, cellsize_rad;
  
  std::unordered_set<int> closedSet;
  std::unordered_set<int> openSet;
  
  AStarModel m;
  double goal_gap_m, goal_gap_rad;
  
  // Random numbers
  std::random_device rd;
  std::mt19937 gen;
  std::uniform_real_distribution<> dis;
};

AStar::AStar(ros::NodeHandle &nh, ros::NodeHandle &pnh):m(nh, pnh), gen(rd()), dis(0,1)
{
  if (!pnh.getParam("K", K)) {
    K = 4;
  }
  if (!pnh.getParam("n_iter", n_iter)) {
    n_iter = 1000;
  }
  if (!pnh.getParam("delta_t", delta_t)) {
    delta_t = 0.5;
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
  
  if (!pnh.getParam("goal_gap_m", goal_gap_m)) {
    goal_gap_m = cellsize_m;
  }
  
  if (!pnh.getParam("goal_gap_rad", goal_gap_rad)) {
    goal_gap_rad = cellsize_rad;
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
  
  int cont = 0;
  
  openSet.insert(start_id);
  int relax = 0;
  while (ret_val < 0.0 && allow_relaxation && relax < 3) {
    while (cont < n_iter && !openSet.empty()) {
      
      // Get node ID with minimum cost
      int current_id = getBestNode(relax);
      
      if (current_id < 0) {
        std::cerr << "ID < 0 --> Openset empty" << std::endl;
        break;
      }
      
      AStarNode* current_node = &nodes[current_id];    
      
      if (isGoal(goal_id, current_node->st)) {
        // Retrieve path
        path.clear();
        ret_val = current_node->fScore;
        path.push_front(*current_node);
        while (current_node->parent != NULL) {
          current_node = current_node->parent;
          path.push_front(*current_node);
        }
        return current_node->fScore;
      }
      // Expands the graph from the current node --> Draws edges according to the inputs  
      expandNode(current_id, relax);
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
        neighbor_node.fScore = tentative_gScore + heuristic_cost(neighbor_node, goal_node);
        
      }
      cont++;
    }
    relax++; // If no solution is found --> try with relaxation (if configured so)
    if (allow_relaxation && ret_val < 0.0)
    {
      // Open the nodes for relaxation!
      for (auto it = closedSet.begin(); it != closedSet.end(); it++) {
        int id = *it;
        closedSet.erase(id);
        openSet.insert(id);
      }
    }
  }  
  return ret_val;
}


double AStar::heuristic_cost(const AStarNode& a, const AStarNode& b) const
{
  return a.st.state.distance(b.st.state);
}

void AStar::expandNode(int base_id, int relaxation_mode)
{
  AStarNode &n = nodes[base_id];
  
  for (int i = 0; i < K; i++) {
    AStarState st = n.st;
    geometry_msgs::Twist command = m.generateRandomCommand();
    double cost = m.integrate(st, command, delta_t, relaxation_mode == 1, false); // If relaxation_mode = 0 --> no wheels. =1 --> one wheel, = 2 two wheels
    
    if (cost < 0.0) {
      // Collision
      // Update??
      ROS_INFO("Detected collision. State: %s.\t Command: %f, %f", st.state.toString().c_str(), command.linear.x, command.angular.z);
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
    if (mode == 2) {
      candidate = nodes.at(*it).fScore - (dis(gen)*0.0005 + 0.9995)*nodes.at(*it).gScore;
      if (m.isCollision(nodes[*it].st))
        continue;
//       candidate = nodes.at(*it).fScore;
    }
    else {
      if (m.isCollision(nodes[*it].st) && mode != 1)
        if (mode == 1)
          candidate += 10000; // Collision penalty
        else
          continue;
      candidate = nodes.at(*it).fScore ;
      
    }
    if (candidate < min) {
      min = candidate;
      ret_val = *it;
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
  m.header.frame_id = "/map";
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
  
  for (auto it = path.begin(); it != path.end(); it++) {
    geometry_msgs::Twist command;
    command.linear.x = it->command_lin;
    command.angular.z = it->command_ang;
    AStarState st = it->st;
    m.integrate(st, command, 1.0, true);
    ret = m.testIntegration(st, true, false);
  }
  
  return ret;
}



#endif
