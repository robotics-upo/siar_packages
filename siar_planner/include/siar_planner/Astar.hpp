#ifndef _ASTAR_HPP_
#define _ASTAR_HPP_

#include <unordered_map>
#include <unordered_set>

#include "siar_planner/AStarState.hpp"
#include "siar_planner/AStarNode.h"
#include "siar_planner/AStarModel.hpp"

#include <ros/ros.h>

class AStar
{
public:
  AStar(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  double getPath(AStarState start, AStarState goal, std::list<AStarState>& path);      
  
protected:
  AStar();
  
  

  void addEdge(int id0, int id1, double cost);
  
  int addNode(AStarState st);
  void addEdge(int id0, int id1);
  void clear();
  
  void expandNode(int base_id);
  
  double heuristic_cost(const AStarNode &a, const AStarNode &b) const;
  
  int getBestNode() const;
  
  //! @brief Searchs for a state in the graph and returns its ID if found
  int getCellID(const AStarState &st) const;

  std::unordered_map<int, AStarNode> nodes;
  
  bool isSameCell(const AStarState& s1, const AStarState& s2) const;
  
  int K, n_iter;
  double delta_t, cellsize_m, cellsize_rad;
  int last_id;
  
  std::unordered_set<int> closedSet;
  std::unordered_set<int> openSet;
  
  AStarModel m;
};

AStar::AStar(ros::NodeHandle &nh, ros::NodeHandle &pnh):m(nh, pnh)
{
  if (!pnh.getParam("K", K)) {
    K = 4;
  }
  if (!pnh.getParam("n_iter", n_iter)) {
    n_iter = 100;
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
  
}


inline
void AStar::clear()
{
  nodes.clear();
}

inline
int AStar::addNode(AStarState st)
{
  int id = getCellID(st);
  if (id < 0) {
    AStarNode &n = nodes[nodes.size()];
    n.id = nodes.size() - 1;
    n.st = st; 
    id = n.id;
  }
  
  return id;
}


inline
void AStar::addEdge(int id0, int id1, double cost)
{
  nodes[id0].neighbors[id1] = cost;  
}


inline
double AStar::getPath(AStarState start, AStarState goal, std::list<AStarState>& path)
{
  nodes.clear(); // Incremental algorithm --> the graph is generated in each calculation
  path.clear();
  openSet.clear();
  closedSet.clear();
  
  
  
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
  while (cont < n_iter && !openSet.empty()) {
    
    // Get node ID with minimum cost
    int current_id = getBestNode();
    
    if (current_id < 0) {
      std::cerr << "ID < 0 --> Openset empty" << std::endl;
      break;
    }
    
    AStarNode* current_node = &nodes[current_id];    
    
    if (current_id == goal_id) {
      // Retrieve path
      path.clear();
      AStarState current_point = current_node->st;
      ret_val = current_node->fScore;
      path.push_front(current_point);
      while (current_node->parent != NULL) {
        current_node = current_node->parent;
        current_point.state = current_node->st.state;
        path.push_front(current_point);
      }
      return 1.0;
    }
    // Expands the graph from the current node --> Draws edges according to the inputs  
    expandNode(current_id);
    openSet.erase(current_id);          
    closedSet.insert(current_id);
    for (auto it = current_node->neighbors.begin(); it != current_node->neighbors.end(); ++it) {
      int neighbor_id = it->first;
      double neighbor_cost = it->second;
      
      if (closedSet.count(neighbor_id)>0) {
        continue;
      }

      double tentative_gScore = current_node->gScore + neighbor_cost;
      AStarNode *neighbor_node = &nodes.at(neighbor_id);
      if (openSet.count(neighbor_id)==0) {
        openSet.insert(neighbor_id);
      } else if (tentative_gScore >= neighbor_node->gScore) {
        continue;
      }
      neighbor_node->parent = current_node;
      neighbor_node->gScore = tentative_gScore;
    }
  }  
  return ret_val;
}


double AStar::heuristic_cost(const AStarNode& a, const AStarNode& b) const
{
  return a.st.state.distance(b.st.state);
}

void AStar::expandNode(int base_id)
{
  AStarNode &n = nodes[base_id];
  
  for (int i = 0; i < K; i++) {
    AStarState st = n.st;
    geometry_msgs::Twist command = m.generateRandomCommand();
    double cost = m.integrate(st, command, delta_t);
    
    if (cost < 0.0) {
      // Collision
      // Update??
      
    } else {
      // Add the vertex to the graph
      int neighID = addNode(st);
      addEdge(base_id, last_id, cost);
      
    }
  }
}

int AStar::getBestNode() const
{
  double min = std::numeric_limits<double>::infinity();
  int ret_val = -1;
  auto current_it = openSet.begin();
  for (auto it = openSet.begin(); it != openSet.end(); ++it) {
    if (nodes.at(*it).fScore < min) {
      current_it = it;
      min = nodes.at(*it).fScore;
    }
  }
  int current_id = *current_it;
  
  return ret_val;
}

int AStar::getCellID(const AStarState& st) const
{
  int ret_val = -1;
  
  for (int i = 0; i <= last_id && ret_val < 0; i++) {
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
      
      ret_val = fabs(round(s1.state[i] / cellsize) - round(s2.state[i]/cellsize)) < (cellsize / 2.0);
    }
  }
  
  return ret_val;
}


#endif
