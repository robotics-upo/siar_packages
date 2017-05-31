#ifndef _ASTAR_HPP_
#define _ASTAR_HPP_

#include <unordered_map>
#include <unordered_set>

#include "siar_planner/AStarState.hpp"
#include "siar_planner/AStarNode.h"
#include "siar_planner/AStarModel.hpp"

class AStar
{
public:
  AStar (ros::nodeHandle &nh, ros::nodeHandle &pnh);
  double getPath(AStarState start, AStarState goal, std::list<AStarState>& path);      
  
protected:
  AStar();
  
  

  void addEdge(int id0, int id1, double cost);
  
  void addNode(int id, AStarState st);
  void addEdge(int id0, int id1);
  void clear();
  
  void expandNode(int base_id);
  
  double heuristic_cost(const AStarNode &a, const AStarNode &b) const;
  
  int getBestNode() const;

  std::unordered_map<int, AStarNode> nodes;
  
  int K, n_iter;
  double delta_t;
  
  AStarModel m;
};

AStar::AStar(ros::nodeHandle &nh, ros::nodeHandle &pnh):m(nh, pnh)
{
  if (!pnh.getParam("K", K)) {
    K = 4;
  }
  if (!pnh.getParam("n_iter", n_iter)) {
    n_iter = 100;
  }
  if (!pnh.getParam("delta_t", delta_t)) {
    delta_t = 0.2;
  }
}


inline
void AStar::clear()
{
  nodes.clear();
}

inline
void AStar::addNode(int id, AStarState st)
{
  AStarNode &n = nodes[id];
  n.id = id;  
  n.st = st;
}


inline
void AStar::addEdge(int id0, int id1, double cost)
{
  nodes[id0].neighbors[id1]=cost;  
}


inline
double AStar::getPath(AStarState start, AStarState goal, std::list<AStarState>& path)
{
  int start_id = 0;
  int goal_id = 1;
  addNode(start, start_id);
  addNode(goal, goal_id);
  double ret_val = -1.0;
  
  AStarNode* start_node = &nodes.at(start_id);
  AStarNode* goal_node = &nodes.at(goal_id);
  path.clear();
  
  for (auto it = nodes.begin(); it != nodes.end(); ++it) {
    it->second.gScore = std::numeric_limits<double>::infinity();
    it->second.fScore = std::numeric_limits<double>::infinity();
    it->second.parent = NULL;
  }
  
  start_node->gScore = 0;
  start_node->fScore = heuristic_cost(start_node,goal_node);  
  
  int cont = 0;
  
  while (cont < n_iter) {
    
    // Get node ID with minimum cost
    int current_id = getBestNode();
    
    if (current_id < 0) {
      std::cerr << "ID < 0 --> this should not happen" << std::endl;
      break;
    }
    
    AStarNode* current_node = nodes[current_id];    
    
    if (current_id == goal_id) {
      // Retrieve path
      path.clear();
      AStarState current_point = current_node->st;
      ret_val = current_node->c_minus;
      path.push_front(current_point);
      while (current_node->parent != NULL) {
        current_node = current_node->parent;
        current_point.state = current_node->st.state;
        path.push_front(current_point);
      }
    } else {
      // Expands the graph from the current node --> Draws edges according to the inputs and 
      expandNode(current_id);
    
      for (auto it = current_node->neighbors.begin(); it != current_node->neighbors.end(); ++it) {
        int neighbor_id = it->first;
        double neighbor_cost = it->second;
        
        
        double tentative_gScore = current_node->gScore + neighbor_cost;
        AStarNode *neighbor_node = &nodes.at(neighbor_id);
        if (openSet.count(neighbor_id)==0) {
          openSet[neighbor_id] = neighbor_node;
        } else if (tentative_gScore >= neighbor_node->gScore) {
          continue;
        }
        neighbor_node->parent = current_node;
        neighbor_node->gScore = tentative_gScore;
      }
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
    double cost = m.integrate(n.st,  m.generateRandomCommand(), delta_t);
    
    if (cost < 0.0) {
      // Collision
      // Update??
      
    }
  }
}

int AStar::getBestNode() const
{
  int ret_val = -1;
  
  double min_val = std::numeric_limits<double>::infinity();
  
  for (auto id: nodes) {
    if (id.second.c_plus < min_val) {
      ret_val = id.first;
      min_val = id.second.c_plus;
    }
  }
  
  return ret_val;
}


#endif
