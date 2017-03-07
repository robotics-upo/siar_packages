#ifndef SEWER_GRAPH_H
#define SEWER_GRAPH_H

#include <string>
#include "config.h"
#ifdef USE_KML
#include <kml/dom.h>
#endif

#include "functions/DegMinSec.h"
#include "functions/RealVector.h"
#include "simple_graph/SimpleGraph.h"
#include "sewer_graph/earthlocation.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/NavSatFix.h>

namespace sewer_graph {
  
  
enum SewerVertexType {
  MANHOLE,
  FORK,
  
  ALL = 255
};

// Types
struct SewerVertex {
  EarthLocation e;
  std::string comments;
  
  double x, y; // Local coordinates
  
  std::string toString() const;
  SewerVertexType type;
};

struct SewerEdge {
  double distance;
  double route; // Angle relative to N
  
  std::string toString() const;
};
  
  


//! @brief This class is used to represent an Earth Location in terms of longitude and latitude.
//! Uses decimal degrees as internal representation of latitude and longitude.
//! The altitude is stored in meters above the sea level.
//! ToString method gives many representations.
class SewerGraph:public simple_graph::SimpleGraph<SewerVertex, SewerEdge>
{
public:
   //! @brief Constructor from file
  SewerGraph(const std::string &filename); 
  
  //! @brief Loads graph from a file
  bool loadGraph(const std::string &filename);
  
//   std::string toString() const;        Por ahora usamos la versión de SimpleGraph
  
  double getDistanceToClosestManhole(double x, double y) const;
  
  double getDistanceToClosestVertex(double x, double y, SewerVertexType type) const;
  
  int getClosestVertex(double x, double y, SewerVertexType type = ALL) const;
  
  double getDistanceToClosestEdge(double x, double y) const;
  
  //! @brief Exports the trajectory information into KML format
  //! @param filename Output filename
  //! @retval true The information has been successfully saved
  //! @retval false Errors while saving information
  bool exportKMLFile(const std::string &filename) const;
  
  //! Grief Exports the Graph in RViz format
  std::vector<visualization_msgs::Marker> getMarkers(std::string ref_frame) const;
  
  sensor_msgs::NavSatFix getReferencePosition() const;

protected:
  EarthLocation center;
  // KML stuff -------------------------------------------------
#ifdef USE_KML  
  kmldom::KmlFactory* factory;
  
  void addKMLStyle(kmldom::DocumentPtr& doc) const;
#endif
  // End of KML stuff -----------------------------------------------
};


}

#endif // EARTHLOCATION_H

