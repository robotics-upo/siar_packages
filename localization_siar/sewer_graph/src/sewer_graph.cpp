#include "sewer_graph/sewer_graph.h"
#include "functions/functions.h"
#include "functions/RealVector.h"
#include <iostream>

#include <ros/ros.h>

#ifdef USE_KML
#include "kml/base/file.h"
#include "kml/base/math_util.h"
#include "kml/dom.h"
#include "kml/engine.h"
#include "kml/convenience/convenience.h"
#include "kml/base/string_util.h"

using kmldom::CoordinatesPtr;
using kmldom::KmlFactory;
using kmldom::LineStringPtr;
using kmldom::PlacemarkPtr;
using kmldom::KmlPtr;
using kmlengine::KmlFile;
using kmlengine::KmlFilePtr;
#endif

using namespace std;
using namespace functions;

typedef vector<vector<double> >  Matrix;

namespace sewer_graph {
  
  
// ------------- Methods of SewerVertex and SewerEdge
  
  
string SewerEdge::toString() const
{
  ostringstream os;
  os << distance << "\t" << route ;
  return os.str();
}


string SewerVertex::toString() const
{
  ostringstream os;
//   os <<  e.toString(' ') ;
  os << x << ", " << y;
  return os.str();
}

ostream& operator << (ostream &o, const SewerVertex &v) {
  o << v.toString();
  return o;
}

ostream& operator << (ostream &o, const SewerEdge &e) {
  o << e.toString();
  return o;
}


// ---------------------- Methods of Sewer Graph

  SewerGraph::SewerGraph(const string& filename)
{
  loadGraph(filename);
}

bool SewerGraph::loadGraph(const string& filename)
{
  bool ret_val = true;
  Matrix info;
  
  try {
    getMatrixFromFile(filename, info);
    unsigned int i;
    for (i = 0; i < info.size(); i++) {
      if (info[i].size() < 2) {
        break; // Node ID invalid --> lets get the Edges
      }
      EarthLocation e(info[i][0], info[i][1]);
      
      if (i == 0) {
        center = e; // Initialize the center as the first loaded vertex
      }
      
      SewerVertex v;
      v.e = e;
      
      RealVector rel = e.toRelative(center);
      v.x = rel[0];
      v.y = rel[1];
      
      if (info[i].size() > 2) {
        v.type = (SewerVertexType)info[i][2];
      } else {
        v.type = MANHOLE;
      }
      
      v.comments = "";
      addVertex(v);
    }
    
    SewerEdge e;
    for (;i < info.size(); i++) {
      if (info[i].size() >= 2) {
        cout << "Adding edge: " << info[i][0] << " to " << info[i][1] << "\t";
        EarthLocation origin(getVertexContent(info[i][0]).e);
        EarthLocation dest(getVertexContent(info[i][1]).e);
        
        e.distance = origin.distance(dest);
        RealVector v = dest.toRelative(origin);
        e.route = atan2(v[1], v[0]);
        cout << "v = " << v.toString() << "\tRoute: " << e.route << endl;
        addEdge(info[i][0], info[i][1], e);
      }
      
    }
    
  } catch (std::exception &e) {
    cerr << "Sewer::loadGraph --> Error while loading file: " << filename << "\n";
    
  }
  
  return ret_val;
}

// ----------------- Calculations -------------------------------------------

int SewerGraph::getClosestVertex(double x, double y, SewerVertexType type) const {
  int ret_val = -1;
  double closest_dist = 1e100;
  for (int i = 0; i < nVertices(); i++) {
    SewerVertex v = getVertexContent(i);
    if (type != ALL && v.type != type) 
      continue;
    double dist = (x - v.x)*(x - v.x) + (y - v.y)*(y - v.y);
    if (dist < closest_dist) {
      ret_val = i;
      closest_dist = dist;
    }
  }
  return ret_val;
}

double SewerGraph::getDistanceToClosestManhole(double x, double y) const {
  return getDistanceToClosestVertex(x, y, MANHOLE);
}
  
double SewerGraph::getDistanceToClosestVertex(double x, double y, SewerVertexType type) const {
  double ret_val = -1.0;
  
  int manhole = getClosestVertex(x, y, type);
  if (manhole >= 0) {
    SewerVertex v = getVertexContent(manhole);
    ret_val = sqrt( (x - v.x)*(x - v.x) + (y - v.y)*(y - v.y));
  }
  
//   std::cout << "Distance to closest vertex: " << ret_val << std::endl;
  
  return ret_val;
}

double SewerGraph::getDistanceToClosestEdge(double x, double y) const {
  double ret_val = -1.0;
  
  RealVector pos;
  pos.push_back(x);
  pos.push_back(y);
  
  RealVector v1(2),v2(2);
  for (int i = 0; i < nVertices(); i++) {
    SewerVertex v = getVertexContent(i);
    v1[0] = v.x;v1[1] = v.y;
    
    // iterate the edges
    std::list<int> n = vertices[i].getNeighbours();
    std::list<int>::iterator it = n.begin();
    
    for (;it != n.end(); it++) {
      //! @brief Calculates the distance between the point and a segment with vertices s1 and s2
      v = getVertexContent(*it);
      v2[0] = v.x;v2[1] = v.y;
      double dis = pos.distanceToSegment(v1, v2);
      if (ret_val < 0.0 || ret_val > dis) {
        ret_val = dis;
      }
      
    }
  }
  
  return ret_val;
}


// ----------------- Representation stuff -------------------------------------

bool SewerGraph::exportKMLFile(const string& filename) const 
{
  bool ret = true;
  
  // Create document. This is necessary to handle styles
  kmldom::DocumentPtr doc = factory->CreateDocument();
  
  // Generates the styles and retale them to the doc if the option is choosen
  addKMLStyle(doc);
      
  for (unsigned int i = 0; i < nVertices(); i++) {
    EarthLocation from = getVertexContent(i).e;
    ostringstream os;
    os << "Vertex " << i;
    doc->add_feature(from.getKMLPlacemark(os.str()));  // kml takes ownership.
    
    // Then iterate the edges
    std::list<int> n = vertices[i].getNeighbours();
    std::list<int>::iterator it = n.begin();
    for (;it != n.end(); it++) {
      int j = *it;
      ostringstream os_2;
      os_2 << "Edge from " << i << " to " << j;
      EarthLocation to = getVertexContent(j).e;
      doc->add_feature(from.getKMLLine(os_2.str(), to, factory));
    }
  }

  
  
  // Create the kml pointer with the document
  KmlPtr kml = factory->CreateKml();
  kml->set_feature(doc);
  
  // Now the file and serialize it to string
  KmlFilePtr kmlfile = KmlFile::CreateFromImport(kml);
  if (!kmlfile) {
    cerr << "error: could not create kml file" << endl;
    return false;
  }
  std::string kml_data;
  kmlfile->SerializeToString(&kml_data);
  
  // Once we get the string --> write it to a file
  if (!kmlbase::File::WriteStringToFile(kml_data, filename.c_str())) {
    cerr << "error: write of " << filename << " failed" << endl;
    ret = false;
  }
  
  return ret;
}

void SewerGraph::addKMLStyle(kmldom::DocumentPtr& doc) const
{
  // Create the style 
  kmldom::StylePtr normal = factory->CreateStyle();
  normal->set_id("normal");
  kmldom::LineStylePtr linestyle = factory->CreateLineStyle();
  linestyle->set_width(5.0);
  kmlbase::Color32 red("red");
  linestyle->set_color(red);
  normal->set_linestyle(linestyle);
  doc->add_styleselector(normal);
        
  kmldom::StyleMapPtr stylemap = factory->CreateStyleMap();
  stylemap->set_id("linestylemap");
  kmldom::PairPtr pair = factory->CreatePair();
  pair->set_key(kmldom::STYLESTATE_NORMAL);
  pair->set_styleurl("#linestyle");
  stylemap->add_pair(pair);
        
  doc->add_styleselector(stylemap);
}

std::vector<visualization_msgs::Marker> SewerGraph::getMarkers(std::string ref_frame) const {
  visualization_msgs::Marker points, forks, lines;
  points.header.frame_id = ref_frame;
  points.header.stamp = ros::Time::now();
  points.ns = "sewer_graph";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  
  forks.header.frame_id = ref_frame;
  forks.header.stamp = ros::Time::now();
  forks.ns = "sewer_graph";
  forks.action = visualization_msgs::Marker::ADD;
  forks.pose.orientation.w = 1.0;
  forks.id = 1;
  forks.type = visualization_msgs::Marker::POINTS;
  
  lines.header.frame_id = ref_frame;
  lines.header.stamp = ros::Time::now();
  lines.ns = "sewer_graph";
  lines.action = visualization_msgs::Marker::ADD;
  lines.pose.orientation.w = 1.0;
  lines.id = 2;
  lines.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 2;
  points.scale.y = 2;
  forks.scale.x = 2;
  forks.scale.y = 2;
  lines.scale.x = 0.75;
  lines.scale.y = 0.75;

  // Manholes are green, forks red
  points.color.g = 1.0;
  points.color.a = 1.0;
  forks.color.r = 1.0;
  forks.color.a = 1.0;
  lines.color.r = 1.0;
  lines.color.g = 1.0;
  lines.color.b = 1.0;
  lines.color.a = 0.7;

  geometry_msgs::Point p, p1;
  
  for (unsigned int i = 0; i < nVertices(); i++) {
    SewerVertex v = getVertexContent(i);
    p.x = v.x;
    p.y = v.y;
    p.z = 0.0;
    
    if (v.type == MANHOLE) 
      points.points.push_back(p);
    else
      forks.points.push_back(p);
  
  // TODO: Represent edges
  
  
    std::list<int> n = vertices[i].getNeighbours();
    std::list<int>::iterator it = n.begin();
    
    for (;it != n.end(); it++) {
      v = getVertexContent(*it);
      p1.x = v.x;
      p1.y = v.y;
      p1.z = 0.0;
      lines.points.push_back(p);
      lines.points.push_back(p1);
    }
  }
  
  std::vector<visualization_msgs::Marker> ret;
  ret.push_back(lines);
  ret.push_back(points);
  ret.push_back(forks);
  
  
  return ret;
}

sensor_msgs::NavSatFix SewerGraph::getReferencePosition() const
{
  sensor_msgs::NavSatFix ret;
  
  if (nVertices() > 0) {
    ret.longitude = getVertexContent(0).e.getLongitude();
    ret.latitude = getVertexContent(0).e.getLatitude();
  }
  ret.status.service = 1;
  ret.status.status = 0;
  ret.header.seq = 0;
  ret.header.frame_id = "/map";
  ret.header.stamp = ros::Time::now();
  
  return ret;
}




// string SewerGraph::toString() const
// {
//   ostringstream os;
//   os << 
//   
// //   os << "Printing graph.\tN_vertices = " << nVertices() << "\tN_edges = " << nEdges() << std::endl;
// // 
// //   for ( unsigned int i=0; i < vertices.size(); i++) {
// //     os <<"Vertex: " << i << " " << getVertexContent(i) << std::endl;
// //     os << "Edges: \n";
// //     os << "Neighbours:\t";
// //     
// //     
// //     os << std::endl;
// //   }
// //  
// //         
// //   os << "End of the graph" << std::endl << std::endl;
//         
//   return os.str();
// }
//   
}