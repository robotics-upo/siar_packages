#include "sewer_graph/sewer_graph.h"
#include <iostream>
#include <ros/ros.h>
#include "dxflib/dl_dxf.h"
#include "dxflib/dl_creationadapter.h"
#include "sewer_graph/test_creationclass.h"

using namespace std;
using namespace sewer_graph;

DL_Dxf* loadData(const std::string &filename, Test_CreationClass *creation_class);
void generateGraph(Test_CreationClass *creation_class, SewerGraph &g);
void newVertexFromGIS(double x, double y, SewerVertex &v);
double getVertexIDFromGIS(double x_, double y_);
double getVertexDistanceFromGIS(double x_, double y_);

double x,y, radius;
SewerGraph g;

int main(int argc, char ** argv) {
  if (argc < 7) {
    cerr << "Usage: " << argv[0] << " <dxf_filename> <out_file> <x> <y> <lat> <lon> <max_d>\n";
    return -1;
  }
  string out_file(argv[2]);
  x = atof(argv[3]);
  y = atof(argv[4]);
  EarthLocation center(atof(argv[5]), atof(argv[6]));
  radius = atof(argv[7]);
  
  string s(argv[1]);
  
  Test_CreationClass* creation_class = new Test_CreationClass();
  
  DL_Dxf* dxf_data = loadData(s, creation_class);
  if (dxf_data != NULL) {

    g.setCenter(center);
//     g.addEdge()
//     cout << g.toString_2() << endl; 
    cout << "File " << s << " loaded successfully\n";
    
    cout << "Loaded " << creation_class->trams.size() << " trams.\n";
    
    cout << "Generating graph.\n";
    generateGraph(creation_class, g);
    cout << "Generated the graph with " << g.nVertices() << " vertices and " << g.nEdges() << " edges.\n";
    if (g.writeGraph(out_file))
      cout << "File " << out_file << "generated successfully\n";
    else
      cerr << "Could not generate " << out_file << " file\n";
  } else {
    cerr << "Could not load " << s << endl;
    return -1;
  }
  
  
  
  return 0;
}


DL_Dxf* loadData(const std::string &filename, Test_CreationClass *creation_class) {
  //All ok proceed whit conversion
  std::cout << "Reading file " << filename << "...\n";
  DL_Dxf* dxf = new DL_Dxf();
  
  if (!dxf->in(filename, creation_class)) { // if file open failed
        std::cerr << filename << " could not be opened.\n";
        delete dxf;
        return NULL;
    }

  return dxf;
}

void generateGraph(Test_CreationClass *creation_class, SewerGraph &g) {
  SewerVertex v;
  
  std::vector<DL_InsertData> &d = creation_class->pous;
  
  for (size_t i = 0; i < d.size(); i++) {
    DL_InsertData &ins = d[i];
    if ( fabs(ins.ipx - x) < radius && fabs(ins.ipy - y) < radius) {
      newVertexFromGIS(ins.ipx, ins.ipy, v);
      v.type = sewer_graph::MANHOLE;
      g.addVertex(v);
    }
    
  }
  d = creation_class->entroncs;
  for (size_t i = 0; i < d.size(); i++) {
    DL_InsertData &ins = d[i];
    if ( fabs(ins.ipx - x) < radius && fabs(ins.ipy - y) < radius) {
      newVertexFromGIS(ins.ipx, ins.ipy, v);
      v.type = sewer_graph::FORK;
      g.addVertex(v);
    }
  }
  // TODO: use the trams to generate the Edges and further Vertices
 std::vector<std::vector <DL_VertexData> > &trams = creation_class->trams;
 int curr_ver, ant_ver;
  for (size_t i = 0; i < trams.size(); i++) {
    std::vector<DL_VertexData> &vertices = trams[i];
    SewerVertex v_s;
    for (size_t j = 0; j < vertices.size(); j++) {
      DL_VertexData &v = vertices[j];
      
      if ( fabs(v.x - x) > radius || fabs(v.y - y) > radius)
        break;
      
      if (getVertexDistanceFromGIS(v.x, v.y) < 1e-1) {
        curr_ver = getVertexIDFromGIS(v.x, v.y);
      } else {
        newVertexFromGIS(v.x, v.y, v_s);
        v_s.type = sewer_graph::NORMAL;
        curr_ver = g.addVertex(v_s);
      }
      if (j > 0) {
        g.addEdge(curr_ver, ant_ver);
      }
      ant_ver = curr_ver;
    }
  }
  
}

void newVertexFromGIS(double x_, double y_, SewerVertex& v)
{
  v.y = x_ - x;
  v.x = y_ - y;
  v.e = g.getCenter();
  v.e.shift(v.x, v.y);
}

double getVertexDistanceFromGIS(double x_, double y_) {
  return g.getDistanceToClosestVertex(y_ - y, x_ - x);
}

double getVertexIDFromGIS(double x_, double y_) {
  return g.getClosestVertex(y_ - y, x_ - x);
}
