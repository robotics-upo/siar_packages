#include "functions/RealVector.h"
#include "functions/functions.h"
#include <iostream>
#include <iomanip>

using namespace functions;
using namespace std;

int main(int argc, char **argv) {
  if (argc != 2) {
    cerr << "Usage: " << argv[0] << " <test_file>\n";
  }
  double epsilon = 1e-10;
  
  
  vector<vector<double> > M;
  getMatrixFromFile(argv[1], M);
  
  RealVector a(M[0]);
  RealVector b(M[1]);
  RealVector p(M[2]);
  
  double dist = p.distanceToSegment(a,b);
  
  cout << setprecision(20);
  cerr << setprecision(20);
  
  if (M.size() > 3) {
    if ( fabs(dist - M[3][0]) > epsilon) {
      
      cerr << "Test Failed. Expected value: " << M[3][0] << ". Obtained value: " << dist << endl;
    } else {
      cout << "Test passed. Expected value: " << M[3][0] << ". Obtained value: " << dist << endl;
    }
    
  } else {
    cout << "Distance: " << dist << endl;
    
  }
  
  
  
  return 0;
}