#include "functions/linear_interpolator.hpp"
#include <iostream>

using namespace functions;
using namespace std;

int main(int argc, char **argv) {
  
  if (argc < 3) {
    cout << "Usage: \n";
    cout << "\t" << argv[0] << "<x value> <Nx2 matrix filename>\n";
    cout << "\t" << argv[0] << "<x value> <x vector file> <y vector file>\n";
    return -1;
  }
  
  LinearInterpolator *lin = NULL;
  
  if (argc == 3) {
    lin = new LinearInterpolator(argv[2]);
    
  } else {
    lin = new LinearInterpolator(argv[2], argv[3]);
  }
   
  cout << "f(" << argv[1] << ") = " << lin->interpolate(atof(argv[1])) << endl;
  
  
  delete lin;
  
  return 0;
  
}