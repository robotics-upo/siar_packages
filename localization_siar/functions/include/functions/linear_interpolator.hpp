#ifndef __FUNCTIONS__LINEAR__INTERPOLATOR__HPP
#define __FUNCTIONS__LINEAR__INTERPOLATOR__HPP

#include <map>
#include "functions.h"
#include <string>

// This code is based on the interpolation code of webpage: "http://www.bnikolic.co.uk/blog/cpp-map-interp.html" by B. Nikolic 

namespace functions {

class LinearInterpolator:std::map<double, double> {
public:
  //! @brief Gets the map from a file with a 2XN matrix
  LinearInterpolator(const std::string filename);
  //! @brief Gets the data
  LinearInterpolator(const std::string filename_x, const std::string filename_y);
  
  double interpolate(double x);
};

LinearInterpolator::LinearInterpolator(const std::string filename)
{
  std::vector<std::vector<double> > M;
  if (getMatrixFromFile(filename, 2, M)) {
    for (size_t i = 0; i < M.size(); i++) {
      insert(std::make_pair(M[i][0], M[i][1]));
    }
  }
}

LinearInterpolator::LinearInterpolator(const std::string filename_x, const std::string filename_y)
{
  std::vector<double> v0, v1;
  
  v0 = getVectorFromFile(filename_x);
  v1 = getVectorFromFile(filename_y);
  
  size_t len = std::min(v0.size(), v1.size());
  for (size_t i = 0; i < len; i++) {
    insert(std::make_pair(v0[i], v1[i]));
  }
  
}

double LinearInterpolator::interpolate(double x)
{
  typedef std::map<double, double>::const_iterator i_t;

  i_t i = upper_bound(x);
  if(i == end())
  {
    return (--i)->second;
  }
  if (i == begin())
  {
    return i->second;
  }
  i_t l = i; --l;

  const double delta = (x - l->first)/(i->first - l->first);
  return delta * i->second + (1 - delta) * l->second;
}

} // Namespace functions

#endif