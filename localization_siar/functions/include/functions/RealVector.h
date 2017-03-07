#ifndef __REAL_VECTOR_H__
#define __REAL_VECTOR_H__

#include <string>
#include <sstream>
#include <cmath>
#include <iostream>
#include <vector>

/** @brief 3D Point structure and operators

 */
#include "Point3D.h"

namespace functions {

class RealVector:public std::vector<double> {
  protected:
    
    //! init without arguments -> all internal data to empty or zero values
    void init();
    
    //! @brief Initializing method from a standard vector
    //! @param vec Std vector of double. If not enough values --> the rest are set to zero. Discards excessive values
    void init(const std::vector<double> &vec);
    
    //! @brief Initializer from point 3D
    //! @param p The point
    void init(const Point3D &p);
    
    void init(const RealVector &v);
    
    void init(int dim, bool random = false);
            
  public:
      
      
    RealVector();
  
    //! @brief Constructor from a std vector.
    //! @param vec Std vector of double. If not enough values --> the rest are set to zero. Discards excessive values
    RealVector(const std::vector<double> &vec);
                
    //! @brief Constructor from an array of double
    RealVector(const double* vec, size_t size);
    
    //! @brief Constructor from a std string.
    //! @param st The string. The format has to be: (n1, n2, ..., nm)
    RealVector(const std::string &st);
    
    //! @brief Constructor from Point 3D
    //! @param p The point
    RealVector (const Point3D &p);

    //! @brief Constructor with a specified dimension
    //! @param dim The dimension
    //! @param random If true, each component is initialized to a random number with U(0,1)
    RealVector (int dim, bool random = false);
    
    RealVector(const RealVector &v);
    
    //!Returns a string representation of the object
    std::string toString() const;
    
    //!Returns a MATLAB-compatible representation of the object
    std::string toMatlabString() const;
    
    void fromString(const std::string &s);
    
    //!addition operator (sums each coordinate)
    friend const RealVector operator+(const RealVector &left, const RealVector &right);
    
    //!substraction operator(substracts each coordinate)
    friend const RealVector operator-(const RealVector &left, const RealVector &right);
    
    //!multiply operator(multiplies each coordinate by a constant)
    friend const RealVector operator*(const RealVector &left, const double &right);
    
    //!dot product operator
    friend  double operator*(const RealVector &left, const RealVector &right);
    
    //!exponentiation operator( exponentiates each coordinate to a given power)
    friend const RealVector operator^(const RealVector &left, const double &right);
    
    //!scalar division  operator
    friend const RealVector operator/(const RealVector &left, const double &right);
    
    //!operator opposite (changes sign to each coordinate)
    friend const RealVector operator-(const RealVector &a);
    
    //! assignment operator
    RealVector & operator = (const RealVector &that);

    //! comparison operator
    bool operator == (const RealVector &other) const;
    
    double distance(const RealVector &p)const;
    double norm()const;
    
    double angle(const RealVector &other) const;
    
    //! @brief Calculates the distance between the point and a segment with vertices s1 and s2
    double distanceToSegment(const RealVector &s1, const RealVector &s2) const;
    
    //! @brief Calculates the cross product between two tridimensional vectors.
    //! \NOTE The vectors must be tridimensional
    //! @param p A 3D vector
    //! @return The cross product (this x p) (note that is not conmutative)
    //! @throws A runtime exception if the input vectors are not tridimensional
    RealVector crossProduct(const RealVector &p)const;

    //! @brief Makes the product component by component of two vectors.
    //! \note The vector must be of the same dimension.
    //! @param p The other vector
    //! @return Another vector with the same dimension. Each components will be obtained by multiplying the
    //! @return same components of the other vectors.
    //! @throws A runtime exception if the input vectors are not of the same dimension
    RealVector componentProduct(const RealVector &p) const;
    
    //! @brief normalizes the vector
    inline void normalize() {
      double norm_ = norm();
      
      for (unsigned int i = 0; i < size(); i++) {
        (*this)[i] /= norm_;
      }
    }
    
};

double pathLength(const std::vector<RealVector> &v);

}

#endif //__REAL_VECTOR_3D_H__
