#ifndef __POINT_3D_H__
#define __POINT_3D_H__

#include <string>
#include <sstream>
#include <cmath>
#include <iostream>
#include <vector>

/** @brief 3D Point structure and operators

 */

namespace functions {

class Point3D{
		
		
	public:
		
	//! Internal data
		double x;
		double y;
		double z;
			
	
	protected:
		
		//! Disposal of resources (erases contents and restart)
		void dispose () throw ();
		
		//! init without arguments -> all internal data to empty or zero values
		void init();
		
	public:
		Point3D();
	
		Point3D(const double &px, const double &py, const double &pz);
	
		Point3D (const Point3D &that);
		
		//! @brief Constructor from a std vector.
		//! @param vec Std vector of double. If not enough values --> the rest are set to zero. Discards excessive values
		Point3D(const std::vector<double> &vec);
		
		//! @brief Initializing method from a standard vector
		//! @param vec Std vector of double. If not enough values --> the rest are set to zero. Discards excessive values
		void init(const std::vector<double> &vec);
		
		//!Destructor
		~Point3D();
		
		//! Prefer clone to copy constructor.
		Point3D * clone () const;  
			
		//!Returns a string representation of the object
		std::string toString(bool format = true) const;
		
		//!addition operator (sums each coordinate)
		friend const Point3D operator+(const Point3D &left, const Point3D &right);
		
		//!substraction operator(substracts each coordinate)
		friend const Point3D operator-(const Point3D &left, const Point3D &right);
		
		//!multiply operator(multiplies each coordinate by a constant)
		friend const Point3D operator*(const Point3D &left, const double &right);
		
		//!dot product operator
		friend  double operator*(const Point3D &left, const Point3D &right);
		
		//!exponentiation operator( exponentiates each coordinate to a given power)
		friend const Point3D operator^(const Point3D &left, const double &right);
		
		//!scalar division  operator
		friend const Point3D operator/(const Point3D &left, const double &right);
		
		//!operator opposite (changes sign to each coordinate)
		friend const Point3D operator-(const Point3D &a);
		
		//! assignment operator
		Point3D & operator = (const Point3D &that);
		
		double distance(const Point3D &p)const;
		double distance2d(const Point3D &p) const;
		double norm()const;
		
		Point3D crossProduct(const Point3D &p)const;
		
		//! @brief normalizes the vector
		inline void normalize() {
			double norm_ = norm();
			
			x /= norm_;
			y /= norm_;
			z /= norm_;
		}
		
		inline double getHeadingTo(const Point3D &p) const {
		  Point3D dif = p - *this;
		  
		  return dif.getHeading();
		}
		
		inline double getHeading() const {
		  return atan2(y, x);
		}
		

		/** Initializing method
		 * @param d internal data
		*/
		void init(const double &px, const double &py, const double &pz);	
};

}

#endif //__POINT_3D_H__
