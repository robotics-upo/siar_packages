#ifndef CLASE_H
#define CLASE_H

#include "Point3D.h"

/** @brief Class brief description

 */

namespace functions {

class HessianPlane{
		
		
	public:
	
		//! Normal vector of the plane
		Point3D *planeVec;
		//! distance of the plane to the origin (0.0 , 0.0, 0.0)
		double p;
	
	protected:
		//! Default constructor is protected, use another one please
		HessianPlane(){init();}
				
		//! Disposal of resources (erases contents and restart)
		void dispose () throw ();
		
		//! init without arguments -> all internal data to empty or zero values
		void init();
		
		/** Initializing method
		 * @param d internal data
		*/
		void init(const Point3D &pv, const double &pp);	
		
		//! @brief Initializes the plane. that will be. normal to the vector ab and contains b.
		//! @param a First point
		//! @param b Second point.
		void init(const Point3D &a, const Point3D &b);
						
	public:
			
	
	
	/** Preferred constructor 
	 * 
	 * @param d internal data
	*/
		HessianPlane(const Point3D &pv, const double &pp);
		
		//! @brief Constructor from two points. The plane is normal to the vector ab and contains b.
		//! @param a First point
		//! @param b Second point.
		HessianPlane(const Point3D &a, const Point3D &b);
		
	
		//!Destructor
		~HessianPlane();
		
		//! Prefer clone to copy constructor.
		HessianPlane * clone () const;  
			
		//!Returns a string representation of the object
		std::string toString();
		
	private:
		//! Hiding default shallow copy of copy constructor
		HessianPlane (const HessianPlane &that);
		//! Hiding assignment operator
		HessianPlane & operator = (const HessianPlane &that);

		
};

}

#endif //CLASE_H 
