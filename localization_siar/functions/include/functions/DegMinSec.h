#ifndef __DEG_MIN_SEC_H__
#define __DEG_MIN_SEC_H__

#include <string>
#include <sstream>
#include <cmath>

/** @brief This class stores an angle with DEG, MIN, SEC representation

 */

namespace functions {

class DegMinSec{
		
		
	public:
		
	//! Internal data
		int deg;
		int min;
		double sec;

		enum EntryMode {
			DEGREES, RADIANS
		};
			
	
	protected:
		
		//! Disposal of resources (erases contents and restart)
		void dispose () throw ();
		
		//! init without arguments -> all internal data to empty or zero values
		void init();
		
		/** Initializing method
		 * @param d internal data
		 * @param m internal data
		 * @param s internal data
		*/
		void init(const int &d, const int &m, const double &s);	
						
	public:
			
		//! @brief Gets the translation to Decimal Degrees
            //! @return The value stored in the class in decimal degrees
		double toDecimalDeg() const;  

		//! @brief Gets the value translated to radians
            //! @return The angle value translated to radians
		double toRadians() const;  

		
		DegMinSec();
		/**Constructor from degrees, minutes and seconds. Degrees are signed. */
		DegMinSec(const int &d, const int &m, const double &s);
	
		DegMinSec (const DegMinSec &that);
		
		//!Constructor from decimal degrees and radians
		//! @param d Double storing the decimal degrees or radians (depends on the mode)
		DegMinSec (const double &d, EntryMode e = DEGREES);
		
		//!Destructor
		~DegMinSec();
		
		//! Prefer clone to copy constructor.
		//! @return A std::string that represents the info stored in the class
		DegMinSec * clone () const;  
			
		//! @brief Returns a string representation of the object
		//! @return A std::string that represents the info stored in the class
		std::string toString();
		
		//!@brief Returns a string representation of the object in decimal minutes
		//! @return A std::string that represents the info stored in the class
		std::string toStringDecMin();
		
		//! assignment operator
		DegMinSec & operator = (const DegMinSec &that);
};

}

#endif //__DEG_MIN_SEC_H__
