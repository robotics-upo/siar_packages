#ifndef FORMATTED_TIME_H
#define FORMATTED_TIME_H

#include <sys/time.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>


namespace functions {
//! @brief This class stores time data and represents it properly.
//!
//! @author David Alejo Teissiere
//!
//! @date Start date:  19-1-09
//! @date Last update: 26-6-09
class FormattedTime {
public:
	//! @brief This enum indicates the resolution of the represented time.
	enum TimeResolution {
		SECONDS, MILLISECONDS, MICROSECONDS,
	};
	
	//! @brief Constructor with parameters and default
	//! @param res Resolution of the default representation ( MILLISECONDS...)
	//! @param format Format in wich the time is being presented ( "%Y-%m-%d-%T." as default )
	FormattedTime( TimeResolution res = MILLISECONDS, const std::string &format = "%Y-%m-%d;%T.");
	
	//! @brief Stores the current time
	void getTime();

	//! @brief Sets the time
	void setTime(long sec, long u_sec) {
	  actual_time.tv_sec = sec;
	  actual_time.tv_usec = u_sec;
	}
	
	//! @brief Sets the representation format
	//! @param new_format New format of the representation
	void setFormat( const std::string &new_format );
	
	//! @brief Function that return a string with the stored resolution
	//! @return A formatted string
	std::string getFormattedTime (bool represent_date = true) const;
	//! @brief Function that return a string with a resolution of seconds
	//! @return A formatted string
	std::string getFormattedTimeS (bool represent_date = true) const ;
	//! @brief Function that return a string with a resolution of miliseconds
	//! @return A formatted string
	std::string getFormattedTimeMS (bool represent_date = true) const;
	//! @brief Function that return a string with a resolution of microseconds
	//! @return A formatted string
	std::string getFormattedTimeUS (bool represent_date = true) const;
	
	//! @brief Function that get the values of the seconds field
	//! @return Seconds stored
	inline unsigned long getSec() const { return actual_time.tv_sec; }
	
	//! @brief Function that get the values of the microseconds field
	//! @return Microseconds stored
	inline unsigned long getUSec() const { return actual_time.tv_usec; }
	
	//! @brief Computes the difference between two instants.
	//! @retval The difference in seconds
	friend double operator - ( const FormattedTime &t1, const FormattedTime &t2 );

	//! @brief Computes the difference between two instants.
	//! @retval The difference in seconds
	friend FormattedTime operator + ( const FormattedTime &t1, double second_increment );
	
	//! @brief Writes the time to a ostream or to a string
	friend std::ostream& operator << ( std::ostream &os, const FormattedTime &t );

	//! @brief Time comparison (lower than)
	//! @param t1 First time
	//! @param t2 Second time
	//! @return true if t1 is lower than t2
	friend bool operator < ( const FormattedTime &t1, const FormattedTime &t2 );
	//! @brief Time comparison (greater than)
	//! @param t1 First time
	//! @param t2 Second time
	//! @return true if t1 is greater than t2
	friend bool operator > ( const FormattedTime &t1, const FormattedTime &t2 );
	//! @brief Time comparison (equal as)
	//! @param t1 First time
	//! @param t2 Second time
	//! @return true if t1 is equal as t2
	friend bool operator == ( const FormattedTime &t1, const FormattedTime &t2 );

private:
	const static std::string format_no_date;
	std::string format;
	struct timeval actual_time;
	TimeResolution resolution;
};

} // Namespace functions

#endif
