//! This file describes the actions of a class that stores the actual time data
//!
//! By David Alejo Teissiere
//!
//! Start date:  19-1-09
//! Last update: 26-6-09

#include "FormattedTime.h"
#include <cstring>
#include <math.h>

using namespace std;

#define MAX_REPRESENTATION_CHARS 30
#define USECS_IN_A_SEC (1000000)

namespace functions {
  
const std::string FormattedTime::format_no_date = "%T.";

// Constructor with parameters and default
FormattedTime::FormattedTime( TimeResolution res, const string &f ) {
	setFormat( f );
	resolution = res;
}

// It gets the actual time
void FormattedTime::getTime() {
	gettimeofday(&actual_time, NULL);
}

// It sets the representation format
void FormattedTime::setFormat(const string &f) {
	format = f;
}

// It returns a buffer with the formatted date and time
string FormattedTime::getFormattedTime(bool represent_date) const {
	string s;
	switch (resolution) {
		case SECONDS:
			s = getFormattedTimeS(represent_date);
			
		case MILLISECONDS:
			s = getFormattedTimeMS(represent_date);
		
		case MICROSECONDS:
			s = getFormattedTimeUS(represent_date);
	}
	return s;
}

// Returns a buffer with the formatted date and time in seconds
string FormattedTime::getFormattedTimeS (bool represent_date) const {
	char *buffer=new char [MAX_REPRESENTATION_CHARS];
	time_t curtime = actual_time.tv_sec;
	string format = this->format;
	
	if (!represent_date) {
	  format = format_no_date;
	}
	
	strftime(buffer, MAX_REPRESENTATION_CHARS, format.c_str(), localtime(&curtime));
	
	string ret_value = buffer;
	
	// Release memory
	delete[] buffer;
	
	return ret_value;
}

// Returns a buffer with the formatted date
string FormattedTime::getFormattedTimeUS (bool represent_date) const {
	char *buffer=new char [MAX_REPRESENTATION_CHARS];
	time_t curtime = actual_time.tv_sec;
	string format = this->format;
	
	if (!represent_date) {
	  format = format_no_date;
	}
	
	strftime(buffer, MAX_REPRESENTATION_CHARS, format.c_str(), localtime(&curtime));
	sprintf(buffer, "%s%06ld", buffer, actual_time.tv_usec);
	
	string ret_value = static_cast<string>(buffer);
	
	// Release memory
	delete[] buffer;
	
	return ret_value;
}

// Returns a buffer with the formatted date
string FormattedTime::getFormattedTimeMS (bool represent_date) const {
	char *buffer=new char [MAX_REPRESENTATION_CHARS];
	time_t curtime = actual_time.tv_sec;
	string format = this->format;
	
	if (!represent_date) {
	  format = format_no_date;
	}
	
	strftime(buffer, MAX_REPRESENTATION_CHARS, format.c_str(), localtime(&curtime));
	sprintf(buffer,"%s%03ld", buffer, actual_time.tv_usec/1000);
	string ret_value = static_cast<string>(buffer);
	
	// Release memory
	delete[] buffer;
	
	return ret_value;
}

// Comparison functions
bool operator < ( const FormattedTime &t1, const FormattedTime &t2 ) {
	return ( t1.actual_time.tv_sec < t2.actual_time.tv_sec || 
				 ( t2.actual_time.tv_sec == t1.actual_time.tv_sec && 
					 t1.actual_time.tv_usec < t2.actual_time.tv_usec)  ) ;
}

bool operator > ( const FormattedTime &t1, const FormattedTime &t2 ) {
	return ( t1.actual_time.tv_sec > t2.actual_time.tv_sec || 
				( t2.actual_time.tv_sec == t1.actual_time.tv_sec && 
					t1.actual_time.tv_usec > t2.actual_time.tv_usec   ) );
}

bool operator == ( const FormattedTime &t1, const FormattedTime &t2 ) {
	return ( 	t1.actual_time.tv_sec == t2.actual_time.tv_sec && 
						t1.actual_time.tv_usec == t2.actual_time.tv_usec );
}

// FormattedTime substract operator
// Computes the difference between two instants
double operator - ( const FormattedTime &t1, const FormattedTime &t2 ) {
	double aux;
	
	// Get the diference, considering the microseconds
	aux = (t1.actual_time.tv_usec - t2.actual_time.tv_usec) / 1e6;
	aux += t1.actual_time.tv_sec - t2.actual_time.tv_sec;

	return aux;
}

FormattedTime operator + ( const FormattedTime &t1, double second_increment ) {
  FormattedTime ret(t1);

  ret.actual_time.tv_sec += static_cast<int>(floor(second_increment));
  ret.actual_time.tv_usec += (second_increment - floor(second_increment))* 1e6;
  if (ret.actual_time.tv_usec > 1000000) {
    ret.actual_time.tv_usec -= 1000000;
    ret.actual_time.tv_sec++;
  }

  return ret;
}

ostream& operator << ( ostream &os, const FormattedTime &t1 ) {
	os << t1.getFormattedTime();
	
	return os;
}

} // Namespace functions
