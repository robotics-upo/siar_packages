#ifndef ______FUNCTIONS__H______
#define ______FUNCTIONS__H______

#include <vector>
#include <list>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include "DegMinSec.h"
#include <time.h>

#ifdef _MSC_VER
 
 #include <Winsock2.h>
#else
 #include <sys/time.h>
#endif

namespace functions {
  
//! Rotates vector angle radians counterclockwise in two dimensions
//! @param v The vector
//! @param angle The angle in radians
void rotateVector(std::vector<double> &v, double angle);

//! Rotates vector angle degrees counterclockwise in two dimensions
//! @param v The vector
//! @param angle The angle in degminsec representation
void rotateVector(std::vector<double> &v, const DegMinSec &angle);

//! @brief Returns the angle in radians in the range of (-PI, PI]
//! @param angle The angle that will be reduced
//! @return The same angle but in (-PI, PI]
double reduceAngle(double angle);
	
std::vector<double>  getVectorFromFile(const std::string &fileName) throw ();

std::vector<double>  getVectorFromStream(std::istream &is);

std::vector<double>  getVectorFromString(const std::string &s);

bool getMatrixFromFile(const std::string &fileName, int width, std::vector<std::vector<double> > &v) throw ();

bool getMatrixFromFile(const std::string &fileName, std::vector<std::vector<double> > &v, bool discard_first = false) throw ();
  
template<typename T> std::string vectorToMatlabString(const std::string &vec_name, const std::vector<T> vec);
	
//! @brief Time representation function. Returns a std::string containing a time lapse.
//! @param t1 Time lapse start
//! @param t2 Time lapse end
//! @return The string containg the time lapse between start and end.
std::string showTime(struct timeval t1, struct timeval t2);
	
//! @brief Calculates the lapse time from two instants of time
//! @param t1 Time lapse start
//! @param t2 Time lapse end
//! @return A float value that indicates the seconds between t1 and t2
float calculateLapseTime(const struct timeval& t1, const struct timeval& t2);


//! @brief Returns a representation of the vector
//! @param value The value to search in the vector
//! @param v The vector
//! @return The index that means the position of the value item in the vector. If not found, it returns -1.
template<typename T> int searchInVector(const T &value, const std::vector<T> &v) {
	int ret_val = -1;
		
	for (unsigned int i = 0; i < v.size() && ret_val < 0; i++) {
		if ( value == v[i] ) {
			ret_val = i;
		}
	}
		
	return ret_val;
}

//! @brief Returns the minimum value taking two into account
//! @param a First value
//! @param b Second value
//! @return The minimum of a and b
template<typename t> inline t minimum(t a, t b) {
	return (a < b)?a : b;
}

//! @brief Returns the maximum value taking two into account
//! @param a First value
//! @param b Second value
//! @return The maximum of a and b
template<typename t> inline t maximum(t a, t b) {
	return (a < b)?b : a;
}

//! @brief Saturates a signal between two limits
//! @param Min The minimum value
//! @param Max The maximum value
template <typename t> inline t saturate(t v, t a, t b) {
  v = (v < b) ? v : b;
  v = (v > a) ? v : a;
  
  return v;
}

//! @brief Checks if the intervals overlap
//! @param first_interval First interval to check. The first value has to be lower than the second
//! @param second_interval Second interval to check. The first value has to be lower than the second
//! @retval true The intervals overlap
//! @retval false The intervals do not overlap
inline static bool overlap(const std::pair<double, double> &first_interval, const std::pair<double, double> &second_interval) {
  return ( first_interval.first <= second_interval.second) && ( first_interval.second >= second_interval.first);
}

////////////////////////////// REPRESENTATION BLOCK ////////////////////////////////////////////

//! @BRIEF Cool representation for boolean data
//! @param bool The data
//! @return A string that contains true or false depending on the data
inline std::string boolToString(bool data) {
	std::string ret;
	
	if (data) {
		ret = "true";
	} else {
		ret = "false";
	}
	
	return ret;
}

//! @brief Returns a string with a determinate anchor that represents the number
//! @param number The number to be represented
//! @param anchor The anchor of the string
//! @return The formatted string
std::string numberToString(int number, int anchor);

//! Returns a string with that represents the number
//! @param data The number
//! @return The string with the number
template<typename T> std::string numberToString(T data);

//! Returns a string with that represents the double number in fixed precision way
//! @param data The number
//! @param precision Number of decimal number to be shown
//! @return The string with the number
std::string numberToString(double data, int precision);
	
//! @brief Returns a representation of the vector
//! @param v The vector
//! @return A string that represents the items of the vector separated by spaces.
template<typename T> std::string printVector(const std::vector<T> &v);

//! @brief Returns a representation of the vector using toString method of the types
//! @param v The vector
//! @return A string that represents the items of the vector separated by spaces.
template<typename T> std::string printToStringVector(const std::vector<T> &v, bool end_line_separation = false);

//! @brief Returns a representation of the vector using toString method of the types
//! @param v The list
//! @return A string that represents the items of the list separated by endlines.
template<typename T> std::string printToStringList(const std::list<T> &v, bool end_line_separation = false);

//! @brief Returns a representation of the matrix
//! @param mat The matrix
//! @return A string that represents the items of the vector separated by spaces.
std::string printMatrix(const std::vector<std::vector <double> > &mat, const std::string &sep = "");


//! @brief Tabulates the string with the desired tabulation value
//! @param st The string to tabulate
//! @param n_tabs The number of tabulations
//! @return The tabulated string
std::string tabulate(const std::string &st, int n_tabs);

//! @brief Exports the content of a matrix to a string in MATLAB format
//! @param name Name of the matrix
//! @param mat The matrix
//! @return A string with the matrix
std::string matrixToMatlabString(const std::string &name, const std::vector<std::vector<double> > &mat);

template<typename T> std::string printVector(const std::vector<T> &v) {
	std::ostringstream os;
	for (unsigned int i = 0; v.size() > 0 && i < v.size() - 1; i++) {
		os << v[i] << " ";
	}
	
	if (v.size() > 0) {
		os << v.at(v.size() - 1);
	}
	
	return os.str();
}

template<typename T> std::string printToStringVector(const std::vector<T> &v, bool end_line_separation) {
	std::ostringstream os;
	for (unsigned int i = 0; v.size() > 0 && i < v.size() - 1; i++) {
		os << v[i].toString() << " ";
		if (end_line_separation) {
		  os << std::endl;
		}
	}
	
	if (v.size() > 0) {
		os << v.at(v.size() - 1).toString();
		if (end_line_separation) {
		  os << std::endl;
		}
	}
	
	return os.str();
}

template<typename T> std::string printToStringList(const std::list<T> &v, bool end_line_separation) {
	std::ostringstream os;
	typename std::list<T>::const_iterator it = v.begin();
	
	for (; it != v.end(); it++) {
		os << it->toString() << " ";
		if (end_line_separation) {
		  os << std::endl;
		}
	}
	
	return os.str();
}

	
	template<typename T> std::string vectorToMatlabString(const std::string &vec_name, const std::vector<T> vec) {
		std::ostringstream os;
		os << vec_name << " = [";
		
		for (unsigned int i = 0; i < vec.size(); i++) {
			os << vec[i] << " ";
		}
		
		os << "];";
		
		return os.str();
	}

template<typename T> std::string numberToString(T data) {
  std::ostringstream os;
  
  os << data;
  return os.str();
}

template<typename T> T sum(const std::vector<T> &v) {
  int i = v.size() - 1;
  T sum = 0;
  
  for (; i >= 0; i--) {
    sum += v.at(i);
  }
  
  return sum;
}

template<typename T> double mean(const std::vector<T> &v) {
  return ( (double)sum(v) / (double)v.size());
}

template <typename T> double variance(const std::vector<T> &v) {
  std::vector<T> quad(v);
  
  for (unsigned int i = 0; i < quad.size(); i++) {
    quad[i] *= quad[i];
  }
  double s = sum(v);
  double s2 = sum(quad);
  unsigned int n = v.size();
  
  return (double)(n * s2 - s * s) / (double) (n * (n - 1));
  
}

template <typename T> double std_dev(const std::vector<T> &v) {
  return sqrt(variance(v));
}

std::string removeSpaces(const std::string &st);

std::string loadStringFile(const std::string &filename);

bool writeStringToFile(const std::string &filename, const std::string &text);

double pathLength(const std::vector<std::vector<double> > &path);

double distance_(const std::vector<double> &v1, const std::vector<double> &v2); 
  
}

#endif //
