#include "functions/functions.h"
#include "functions/RealVector.h"
#include <math.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>
#include <boost/lexical_cast.hpp>

#ifndef M_PI
#define M_PI 3.14159265
#endif

#define MAX_BUFFER_SIZE_FUNCTIONS__  655536

using namespace std;

namespace functions{

std::string tabulate(const string& st, int n_tabs)
{
  ostringstream os;
  
  for (int i = 0; i < n_tabs;i ++) {
    os << "\t";
  }
  
  os << st;
  
  return os.str();
}

double reduceAngle(double phi)
{
  while ( fabs(phi) > M_PI) {
    phi = phi + ( phi > 0.0 ? -1.0 : 1.0 ) * 2.0 * M_PI;
  }
  
  return phi;
}

void rotateVector(std::vector<double> &v, double angle) {
  if (v.size() >= 2) {
    double x, y;
    
    cout << "Angle: " << angle << endl;
    
    x = v.at(0)*cos(angle) - v.at(1)*sin(angle);
    y = v.at(0)*sin(angle) + v.at(1)*cos(angle);
    v[0] = x;
    v[1] = y;
  }
}

void rotateVector(std::vector<double> &v, const DegMinSec &angle) {
  rotateVector(v, angle.toRadians());
}
	
std::vector<double>  getVectorFromFile(const std::string &fileName) throw (){

	vector <double> ret;
	
	ifstream filestr;
	try{
		filestr.open( fileName.c_str() );
		filestr.exceptions(ifstream::failbit | ifstream::badbit);
		ret = getVectorFromStream(filestr);
	}
	catch (...){
// 		cout << "Error flags: " << filestr.rdstate() << endl;
// 		ret.clear();
	}
	
	return ret;
}

std::vector< double > getVectorFromStream(istream& is)
{
  vector<double> ret;
  try {
    while ( is.good() && !is.eof() ){
      double au;
      if (is >> au) {
	ret.push_back(au);
      }
    }
  }catch (exception &e) {
      
  }
  return ret;
}

std::vector< double > getVectorFromString(const string& s)
{
  vector<double> ret;
  string ss = removeSpaces(s);
  char *copy_ = new char[ss.length() + 1];
  strcpy(copy_, ss.c_str());
  char *tok = copy_;
  tok = strtok(tok, " \n\t\r");
  try {
    while ( tok != NULL && strlen(tok) > 0 ) {
      ret.push_back(boost::lexical_cast<double>(tok));
      tok = strtok(NULL, " \n\t\r");
    }  
  } catch (exception &e) {
    ret.clear();
  }
  delete[] copy_;
  return ret;
}

bool getMatrixFromFile(const std::string &fileName, int width, std::vector<std::vector<double> > &v) throw (){
	bool ret = true;
	
	if (v.size() > 0) 
	  v.clear();
	
	ifstream filestr;
	try{
		
		double aux;
		int i;
		filestr.open( fileName.c_str() );
		filestr.exceptions(ifstream::failbit | ifstream::badbit);
		char buff[MAX_BUFFER_SIZE_FUNCTIONS__];
		filestr.getline(buff, MAX_BUFFER_SIZE_FUNCTIONS__ - 1); // Discard the first line
		while ( filestr.good() ){
			vector<double> v_aux;
			
			bool valid = false;
			
			for (i = 0; i < width && filestr.good(); i++) {
			  filestr >> aux;
			  v_aux.push_back(aux);
				if (aux != 0.0) {
					valid = true;
				}
			}
			
			if (i == width && i > 0) {
			  if (valid) {
			    // Discard the zero vectors
			    v.push_back(v_aux);
			  }
			} else {
			  ret = false;
			}
		}
	}
	catch (...){
		cout << "Error flags: " << filestr.rdstate() << endl;
		if  (!filestr.eof()){
			throw;
		}
	}
	
	return ret;
}

bool getMatrixFromFile(const std::string &fileName, std::vector<std::vector<double> > &v, bool discard_first) throw (){
	bool ret = true;
	
	if (v.size() > 0) 
	  v.clear();
	
	ifstream filestr;
	char *buff = new char[MAX_BUFFER_SIZE_FUNCTIONS__ + 1];
	try{
		
		filestr.open( fileName.c_str() );
		
		if (filestr.is_open()) {
// 		  filestr.exceptions(ifstream::failbit | ifstream::badbit);
		  if (discard_first) {
			  filestr.getline(buff, MAX_BUFFER_SIZE_FUNCTIONS__ - 1); // Discard the first line
		  }
		  while ( !filestr.eof() && filestr.good() ){
			  vector<double> v_aux;
			  filestr.getline(buff, MAX_BUFFER_SIZE_FUNCTIONS__ - 1); // Get the current line
			  if (strlen(buff) > 0) {
			    string s(buff);
			    v_aux = getVectorFromString(s);
			    if (v_aux.size() > 0) {
				  v.push_back(v_aux);
			    }
			  }
		  }
		  filestr.close();
		} else {
		  ret = false;
		}
	}
	catch (exception &e){
		cerr << "Error flags: " << filestr.rdstate() << endl;
		cerr << "Exception catched while loading file " << fileName << " from disk. Content: " << e.what() << endl;
		if  (!filestr.eof()){
		  delete buff;
		  buff = NULL;
			throw e;
		}
		filestr.close();
	}
	delete[] buff;
	
	return ret;
}


std::string showTime(struct timeval t1, struct timeval t2){
	long usecs = (t2.tv_sec -t1.tv_sec)*1000.0 + (t2.tv_usec -t1.tv_usec)/1000.0;
	std::ostringstream os;
	os <<  ": "<<  usecs << "ms" << endl;	
	
	return os.str();
}

float calculateLapseTime(const struct timeval &t1, const struct timeval &t2) {
	return (t2.tv_sec - t1.tv_sec) + (t2.tv_usec - t1.tv_usec)/1e6;
}
	
string numberToString(int number, int anchor) {
	ostringstream oss;
	
	oss.str("");
  oss.fill('0');
  oss.width(anchor);
  oss << right << number; 
  return oss.str();
}

string numberToString(double data, int precision) {
  ostringstream oss;
  oss << fixed << setprecision(precision) << data;
  return oss.str();
}

string printMatrix(const std::vector< std::vector< double > >& mat, const std::string &sep)
{
	ostringstream os;
	
	for (unsigned int row = 0; row < mat.size(); row++) {
		os << printVector(mat[row]) << sep << endl; 
	}
	
	return os.str();
}

string printMatrix(const std::vector< RealVector >& mat, const std::string &sep)
{
	ostringstream os;
	
	for (unsigned int row = 0; row < mat.size(); row++) {
		os << printVector(mat[row]) << sep << endl; 
	}
	
	return os.str();
}

string matrixToMatlabString(const std::string& name, const std::vector< std::vector< double > >& mat)
{
	ostringstream os;
	
	os << name << " = [" << printMatrix(mat, ";") << "];" << endl;
	
	return os.str();
}

string matrixToMatlabString(const std::string& name, const std::vector< functions::RealVector>& mat)
{
	ostringstream os;
	
	os << name << " = [" << printMatrix(mat, ";") << "];" << endl;
	
	return os.str();
}

string removeSpaces(const std::string &st)
{
  string ret;
  if (st.size() == 0) {
    return st;
  }
  unsigned int first, last;
  for (first = 0; first < st.length() && (st.at(first)==' ' || st.at(first)=='\r' || st.at(first)=='\t') ; first++) {
  }
  for (last = st.length() ; last >= 1 && (st.at(last - 1)==' ' || st.at(last - 1)=='\r' || st.at(last - 1)=='\t') ; last--) {
  }
  if (first >= 0 && first < st.length() && last >= 0 && last <= st.length() && first < last) {
    string ret_2(st, first, last);
    ret = ret_2;
  }
  
  return ret;
}

bool writeStringToFile(const string& filename, const std::string &text)
{
  bool ret_val = true;
  
  ofstream ofs;
  
  ofs.open(filename.c_str());
  if (ofs.is_open()) {
    ofs << text;
  } else {
    ret_val = false;
  }
  ofs.close();
  
  return ret_val;
}

double pathLength(const vector< vector< double > >& path)
{
  double length = 0.0;
  
  for (unsigned int i = 0; i < path.size() - 1 && path.size() > 1; i++) {
    length += distance_(path[i], path[i + 1]);
  }
  
  return length;
}

double distance_(const std::vector<double> &v1, const std::vector<double> &v2)
{
  unsigned int length = minimum(v1.size(), v2.size());
  double distance = 0.0;
  
  for (unsigned int i = 0; i < length; i++) {
    distance += pow(v1.at(i) - v2.at(i), 2.0);
  }
  
  return sqrt(distance);
}

string loadStringFile(const string& filename)
{
  string ret;
  
  ifstream filestr;
  char *buff = new char[MAX_BUFFER_SIZE_FUNCTIONS__ + 1];
  try {
    filestr.open( filename.c_str() );
    if (filestr.is_open()) {
//       filestr.exceptions(ifstream::failbit | ifstream::badbit);
      while ( !filestr.eof() && filestr.good() ) {
	filestr.getline(buff, MAX_BUFFER_SIZE_FUNCTIONS__ - 1); // Get the current line
	string s(buff);
	s.append("\n");
	if (s.size() > 0) {
	  ret.append(s);
	}
      }
      filestr.close();
    }
  }
  catch (exception &e){
    cerr << "Error flags: " << filestr.rdstate() << endl;
    cerr << "Exception catched while loading file " << filename << " from disk. Content: " << e.what() << endl;
    if  (!filestr.eof()){
      delete[] buff;
      buff = NULL;
      throw e;
    }
    filestr.close();
  }
  delete[] buff;
  
  return ret;
}


} // End of namespace functions
