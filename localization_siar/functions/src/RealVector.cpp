#include "RealVector.h"
#include "functions.h"
#include "RandomNumberGenerator.h"

using namespace std;

namespace functions {

RealVector::RealVector(){
	init();
}

RealVector::RealVector(const std::vector<double>& vec)
{
	init(vec);
}

RealVector::RealVector(const double* vec, size_t size) :vector< double, std::allocator <double> >(vec, vec + size)
{
  
}


RealVector::RealVector(const std::string& st)
{
  fromString(st);
}

RealVector::RealVector(const functions::Point3D& p): vector< double, std::allocator< double > >()
{
  init(p);
}

RealVector::RealVector(const RealVector& v):std::vector<double>(v)
{
  init(v);
}

RealVector::RealVector(int dim, bool random): vector<double>(dim, 0.0) {
  init(dim, random);
}

void RealVector::init(const functions::Point3D& p)
{
  clear();
  push_back(p.x);
  push_back(p.y);
  push_back(p.z);
}


void RealVector::fromString(const std::string& s)
{
  clear();
  
  string clean = removeSpaces(s);
//   std::cout << "Clean: " << clean << endl;
  string no_parenthesis = clean;
  if (clean.length() > 2 && clean.at(0) == '(' && clean.at(clean.length() - 1) == ')') {
    string aux(clean, 1, clean.length() - 2);
    no_parenthesis = aux;
  }
//     std::cout << "No parenthesis: " << no_parenthesis << endl;
  int last_find = -1;
  double aux;
     
  int i = no_parenthesis.find(',');
  
  for (;i < (int)no_parenthesis.length(); i= no_parenthesis.find(',', i + 1) ) {
    string curr(no_parenthesis, last_find + 1, i - last_find);
    istringstream is(curr);
    is >> aux;
    push_back(aux);
    last_find = i;
  }
  // Get last token
  string la(no_parenthesis, last_find + 1, no_parenthesis.length());
  istringstream is2(la);
  is2 >> aux;
  push_back(aux);
  
}

void RealVector::init(){
	clear();
}

void RealVector::init(const RealVector& v)
{
  init(v.size());
  
  for (unsigned int i = 0; i < v.size(); i++) {
    at(i) = v.at(i);
  }
}


void RealVector::init(const std::vector< double >& vec)
{
  init(vec.size());
  
  for (unsigned int i = 0; i < vec.size(); i++) {
    at(i) = vec.at(i);
  }
}

void RealVector::init(int dim, bool random)
{
  if ((int)size() != dim) {
    this->resize(dim, 0.0);
  }
  
  RandomNumberGenerator ran;
  for (unsigned int i = 0; random && i < size(); i++) {
      this->at(i) = ran.rnd01();
  }
}


RealVector &RealVector::operator = (const RealVector &that){
	if (this == &that) {
		return *this;
}
	init(); 
	init(that); 
	return *this;
}

std::string RealVector::toString() const {
	using std::endl;
	std::ostringstream os;
	
	if (size() > 0) {
		os << "(";
	
		for (unsigned int i = 0; i < size() - 1; i++) {
			os << at(i) << ", ";
		}
		os << at(size() - 1) << ")";
	}

	return os.str();
}

std::string RealVector::toMatlabString() const {
	using std::endl;
	std::ostringstream os;
	
	if (size() > 0) {
		for (unsigned int i = 0; i < size(); i++) {
			os << at(i) << " ";
		}
	}

	return os.str();
}


const RealVector operator+(const RealVector &a,const RealVector &b) {
	RealVector ret;
	
	if ( a.size() != b.size() ) {
		throw("Different vector size");
	} else {
		for (unsigned int i = 0; i < a.size(); i++) {
			ret.push_back(a.at(i) + b.at(i));
		}
	}
	
	return ret;
}

const RealVector operator-(const RealVector &a,const RealVector &b) {
	RealVector ret;
	
	if ( a.size() != b.size() ) {
		throw("Different vector size");
	} else {
		for (unsigned int i = 0; i < a.size(); i++) {
			ret.push_back(a.at(i) - b.at(i));
		}
	}
	
	return ret;
}

const RealVector operator*(const RealVector &a,const double &b) {
	RealVector ret;
	
	
	for (unsigned int i = 0; i < a.size(); i++) {
		ret.push_back(a.at(i) * b);
	}
	
	return ret;
}

double operator*(const RealVector &a,const RealVector &b) {
	double ret = 0.0;
	
	if ( a.size() != b.size() ) {
		throw("Different vector size");
	} else {
		for (unsigned int i = 0; i < a.size(); i++) {
			ret += a.at(i) * b.at(i);
		}
	}
	
	return ret;
}

const RealVector operator^(const RealVector &a,const double &b) {
	RealVector ret;
	
	for (unsigned int i = 0; i < a.size(); i++) {
		ret.push_back( pow(a.at(i), b));
	}

	return ret;
}

const RealVector operator/(const RealVector &a,const double &b) {
	RealVector ret;
	
	for (unsigned int i = 0; i < a.size(); i++) {
		ret.push_back( a.at(i) /  b);
	}

	return ret;
}

const RealVector operator-(const RealVector& a) {
	RealVector ret;
	
	for (unsigned int i = 0; i < a.size(); i++) {
		ret.push_back( -a.at(i));
	}

	return ret;
}

double RealVector::distance(const RealVector &p)const {
	double ret = 0.0;
	unsigned int max_index = functions::minimum(p.size(), size());
	
	for (unsigned int i = 0; i < max_index; i++) {
		ret += pow(p.at(i) - at(i), 2.0);
	}
	
	return sqrt(ret);
}
double RealVector::norm()const {
	return sqrt((*this)*(*this));
}

RealVector RealVector::crossProduct(const RealVector &p)const {
	RealVector ret;
	
	if ( this->size() == p.size() && p.size() == 3) {
		ret.push_back( this->at(1) * p.at(2) - this->at(2) * p.at(1) );
		ret.push_back( this->at(2) * p.at(0) - this->at(0) * p.at(2) );
		ret.push_back( this->at(0) * p.at(1) - this->at(1) * p.at(0) );
	} else {
		throw("RealVector::crossProduct. Error: dimension of the vectors are not 3");
	}
	
	return ret;
}

RealVector RealVector::componentProduct(const RealVector &p)const {
	RealVector ret(size());

	if (this->size() == p.size()) {
	  for (unsigned int i = 0; i < size(); i++) {
	    ret.at(i) = at(i) * p.at(i);
	  }
	} else {
		throw("RealVector::component product. Different vector size detected.\n");
	}

	return ret;
}


double RealVector::distanceToSegment(const functions::RealVector& s1, const functions::RealVector& s2) const
{
  // As indicated in http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/
  // Calculate the u value.
  // This value will be used to calculate the projection point (P') of this point to the line that passes through s1 and s2
  // P = P1 + u (P2 - P1)
  
  double u = (*this - s1) * (s2 - s1) / ((s2 - s1) * (s2 - s1));
  
  if (u > 0 && u < 1) {
    // The projection point is inside the segment -> so the distance to the segment is the distance to the projection
    return distance(s1 + (s2 - s1) * u);
  } else {
    // If the projection point does not belong to the segment
    // The distance to the segment is the minimum distance to the vertices
    return minimum(distance(s1), distance(s2));
  }
  
  RealVector aux; 
}

double RealVector::angle(const functions::RealVector& other) const
{
  
  double cosin = ((*this) * (other)) / (norm()*other.norm());
  
  cosin = min(1.0,cosin);
  cosin = max(-1.0,cosin);
  
  return acos(cosin);
}

bool RealVector::operator ==(const RealVector &other) const {
  bool ret = size() == other.size();

  for (unsigned int i = 0; i < size() && ret; i++) {
    ret = at(i) == other.at(i);
  }

  return ret;
}

double pathLength(const vector< RealVector >& path)
{
  double length = 0.0;
  
  for (unsigned int i = 0; i < path.size() - 1 && path.size() > 1; i++) {
    length += path[i].distance(path[i + 1]);
  }
  
  return length;
}

}
