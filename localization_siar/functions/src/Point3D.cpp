#include "Point3D.h"

using namespace std;

namespace functions {

Point3D::Point3D(){
	init();
}

Point3D::Point3D(const double &px, const double &py, const double &pz){
	init(px,py,pz);
	
}

Point3D::Point3D(const std::vector<double>& vec)
{
	init(vec);
}


Point3D::~Point3D(){
	dispose();
}

void Point3D::init(){
	x=0.0;
	y=0.0;
	z=0.0;
}

void Point3D::init(const double &px, const double &py, const double &pz){
	x=px;
	y=py;
	z=pz;
}

void Point3D::init(const std::vector< double >& vec)
{
	init();
	try {
		x = vec.at(0);
		y = vec.at(1);
		if (vec.size() > 2) {
		  z = vec.at(2);
		} else {
		  z = 0.0;
		}
	} catch (std::exception &e) {
		
	}
}


void Point3D::dispose () throw () {
// 	init();
}
		
Point3D::Point3D(const Point3D &that )
{
	init(that.x,that.y,that.z);
};

Point3D &Point3D::operator = (const Point3D &that){
	if (this == &that) {
		return *this;
}
	dispose(); 
	init(that.x,that.y,that.z); 
	return *this;
}

Point3D *Point3D::clone () const { 
	return new Point3D (x,y,z);
}


std::string Point3D::toString(bool format) const {
	using std::endl;
	std::ostringstream os;
	if (format) {
	  os << "(" << x << ", " << y << ", " << z << ")";
	} else {
	  os << x << " " << y << " " << z;
	}
	
	return os.str();
}

const Point3D operator+(const Point3D &a,const Point3D &b) {
	return Point3D(a.x + b.x, a.y + b.y, a.z + b.z);
}

const Point3D operator-(const Point3D &a,const Point3D &b) {
	return Point3D(a.x - b.x, a.y - b.y, a.z - b.z);
}

const Point3D operator*(const Point3D &a,const double &b) {
	return Point3D( b * a.x, b * a.y, b * a.z );
}

double operator*(const Point3D &a,const Point3D &b) {
	return a.x * b.x  +  a.y * b.y  +   a.z * b.z;
}

const Point3D operator^(const Point3D &a,const double &b) {
	return Point3D( pow(a.x, b) , pow( a.y, b), pow( a.z, b) );
}

const Point3D operator/(const Point3D &a,const double &b) {
	return Point3D( a.x/b , a.y/b, a.z/b );
}

const Point3D operator-(const Point3D& a) {
	return Point3D(- a.x, -a.y, -a.z);
}

double Point3D::distance(const Point3D &p)const {
	return sqrt( ( p.x - x ) * ( p.x - x ) + 
			( p.y - y ) * ( p.y - y ) + 
			( p.z - z ) * ( p.z - z ) );
	
}

double Point3D::distance2d(const Point3D& p) const
{
  return sqrt( ( p.x - x ) * ( p.x - x ) + ( p.y - y ) * ( p.y - y ));
}


double Point3D::norm()const {
	return sqrt( x*x + y*y + z*z );
	
}

Point3D Point3D::crossProduct(const Point3D &p)const {
	Point3D ret;
	ret.x = y * p.z - z * p.y ;
	ret.y = z * p.x - x * p.z ;
	ret.z = x * p.y - y * p.x ;
	return ret;
}

}
