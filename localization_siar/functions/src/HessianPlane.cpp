#include "HessianPlane.h"

using namespace std;

namespace functions {

HessianPlane::HessianPlane(const Point3D &pv, const double &pp){
	init(pv,pp);
	
}

HessianPlane::HessianPlane(const functions::Point3D& a, const functions::Point3D& b)
{
	init(a, b);
}


HessianPlane::~HessianPlane(){
	dispose();
}

void HessianPlane::init(){
	planeVec = NULL;
	p = 0.0;
}

void HessianPlane::init(const Point3D &pv, const double &pp){
	planeVec = new Point3D(pv);
	p = pp;
}

void HessianPlane::init(const functions::Point3D& a, const functions::Point3D& b)
{
	double d,p;
  Point3D nv( b - a );
  Point3D pv(nv.x,nv.y,nv.z);
	
  d= -nv * b;
  
  p= d / nv.norm ();  
  pv.normalize();
	
  init(pv, p);
}


void HessianPlane::dispose () throw () {
	delete planeVec;
	init();
}
		
HessianPlane::HessianPlane(const HessianPlane &that )
{
	init(*(that.planeVec),that.p);
};

HessianPlane &HessianPlane::operator = (const HessianPlane &that){
	if (this == &that) {
		return *this;
}
	dispose(); 
	init(*(that.planeVec),that.p); 
	return *this;
}

HessianPlane *HessianPlane::clone () const { 
	if (planeVec == NULL) {
		return new HessianPlane();
	} else {
		return new HessianPlane (*planeVec, p);
	}
}


std::string HessianPlane::toString() {
	using std::endl;
	std::ostringstream os;
	os << planeVec->toString() <<" | " << p ;
	
	
	return os.str();
}

}

