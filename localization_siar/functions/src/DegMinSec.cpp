#include "DegMinSec.h"

using namespace std;

#include<math.h>



namespace functions {

	#ifndef PI
	#define PI 3.1415926535897932384626433832795028841971693993
	#endif
	
DegMinSec::DegMinSec(){
	init();
}

DegMinSec::DegMinSec(const int &d, const int &m, const double &s){
	init(d,m,s);
	
}

DegMinSec::~DegMinSec(){
	dispose();
}

void DegMinSec::init(){
	deg=0;
	min=0;
	sec=0.0;
}

void DegMinSec::init(const int &d, const int &m, const double &s){
	deg=d;
	min=m;
	sec=s;
}

void DegMinSec::dispose () throw () {
	init();
}
		
DegMinSec::DegMinSec(const DegMinSec &that )
{
	init(that.deg,that.min,that.sec);
};

DegMinSec &DegMinSec::operator = (const DegMinSec &that){
	if (this == &that) {
		return *this;
}
	dispose(); 
	init(that.deg,that.min,that.sec); 
	return *this;
}

DegMinSec *DegMinSec::clone () const { 
	return new DegMinSec (deg,min,sec);
}


std::string DegMinSec::toString() {
	using std::endl;
	std::ostringstream os;
	os << deg <<"º" << min << "'" << sec << "''";
	return os.str();
}

std::string DegMinSec::toStringDecMin() {
	using std::endl;
	std::ostringstream os;
	os << deg << "º" << min + sec / 60.0 << "'";
	return os.str();
}

double DegMinSec::toDecimalDeg() const{
	double aux=fabs(deg) + min/60.0 + sec/3600.0;
	return deg>0?aux:-aux;
	
}  

double DegMinSec::toRadians() const {
	return toDecimalDeg() / 180.0 * PI;
}  

DegMinSec::DegMinSec(const double &d, EntryMode e){
	double d1 = d;
	
	if ( e == RADIANS ) {
		d1 *= 180.0 / PI;
	}
	
	double aux = fabs(d1);
	deg = (int)floor(aux);
	deg = d1 < 0 ? - deg : deg;
	aux -= fabs((float)deg);
	min = (int)floor(aux * 60.0);
	sec = (aux - min / 60.0) * 3600;
}
}