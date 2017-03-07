#include "functions/RandomNumberGenerator.h"

#ifndef _TAU 
#define _TAU  6.28318530717958647692528676655900576839433879875021
#endif


namespace functions {
 

RandomNumberGenerator *RandomNumberGenerator::instance = NULL;  
  
RandomNumberGenerator::RandomNumberGenerator()
{
  boost::posix_time::ptime p;
  gen = new boost::mt19937;
  // Set the seed to the system time
  gen->seed(static_cast<unsigned int>(boost::posix_time::microsec_clock::universal_time().time_of_day().total_milliseconds() ) );
  distribution = new boost::uniform_01<boost::mt19937>(*gen);
}

RandomNumberGenerator::~RandomNumberGenerator()
{
  dispose();
}

double RandomNumberGenerator::getRandomNumber(double low, double high)
{
  if (instance == NULL) {
    instance = new RandomNumberGenerator;
  }
  double ret = instance->rnd01();
  
  if (high != 1.0 || low != 0.0) {
    ret = ret * (high - low) + low;
  }
    
  return ret;
}

double RandomNumberGenerator::randomGaussian(double sigma, double mean)
{
  double x1,x2;
  x1 = getRandomNumber();
  x2 = getRandomNumber();

  double z;
  z = sqrt(-2.0 * log(x1))*cos(_TAU * x2);

  return z * sigma + mean;
}


double RandomNumberGenerator::rnd01()
{
  return (*distribution)();
}

void RandomNumberGenerator::dispose()
{
  delete gen;
  delete distribution;
  pointersToNULL();
}

void RandomNumberGenerator::pointersToNULL()
{
  gen = NULL;
  distribution = NULL;
}


}