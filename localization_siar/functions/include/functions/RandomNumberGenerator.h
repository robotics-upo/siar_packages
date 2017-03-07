#ifndef __RANDOM_NUMBER_GENERATOR_H__
#define __RANDOM_NUMBER_GENERATOR_H__

#include <boost/random.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

/** @brief Random number generator

To get a random number between 0 and 1:
<pre>
double random = RandomNumberGenerator::rnd01();</pre>
*/

namespace functions {

class RandomNumberGenerator
{
  
  private:
    //!Generator pointer (mersenne twister)
    boost::mt19937 *gen;
    //!Uniform Distribution [0..1] pointer
    boost::uniform_01<boost::mt19937> *distribution;
    
    
    RandomNumberGenerator(RandomNumberGenerator const&);              // Don't want copy
    void operator=(RandomNumberGenerator const&); // Don't want assign
    
  public:
        /** 
	 * @brief Default constructor, recomended.
	*/
    RandomNumberGenerator();

    
    //!Destructor. 
    ~RandomNumberGenerator();
    

    //!gets a pointor to the generator
    boost::mt19937 *getGen(){
      return gen;
    }
    //!Gets a random number between 0 and 1. Samples a uniform distribution.
    double rnd01();
    
    inline double rnd(double low = 0.0, double high = 1.0) { return rnd01()*(high - low) + low;}
    
    static double getRandomNumber(double low = 0.0, double high = 1.0);
    
    double randomGaussian(double sigma, double mean);
      
    void dispose();
    
    //! @brief Makes all internal pointers point to NULL
    void pointersToNULL();
    
    static RandomNumberGenerator *instance;
    
};



}

#endif //__RANDOM_NUMBER_GENERATOR_H__
