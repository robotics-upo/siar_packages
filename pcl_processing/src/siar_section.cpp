
#include "siar_section.h"


SIAR_section::SIAR_section()
{
	initICP();


}


void SIAR_section::initICP()
{

	icp.setMaximumIterations (100);
  	icp.setTransformationEpsilon (1e-9);
  	icp.setMaxCorrespondenceDistance (0.2);
  	icp.setEuclideanFitnessEpsilon (1);
  	icp.setRANSACOutlierRejectionThreshold (0.002);



}
