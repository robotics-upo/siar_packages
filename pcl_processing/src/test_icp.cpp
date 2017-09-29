#include "pointmatcher/PointMatcher.h"
#include <cassert>
#include <iostream>
#include <fstream>

using namespace std;
using namespace PointMatcherSupport;

typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

int main(int argc, char *argv[]) {
	if (argc != 3) {
		std::cerr << "Error: invalid number of arguments" << std::endl;
	}

	PM::TransformationParameters T;
	T = PM::TransformationParameters::Identity(4,4);
	// Applying a translation in the x direction
	T(0,3) = 50;

	std::cout << "Transformation Matrix: " << std::endl << T << std::endl;

	PM::Transformation* rigidTrans;
	rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

	if (!rigidTrans->checkParameters(T)) {
		std::cout << "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis"
				<< std::endl;
		T = rigidTrans->correctParameters(T);
	}

	// Load a point cloud from a file
	PM::DataPoints pointCloud;
	std::string inputFile = argv[1];
	pointCloud = PM::DataPoints::load(inputFile);

	// Compute the transformation
	PM::DataPoints outputCloud =  rigidTrans->compute(pointCloud,T);

	outputCloud.save(argv[2]);

	std::cout << "Transformed cloud saved to " << argv[2] << std::endl;
	return 0;
}

