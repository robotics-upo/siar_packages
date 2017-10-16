#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/search/search.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/segment_differences.h>




#include "siar_map_analysis.h"
#include "processPointCloud.h"

SIAR_map_analysis::SIAR_map_analysis() : 
	cloudIn(new pcl::PointCloud<pcl::PointXYZ>)
{

	currentType = -1;

	threshold_min = 0.005;
	threshold_max = 0.04;

	global_align_th=0.1;

	//Create sections

	SIAR_section nt_120a;

	generateNT120A(nt_120a.model,0.05,2.0,0.05,1.00); //0.02, 3.0, 0.05, 1.40

	nt_120a.type = "NT_120A";

	sections.push_back(nt_120a);
	
	SIAR_section t133;

	generateT133(t133.model,0.05,2.0,0.05,1.00);

	t133.type = "T133";

	sections.push_back(t133);

	SIAR_section t164;
  	generateT164(t164.model,0.05,2.0,0.05,1.00);
	t164.type = "T164";

	sections.push_back(t164);

	SIAR_section t181;
  	generateT181(t181.model,0.05,2.0,0.05,1.00);
	t181.type = "T181";

	sections.push_back(t181);

	SIAR_section d1400;
  	generateD1400(d1400.model,0.05,2.0,0.05,1.00);
	d1400.type = "D1400";

	sections.push_back(d1400);

}


void SIAR_map_analysis::setInputCloud(pcl::PointCloud<pcl::PointXYZ> & cloud)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAux (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAux2 (new pcl::PointCloud<pcl::PointXYZ>);
   
   	cloudAux = cloud.makeShared();

	//We should  cut the points far away in the z axis, as the resolution is low
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloudAux);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0,5.0);
	pass.filter(*cloudAux2);
   	
	//Downsample input cloud
   	pcl::VoxelGrid<pcl::PointXYZ> sor;
  
   	sor.setInputCloud(cloudAux2);
   	sor.setLeafSize (0.05, 0.05, 0.05);
   	sor.filter(*cloudIn); 

	

}

std::string SIAR_map_analysis::estimateSectionType(void)
{

	//Initialize ICP
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;	
	icp.setMaximumIterations (100);
  	icp.setTransformationEpsilon (1e-9);
  	icp.setMaxCorrespondenceDistance (0.2);
  	icp.setEuclideanFitnessEpsilon (1);
  	icp.setRANSACOutlierRejectionThreshold (0.002);


   	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZ>);
   	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut_new (new pcl::PointCloud<pcl::PointXYZ>) ;


	std::string type = "None";
	float min_align = 100000.0;
	float aux_align;
	currentType = -1;

	//Input ICP cloud
	icp.setInputTarget(cloudIn);

	//Now loop over section types
  	for(int i=0; i<sections.size();i++)
	{
		cloudOut = sections[i].model.makeShared(); //This to be moved inside SIAR_section

		icp.setInputSource(cloudOut); //This to be moved inside SIAR_section
  		
		icp.align(*cloudOut_new);
		if (icp.hasConverged())
		{
		      	std::cout << "ICP converged." << std::endl
				<< "The score is " << icp.getFitnessScore() << std::endl;

			aux_align = icp.getFitnessScore(); 
			
			//Determine section with lowest fitness score
			if(aux_align < min_align && aux_align < global_align_th)
			{
				min_align = icp.getFitnessScore();
				type = sections[i].type; 
				currentType = i;
			}
			
			//Update section model to the new transformation
			sections[i].model = *cloudOut_new;
		}
		else std::cout << "ICP did not converge." << std::endl;
 	}

	return type;

}


//Detect defects: difference segmentation between point cloud and section type
int SIAR_map_analysis::computeDefects(pcl::PointCloud<pcl::PointXYZ> & finalCloud)
{
	pcl::SegmentDifferences<pcl::PointXYZ> p;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut_new (new pcl::PointCloud<pcl::PointXYZ>);

	if(currentType >= 0)
	{

		cloudOut_new = sections[currentType].model.makeShared();

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); 

   		p.setInputCloud(cloudOut_new); //We segment over the model cloud
   		p.setTargetCloud(cloudIn);

   		p.setSearchMethod(tree); 

   		p.setDistanceThreshold(threshold_min); //Threshold as squared distance! 0.005
   		
		p.segment(finalCloud);
   	
	}

	return currentType;



}


int SIAR_map_analysis::computeDefectsCustom(pcl::PointCloud<pcl::PointXYZ> & output)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut_new (new pcl::PointCloud<pcl::PointXYZ>);

	

	if(currentType >= 0)
	{

		cloudOut_new = sections[currentType].model.makeShared();

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); 

		tree->setInputCloud (cloudIn);

		 // We're interested in a single nearest neighbor only
  		std::vector<int> nn_indices (1);
		std::vector<float> nn_distances (1);
    		// The src indices that do not have a neighbor in tgt
      		std::vector<int> src_indices;
   
  		// Iterate through the source data set
	        for (int i = 0; i < static_cast<int> ((*cloudOut_new).points.size ()); ++i)
	   	{
			if (!isFinite ((*cloudOut_new).points[i]))
        			continue;
  			// Search for the closest point in the target data set (number of neighbors to find = 1)
  			if (!tree->nearestKSearch ((*cloudOut_new).points[i], 1, nn_indices, nn_distances))
  			{
				continue;
   			}
   
   			if (nn_distances[0] > threshold_min && nn_distances[0] < threshold_max)
   	    			src_indices.push_back (i);
   		}
  
		// Allocate enough space and copy the basics
		output.points.resize (src_indices.size ());
		output.header   = (*cloudOut_new).header;
		output.width    = static_cast<uint32_t> (src_indices.size ());
		output.height   = 1;

		output.is_dense = true;

		typedef typename pcl::traits::fieldList<pcl::PointXYZ>::type FieldList;
		// Iterate over each point
		for (size_t i = 0; i < src_indices.size (); ++i)
		// Iterate over each dimension
			pcl::for_each_type <FieldList> (pcl::NdConcatenateFunctor <pcl::PointXYZ, pcl::PointXYZ> ((*cloudOut_new).points[src_indices[i]], output.points[i]));


   	
	}

	return currentType;



}





