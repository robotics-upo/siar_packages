
#include <vector>

#include "siar_section.h"


class SIAR_map_analysis
{

public:
	SIAR_map_analysis();

	//!Set the input cloud to be analyzed
	void setInputCloud(pcl::PointCloud<pcl::PointXYZ> & cloud);


	//!Estimate the section type from the list of potential sections
	std::string estimateSectionType(void);
	
	//!Compute defects once a section type has been determined (thus, to be called after estimateSectionType)
	int computeDefects(pcl::PointCloud<pcl::PointXYZ> & finalCloud);

	int computeDefectsCustom(pcl::PointCloud<pcl::PointXYZ> & finalCloud);

	void setMinThresholdDefects(float t){threshold_min=t;};

	void setMaxThresholdDefects(float t){threshold_max=t;};;

	void setGlobalAlignThreshold(float t){global_align_th=t;};

private:

	//!Current type (-1 if not determined)
	int currentType;

	//!Pointer to current point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn;

	//!List of potential section types
	std::vector<SIAR_section> sections;

	float threshold_min;
	float threshold_max;

	float global_align_th;

};

