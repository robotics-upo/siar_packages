

#include <sensor_msgs/PointCloud2.h>
#include <pcl/registration/icp.h>

typedef enum {NT_120A,T133,T164} SIAR_section_type;

class SIAR_section
{

public:
	//SIAR_section_type type;
	std::string type;

	//!Point cloud model
	pcl::PointCloud<pcl::PointXYZ> model;

	//!Local ICP object (not in use now)
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	SIAR_section();

private:
	void initICP();

};
