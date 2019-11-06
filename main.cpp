#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "LMicp.h"
int main() {
	//整个地图
	std::string _whole_map = "/home/echo/log/map.pcd";
	//std::string _whole_map = "/home/echo/CLionProjects/LMICP/data/map/project.pcd";
	

	//当前扫描的点云
	std::string cloud_frame = "/home/echo/CLionProjects/LMICP/data/curr/moved.pcd";
	//设置初始的位姿
	Eigen::Isometry3d raw_pose = Eigen::Isometry3d::Identity();
	pcl::PointCloud<pcl::PointXYZI>target_cloud;
	pcl::PointCloud<pcl::PointXYZI> source;
	pcl::PointCloud<pcl::PointXYZI>::Ptr fast_map(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::io::loadPCDFile<pcl::PointXYZI>(_whole_map, *fast_map);
	pcl::io::loadPCDFile<pcl::PointXYZI>(cloud_frame, source);


	LMicp lm,a;
	a.timeCalcSet("lm");
	lm.solveICP(fast_map,source,raw_pose);
	a.timeUsed();
	return 0;
}