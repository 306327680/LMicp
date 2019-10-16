#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "LMicp.h"
int main() {
	//整个地图
	std::string _whole_map = "/home/echo/CLionProjects/LMICP/data/pcd/1551690896.751801000.pcd";
	//当前扫描的点云
	std::string cloud_frame = "/home/echo/CLionProjects/LMICP/data/map/project.pcd";
	//设置初始的位姿
	Eigen::Isometry3d raw_pose = Eigen::Isometry3d::Identity();
	pcl::PointCloud<pcl::PointXYZI>target_cloud;
	pcl::PointCloud<pcl::PointXYZI> source;
	pcl::io::loadPCDFile<pcl::PointXYZI>(_whole_map, target_cloud);
	pcl::io::loadPCDFile<pcl::PointXYZI>(cloud_frame, source);
	LMicp lm;
	lm.timeCalcSet("lm");
	lm.solveICP(target_cloud,source,raw_pose);
	lm.timeUsed();
	return 0;
}