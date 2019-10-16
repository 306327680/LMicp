//
// Created by echo on 2019/10/14.
//
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <chrono>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#ifndef LMICP_LMICP_H
#define LMICP_LMICP_H


class LMicp {
public:
	LMicp(){};
	Eigen::Isometry3d solveICP(pcl::PointCloud<pcl::PointXYZI> target, pcl::PointCloud<pcl::PointXYZI> source,
							   Eigen::Isometry3d init_pose) ;
	void timeUsed() {
		auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>
				(std::chrono::high_resolution_clock::now() - _now_ms);
		std::cout<<name_global<<" time is :"<< duration.count()/1e9<<std::endl;
		
	}
	
	void timeCalcSet(std::string name) {
		_now_ms = std::chrono::high_resolution_clock ::now();
		name_global = name;
		
	}

private:
	std::string name_global;
	std::chrono::high_resolution_clock::time_point _now_ms;
	
};


#endif //LMICP_LMICP_H
