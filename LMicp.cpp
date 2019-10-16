//
// Created by echo on 2019/10/14.
//

#include "LMicp.h"
//source 是当前扫描到的点 target 是地图的点
Eigen::Isometry3d LMicp::solveICP(pcl::PointCloud<pcl::PointXYZI> target, pcl::PointCloud<pcl::PointXYZI> source,
								  Eigen::Isometry3d init_pose) {
	//1.转换点云
	pcl::PointCloud<pcl::PointXYZI> tfed_source;
	pcl::transformPointCloud(tfed_source, source, init_pose.matrix());
	//2.计算最近的点云
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr target_ptr(new pcl::PointCloud<pcl::PointXYZI>);
	*target_ptr = target;
	kdtreeLast->setInputCloud(target_ptr);
	pcl::PointXYZI pointSel; // 要搜索的点
	std::vector<int> pointSearchInd;//搜索最近点的id
	std::vector<float> pointSearchSqDis;//最近点的距离
	for (int i = 0; i < source.size(); ++i) {
		pointSel = source[i];
		kdtreeLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
	}

	//3.构建约束
	//4.LM优化
	return Eigen::Isometry3d();
}
