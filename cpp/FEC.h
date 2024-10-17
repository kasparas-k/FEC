#pragma once
#pragma warning(disable:4996)

#include <ctime>
#include <iostream>
#include <omp.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>

using namespace std;

/**
* Store index and label information for each point
*/
struct PointIndex_NumberTag
{
    float nPointIndex;
    float nNumberTag;
};

bool NumberTag(const PointIndex_NumberTag& p0, const PointIndex_NumberTag& p1);
std::vector<pcl::PointIndices> FEC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int min_component_size, double tolerance, int max_n);