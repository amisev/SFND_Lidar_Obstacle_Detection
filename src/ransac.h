//
// Created by Alexey on 30/10/2022.
//

#ifndef PLAYBACK_RANSAC_H
#define PLAYBACK_RANSAC_H

#include <unordered_set>
#include <iostream>
#include <pcl/common/common.h>

namespace ransac {
    template<typename PointT>
    std::unordered_set<int> segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    template<typename PointT>
    float get_point_plane_dist(PointT point1, PointT point2, PointT point3, PointT point);

    template<typename PointT>
    bool collinear(PointT point1, PointT point2, PointT point3);
}
#endif //PLAYBACK_RANSAC_H
