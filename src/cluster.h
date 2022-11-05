//
// Created by Alexey on 02/11/2022.
//
#ifndef PLAYBACK_CLUSTER_H
#define PLAYBACK_CLUSTER_H

#include <vector>
#include <iostream>
#include <cmath>
#include "search.h"

namespace hw {
    namespace cluster {
        template<typename PointT>
        void proximity(int point_id, const std::vector<PointT> &points, std::vector<int> &cluster,
                       std::vector<char> &visited, hw::search::KdTree<PointT> *tree, float distanceTol);

        template<typename PointT>
        std::vector<std::vector<int>>
        cluster(std::vector<PointT> &points, hw::search::KdTree<PointT> *tree, float distanceTol);
    }
}
#endif //PLAYBACK_CLUSTER_H
