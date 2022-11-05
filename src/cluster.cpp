//
// Created by Alexey on 04/11/2022.
//
#include "cluster.h"

template<typename PointT>
void hw::cluster::proximity(int point_id, const std::vector<PointT> &points, std::vector<int> &cluster,
               std::vector<char> &visited, hw::search::KdTree<PointT> *tree, float distanceTol) {
    visited[point_id] = true;
    cluster.push_back(point_id);
    auto nearby_points = tree->search(points[point_id], distanceTol);
    for (auto np: nearby_points) {
        if (!visited[np]) {
            hw::cluster::proximity(np, points, cluster, visited, tree, distanceTol);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>>
hw::cluster::cluster(std::vector<PointT> &points, hw::search::KdTree<PointT> *tree, float distanceTol) {
    std::vector<std::vector<int>> clusters;
    std::vector<char> visited(points.size(), false);
    for (int point_id = 0; point_id < points.size(); point_id++) {
        if (!visited[point_id]) {
            std::vector<int> cluster;
            hw::cluster::proximity(point_id, points, cluster, visited, tree, distanceTol);
            clusters.push_back(cluster);
        }
    }
    return clusters;
}
