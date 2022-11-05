//
// Created by Alexey on 30/10/2022.
//
#include "ransac.h"

template<typename PointT>
float hw::ransac::get_point_plane_dist(PointT point1, PointT point2, PointT point3, PointT point) {
    float a = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
    float b = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
    float c = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
    float d = -(a * point1.x + b * point1.y + c * point1.z);
    return std::abs(a * point.x +
                    b * point.y + c * point.z + d) / std::pow((a * a + b * b + c * c), 0.5);
};

template<typename PointT>
std::unordered_set<int>
hw::ransac::segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliers_result;
    std::srand(std::time(nullptr));
    size_t idx1, idx2, idx3;
    PointT point1, point2, point3;
    bool found_points = false;
    size_t matches, max_matches = 0;
    std::unordered_set<int> aux;
    for (int i = 0; i < maxIterations; i++) {
        found_points = false;
        while (!found_points) {
            idx1 = std::rand() % cloud->size();
            while ((idx2 = std::rand() % cloud->size()) == idx1) {}
            while ((idx3 = std::rand() % cloud->size()) == idx1 || (idx3 == idx2)) {}
            point1 = cloud->points[idx1];
            point2 = cloud->points[idx2];
            point3 = cloud->points[idx3];
            if (!hw::ransac::collinear(point1, point2, point3)) {
                found_points = true;
            }
        }
        matches = 0;
        aux.clear();
        for (int j = 0; j < cloud->size(); j++) {
            auto point = cloud->points[j];
            auto dist = hw::ransac::get_point_plane_dist(point1, point2, point3, point);
            if (dist <= distanceTol) {
                ++matches;
                aux.insert(j);
            }
        }
        if (matches > max_matches) {
            max_matches = matches;
            inliers_result = aux;
        }
    }
    return inliers_result;
}

template<typename PointT>
bool hw::ransac::collinear(PointT point1, PointT point2, PointT point3) {
    if (point3.x == point1.x) {
        if ((point3.y == point1.y) && (point3.z = point1.z)) {
            return true;
        } else {
            if (point2.x == point1.x) {
                float area = point1.y * (point2.z - point3.z) + point2.y * (point3.z - point1.z) +
                             point3.y * (point1.z - point2.z);
                return area == 0;
            }
        }
    } else {
        float coef = (point2.x - point1.x) * 1.0f / (point3.x - point1.x);
        if (((point2.y - point1.y) == coef * (point3.y - point1.y)) &&
            ((point2.z - point1.z) == coef * (point3.z - point1.z))) {
            return true;
        }
    }
    return false;
}
