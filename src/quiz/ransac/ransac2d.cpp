/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <Eigen/Dense>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++) {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }
    // Add outliers
    int numOutliers = 10;
    while (numOutliers--) {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);

    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
    viewer->addCoordinateSystem(1.0);
    return viewer;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    size_t idx1, idx2, idx3;
    auto get_dist = [](pcl::PointXYZ point1, pcl::PointXYZ point2, pcl::PointXYZ point3, pcl::PointXYZ point) {
        float a = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
        float b = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
        float c = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
        float d = -(a * point1.x + b * point1.y + c * point1.z);
        return std::abs(a * point.x +
                        b * point.y + c * point.z + d) / std::pow((a * a + b * b + c * c), 0.5);
    };

    size_t matches, max_matches = 0;
    std::unordered_set<int> aux;
    for (int i = 0; i < maxIterations; i++) {
        idx1 = std::rand() % cloud->size();
        while ((idx2 = std::rand() % cloud->size()) == idx1) {}
        while ((idx3 = std::rand() % cloud->size()) == idx1 || (idx3 == idx2)) {}
        auto point1 = cloud->points[idx1];
        auto point2 = cloud->points[idx2];
        auto point3 = cloud->points[idx3];
        matches = 0;
        aux.clear();
        for (int j = 0; j < cloud->size(); j++) {
            auto point = cloud->points[j];
            auto dist = get_dist(point1, point2, point3, point);
            if (dist <= distanceTol) {
                ++matches;
                aux.insert(j);
            }
        }
        if (matches > max_matches) {
            max_matches = matches;
            inliersResult = aux;
        }
    }
    return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    size_t idx1, idx2;
    auto get_dist = [](pcl::PointXYZ point1, pcl::PointXYZ point2, pcl::PointXYZ point) {
        float a = (point1.y - point2.y);
        float b = (point2.x - point1.x);
        return std::abs(a * point.x +
                        b * point.y + (point1.x * point2.y - point2.x * point1.y)) / std::pow((a * a + b * b), 0.5);
    };

    size_t matches, max_matches = 0;
    std::unordered_set<int> aux;
    for (int i = 0; i < maxIterations; i++) {
        idx1 = std::rand() % cloud->size();
        while ((idx2 = std::rand() % cloud->size()) == idx1) {}
        auto point1 = cloud->points[idx1];
        auto point2 = cloud->points[idx2];
        matches = 0;
        aux.clear();
        for (int j = 0; j < cloud->size(); j++) {
            auto point = cloud->points[j];
            auto dist = get_dist(point1, point2, point);
            if (dist <= distanceTol) {
                ++matches;
                aux.insert(j);
            }
        }
        if (matches > max_matches) {
            max_matches = matches;
            inliersResult = aux;
        }
    }
    return inliersResult;
}

int main() {

    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

    // Create data
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac3D(cloud, 100, 0.25);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    // Render 2D point cloud with inliers and outliers
    if (inliers.size()) {
        renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
        renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
    } else {
        renderPointCloud(viewer, cloud, "data");
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

}
