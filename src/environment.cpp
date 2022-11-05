/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene) {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    auto *lidar = new Lidar(cars, 0.0);
    // TODO:: Create point processor
    auto point_cloud = lidar->scan();
//    renderRays(viewer, lidar->position, point_cloud);
    renderPointCloud(viewer, point_cloud, "point_cloud");
//    point_cloud->points;
    ProcessPointClouds<pcl::PointXYZ> processPointClouds;
//    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processPointClouds.SegmentPlane(
//            point_cloud, 100, 0.2);
//    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
//    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

//    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processPointClouds.Clustering(segmentCloud.first,
//                                                                                                   1.0, 3, 30);
//
//    int clusterId = 0;
//    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(0, 1, 1)};
//
//    for (pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster: cloudClusters) {
//        std::cout << "cluster size ";
//        processPointClouds.numPoints(cluster);
//        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);
//        Box box = processPointClouds.BoundingBox(cluster);
//        renderBox(viewer, box, clusterId);
//        ++clusterId;
//    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *point_processor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud) {
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud;
    filtered_cloud = point_processor->FilterCloud(input_cloud, 0.5f, Eigen::Vector4f(-15.0, -8.0, -3.0, 1),
                                                  Eigen::Vector4f(15.0, 8.0, 3.0, 1));

    auto segmented_cloud = point_processor->SegmentPlane(
            filtered_cloud, 100, 0.55);

    renderPointCloud(viewer, segmented_cloud.second, "planeCloud", Color(0, 1, 0));

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    auto clustered_cloud = point_processor->Clustering(segmented_cloud.first,
                                                       0.6, 3, 100);
    for (auto &cluster: clustered_cloud) {
        std::cout << "cluster size ";
        point_processor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        Box box = point_processor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle) {
        case XY :
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown :
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side :
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS :
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}


int main(int argc, char **argv) {
    std::cout << "starting enviroment" << std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    auto *point_processor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<std::__fs::filesystem::path> stream = point_processor->streamPcd("../src/sensors/data/pcd/data_1");
    auto stream_iterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;
    while (!viewer->wasStopped()) {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        input_cloud = point_processor->loadPcd((*stream_iterator).string());
        cityBlock(viewer, point_processor, input_cloud);
        stream_iterator++;
        if (stream_iterator == stream.end())
            stream_iterator = stream.begin();
        viewer->spinOnce(100);
    }
}