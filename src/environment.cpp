/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    double slope = 0.0;
    Lidar *lidar = new Lidar(cars, slope);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> *pointCloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = 
	pointCloudProcessor->Segment(inputCloud, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    // Do the clustering of the obstacle cloud.
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointCloudProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterID = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 1, 1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        std::cout << "Cluster size: ";
        pointCloudProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstacleCloud"+std::to_string(clusterID), colors[clusterID%colors.size()]);

        // Render the bounding boxes
        Box bbox = pointCloudProcessor->BoundingBox(cluster);
        float opacity = 0.3;
        renderBox(viewer, bbox, clusterID, colors[clusterID%colors.size()], opacity);

        ++clusterID;
    }

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // ------Lesson 4: Working with Real PCD         ------
    // ------Open 3D viewer and display a city block ------
    // ----------------------------------------------------

    // Create a point cloud processor which also considers the intensity I.
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    // Load the point cloud data.
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    // Show the point cloud.
    // renderPointCloud(viewer,inputCloud,"inputCloud");

    // Filter the cloud
    float filter_resolution = 0.5;
    Eigen::Vector4f min_point(-10.0, -10.0, -3.0, 1.0);
    Eigen::Vector4f max_point(30.0, 10.0, 3.0, 1.0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filter_resolution, min_point, max_point);
    // Show the filtered point cloud.
    renderPointCloud(viewer,filteredCloud,"filteredCloud");

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}
