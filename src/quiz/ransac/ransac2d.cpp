/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	int point_count = cloud->points.size();
	std::cout << "There are " << point_count << " points in the PC (should be 20)\n";

	// For max iterations 
	while(maxIterations--) {
		std::cout << "* Iterations left: " << maxIterations << std::endl;

		// Randomly sample subset and fit line
		std::unordered_set<int> inliers;
		while (inliers.size() < 2) {
			int r = rand();
			r = r % point_count;
			// Elements in the unordered set are guaranteed to be unique
			inliers.insert(r);
		}
		// Create a vector easy access
		std::vector<int> inliers_vec(inliers.begin(), inliers.end());
		std::cout << "* Chose indices " << inliers_vec[0] << " and " << inliers_vec[1] << std::endl;
		
		// Compute the line parameters
		double x1 = cloud->points[inliers_vec[0]].x;
		double y1 = cloud->points[inliers_vec[0]].y;
		double x2 = cloud->points[inliers_vec[1]].x;
		double y2 = cloud->points[inliers_vec[1]].y;
		double A = y1 - y2;
		double B = x2 - x1;
		double C = x1*y2 - x2*y1;
		// Normalize
		double line_norm = sqrt(A*A+B*B);
		A /= line_norm;
		B /= line_norm;
		C /= line_norm;
		// The line is now fit and ready to use

		// Measure distance between every point and fitted line
		for (int ii=0; ii<point_count; ii++) {
			// Check if the point was one of the two points to
			// create the line. If so, continue.
			if (inliers.count(ii) > 0) {
				continue;
			}

			double distance = A*cloud->points[ii].x + B*cloud->points[ii].y + C;
			distance = fabs(distance);

			// If distance is smaller than threshold count it as inlier
			if (distance <= distanceTol) {
				inliers.insert(ii);
			}
		}

		// Check if this is our new consensus set
		if (inliers.size() > inliersResult.size()) {
			std::cout << "* The consensus set has now " << inliers.size() << " inliers.\n";
			inliersResult = inliers;
		}

	}

	std::cout << "The final consensus set has " << inliersResult.size()
			<< " inliers.\n";


	// Return indicies of inliers from fitted line with most inliers
	// This is the consensus set
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
								    int maxIterations,
									float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	int point_count = cloud->points.size();
	std::cout << "Plane RANSAC: There are " << point_count << " points in the PC\n";

	// For max iterations 
	while(maxIterations--) {
		std::cout << "* Iterations left: " << maxIterations << std::endl;

		// Randomly sample subset and fit plane
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			int r = rand();
			r = r % point_count;
			// Elements in the unordered set are guaranteed to be unique
			inliers.insert(r);
		}
		// Create a vector easy access
		std::vector<int> inliers_vec(inliers.begin(), inliers.end());
		std::cout << "* Chose indices: " << inliers_vec[0] << ", "
				 << inliers_vec[1] << ", " << inliers_vec[2] << std::endl;
		
		// Compute the plane parameters
		// First of all, grab the 3 points on the plane
		linalg::Vector3<double> p1(cloud->points[inliers_vec[0]].x, cloud->points[inliers_vec[0]].y, cloud->points[inliers_vec[0]].z);
		linalg::Vector3<double> p2(cloud->points[inliers_vec[1]].x, cloud->points[inliers_vec[1]].y, cloud->points[inliers_vec[1]].z);
		linalg::Vector3<double> p3(cloud->points[inliers_vec[2]].x, cloud->points[inliers_vec[2]].y, cloud->points[inliers_vec[2]].z);
		auto v1 = p2 - p1;
		auto v2 = p3 - p1;
		auto n = v1.cross(v2);
		n.normalize();

		linalg::Plane<double> plane;
		plane.a = n.x;
		plane.b = n.y;
		plane.c = n.z;
		plane.d = -1.0 * (n.dot(p1));
		// The plane is now fit and ready to use

		// Measure distance between every point and the fitted plane
		for (int ii=0; ii<point_count; ii++) {
			// Check if the point was one of the three points to
			// create the plane. If so, continue.
			if (inliers.count(ii) > 0) {
				continue;
			}

			linalg::Vector3<double> pt(cloud->points[ii].x, cloud->points[ii].y, cloud->points[ii].z);
			double distance = plane.distance(pt);

			// If distance is smaller than threshold count it as inlier
			if (distance <= distanceTol) {
				inliers.insert(ii);
			}
		}

		// Check if this is our new consensus set
		if (inliers.size() > inliersResult.size()) {
			std::cout << "* The consensus set has now " << inliers.size() << " inliers.\n";
			inliersResult = inliers;
		}

	}

	std::cout << "The final consensus set has " << inliersResult.size()
			<< " inliers.\n";


	// Return indicies of inliers from fitted line with most inliers
	// This is the consensus set
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
