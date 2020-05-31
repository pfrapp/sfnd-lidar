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

	// Any random number
	double r = 0.0;
	// Maximum random number (as double)
	const double r_max = (double) RAND_MAX;

	int point_count = cloud->points.size();
	std::cout << "There are " << point_count << " points in the PC (should be 20)\n";

	double *distances = new double[point_count];
	
	// TODO: Fill in this function

	// For max iterations 
	for (int iter = 0; iter < maxIterations; iter++) {
		std::cout << "* Iteration " << (iter+1) << "/" << maxIterations << std::endl;

		// Randomly sample subset and fit line
		r = rand();
		int idx_1 = (int) (r / r_max * point_count);
		int idx_2;
		do {
			r = rand();
			idx_2 = (int) (r / r_max * point_count);
		} while (idx_1 == idx_2);
		std::cout << "* Chose indices " << idx_1 << " and " << idx_2 << std::endl;
		
		// Compute the line parameters
		double x1 = cloud->points[idx_1].x;
		double y1 = cloud->points[idx_1].y;
		double x2 = cloud->points[idx_2].x;
		double y2 = cloud->points[idx_2].y;
		double A = y1 - y2;
		double B = x2 - x1;
		double C = x1*y2 - x2*y1;
		// Normalize
		double line_norm = sqrt(A*A+B*B);
		A /= line_norm;
		B /= line_norm;
		C /= line_norm;

		// Measure distance between every point and fitted line
		for (int ii=0; ii<point_count; ii++) {
			distances[ii] = A*cloud->points[ii].x + B*cloud->points[ii].y + C;
			distances[ii] = fabs(distances[ii]);
		}

		// If distance is smaller than threshold count it as inlier
		std::unordered_set<int> inliers_this_iteration;
		for (int ii=0; ii<point_count; ii++) {
			if (distances[ii] < distanceTol) {
				inliers_this_iteration.insert(ii);
			}
		}

		// Check if this is our new consensus set
		if (inliers_this_iteration.size() > inliersResult.size()) {
			std::cout << "* The consensus set has now " << inliers_this_iteration.size() << " inliers.\n";
			inliersResult = inliers_this_iteration;
		}

	}

	
	delete[] distances;

	// Return indicies of inliers from fitted line with most inliers
	// This is the consensus set
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);

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
