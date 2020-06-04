// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    std::cout << "There are " << cloud->points.size() << " points in the original point cloud.\n";

    // First, use a voxel grid.
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(filterRes, filterRes, filterRes);
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    // filter or applyFilter?
    voxel_grid.filter(*cloud_filtered);

    std::cout << "There are " << cloud_filtered->points.size() << " points in the voxel-filtered point cloud.\n";

    // Second, crop the region of interest.
    typename pcl::CropBox<PointT>::Ptr roi_box(new pcl::CropBox<PointT>);
    roi_box->setMin(minPoint);
    roi_box->setMax(maxPoint);
    roi_box->setInputCloud(cloud_filtered);
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered_and_cropped(new pcl::PointCloud<PointT>);
    roi_box->filter(*cloud_filtered_and_cropped);

    // As a final step, remove the points from the ego vehicle's roof
    typename pcl::CropBox<PointT>::Ptr roof_box(new pcl::CropBox<PointT>);
    roof_box->setMin(Eigen::Vector4f(-1.5f, -1.7f, -1.0f, 1.0f));
    roof_box->setMax(Eigen::Vector4f(3.0f, 1.7f, 0.0f, 1.0f));
    roof_box->setInputCloud(cloud_filtered_and_cropped);
    // Grab the indices of the roof points
    std::vector<int> roof_point_indices;
    roof_box->filter(roof_point_indices);

    std::cout << "There are " << cloud_filtered_and_cropped->points.size() << " points in the cropped point cloud.\n";

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered_and_cropped);
    // Unfortunately, the 'setIndices' method does not take a std::vector<int>, so we
    // need to convert it to a pcl::PointIndices::Ptr beforehand.
    pcl::PointIndices::Ptr pcl_roof_point_indices(new pcl::PointIndices);
    for (int idx : roof_point_indices) {
        pcl_roof_point_indices->indices.push_back(idx);
    }
    extract.setIndices(pcl_roof_point_indices);
    // Set negative to true in order to remove the selected indices (instead of keeping them).
    extract.setNegative(true);
    typename pcl::PointCloud<PointT>::Ptr cloud_final(new pcl::PointCloud<PointT>);
    extract.filter(*cloud_final);

    std::cout << "There are " << cloud_final->points.size() << " points in the final point cloud (ie., roof points are removed).\n";


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_final;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>);

    // We can fill the plane cloud (which are the inliers) with the following two lines
    // of code:
    // extract.setNegative(false);
    // extract.filter(*planeCloud);
    // However, just for the sake of experimenting, we can also do it by looping over
    // the indices
    for (int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }

    // Fill the outliers (which are the obstacles)
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    // The inliers will probably be the points which are on the road.
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    // Create an object for the model coefficients
    // (this can also be used to plot the plane later)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Optional according to the tutorial, but as we want to
    // fit the plane coefficients, we need to set this to true.
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Provide the input point cloud to the segmentation object
    seg.setInputCloud(cloud);
    // Now to the actual segmentation.
    // Note that we need to provide references, not pointers.
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // Plot the coefficients
    std::cout << "Plane coefficients: ";
    for (auto c : coefficients->values) {
        std::cout << c << ", ";
    }
    std::cout << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Plane coefficients
    linalg::Plane<double> plane;

    // Inlieres
    std::unordered_set<int> inliers_set = RansacPlane(&plane, cloud, maxIterations, distanceThreshold);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (auto index : inliers_set) {
        inliers->indices.push_back(index);
    }

    // Plot the coefficients
    std::cout << "Plane coefficients: ";
    std::cout << plane.a << ", " << plane.b << ", " << plane.c << ", " << plane.d;
    std::cout << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "my own plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(
                                    linalg::Plane<double> *ptr_plane,
                                    typename pcl::PointCloud<PointT>::Ptr cloud,
								    int maxIterations,
									float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	int point_count = cloud->points.size();
	std::cout << "Plane RANSAC: There are " << point_count << " points in the PC\n";

    linalg::Plane<double> plane;

	// For max iterations 
	while(maxIterations--) {
		//std::cout << "* Iterations left: " << maxIterations << std::endl;

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
		//std::cout << "* Chose indices: " << inliers_vec[0] << ", "
	//			 << inliers_vec[1] << ", " << inliers_vec[2] << std::endl;
		
		// Compute the plane parameters
		// First of all, grab the 3 points on the plane
		linalg::Vector3<double> p1(cloud->points[inliers_vec[0]].x, cloud->points[inliers_vec[0]].y, cloud->points[inliers_vec[0]].z);
		linalg::Vector3<double> p2(cloud->points[inliers_vec[1]].x, cloud->points[inliers_vec[1]].y, cloud->points[inliers_vec[1]].z);
		linalg::Vector3<double> p3(cloud->points[inliers_vec[2]].x, cloud->points[inliers_vec[2]].y, cloud->points[inliers_vec[2]].z);
		auto v1 = p2 - p1;
		auto v2 = p3 - p1;
		auto n = v1.cross(v2);
		n.normalize();

		
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
			//std::cout << "* The consensus set has now " << inliers.size() << " inliers.\n";
			inliersResult = inliers;
		}

	}

	//std::cout << "The final consensus set has " << inliersResult.size()
//			<< " inliers.\n";
    if (ptr_plane != nullptr) {
        *ptr_plane = plane;
    }


	// Return indicies of inliers from fitted line with most inliers
	// This is the consensus set
	return inliersResult;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // This is a vector of point clouds (in the form of point cloud pointers).
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles.
    // Just like with the segmentation task, we start by using the built-in PCL
    // functions.
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // We now have the indices of each cluster.
    std::cout << "There are " << cluster_indices.size() << " clusters.\n";

    // Based on the indices, create the point clouds for each cluster.
    for (pcl::PointIndices& indices_of_one_cluster : cluster_indices) {
        typename pcl::PointCloud<PointT>::Ptr cloud_of_one_cluster(new pcl::PointCloud<PointT>);
        for (int idx : indices_of_one_cluster.indices) {
            cloud_of_one_cluster->points.push_back(cloud->points[idx]);
        }
        cloud_of_one_cluster->width = cloud_of_one_cluster->points.size();
        cloud_of_one_cluster->height = 1;
        cloud_of_one_cluster->is_dense = true;
        clusters.push_back(cloud_of_one_cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
