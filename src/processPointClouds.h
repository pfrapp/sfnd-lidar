// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"

namespace linalg {

template<typename T>
class Vector3 {
    public:
    union {
        struct { T x, y, z; };
        T data[3];
    };

    Vector3(T x, T y, T z) : x(x), y(y), z(z) {}

    Vector3<T> operator+(const Vector3& v_other) const {
        Vector3<T> v_result(this->x + v_other.x, this->y + v_other.y, this->z + v_other.z);
        return v_result;
    }

    Vector3<T> operator-(const Vector3& v_other) const {
        Vector3<T> v_result(this->x - v_other.x, this->y - v_other.y, this->z - v_other.z);
        return v_result;
    }

    T dot(const Vector3& v) const {
        T inner_product = 0.0;
        for (int ii=0; ii<3; ii++) {
            inner_product += this->data[ii] * v.data[ii];
        }
        return inner_product;
    }

    //! Compute the 2-norm
    T norm() const {
        T norm_squared = this->dot(*this);
        return std::sqrt(norm_squared);
    }

    //! Normalize without check for division by zero
    void normalize() {
        T n = this->norm();
        for (int ii=0; ii<3; ii++) {
            this->data[ii] /= n;
        }
    }

    //! Cross product
    Vector3<T> cross(const Vector3& v) const {
        Vector3<T> outer_product(
            this->y * v.z - this->z * v.y,
            this->z * v.x - this->x * v.z,
            this->x * v.y - this->y * v.x
        );
        return outer_product;
    }

};

template<typename T>
std::ostream& operator<<(std::ostream& out, const Vector3<T> v) {
    out << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return out;
}

void test() {
    Vector3<double> A(1.0, 2.0, 3.0);
    Vector3<double> B(4.0, 5.0, 6.0);

    auto v_add = A+B;
    auto v_minus = A-B;
    std::cout << A << " + " << B << " = " << v_add << std::endl;
    std::cout << A << " - " << B << " = " << v_minus << std::endl;
    std::cout << "norm(A) = " << A.norm() << std::endl;
    std::cout << "A.B = " << A.dot(B) << std::endl;
    auto v_cross = A.cross(B);
    std::cout << "A x B = " << v_cross << std::endl;
    B.normalize();
    std::cout << "B has now been normalized\n";
    std::cout << "B = " << B << std::endl;
}

} // namespace linalg

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */