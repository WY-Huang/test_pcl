#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <thread>
#include <pcl/io/ply_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/plane_clipper3D.h>

using namespace std;

class Ground_segment
{
public:
    explicit Ground_segment(string filename);
    ~Ground_segment();

    string filename;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

    pcl::ModelCoefficients::Ptr coefficients;   // 创建分割时所需要的模型系数对象coefficients
    pcl::PointIndices::Ptr inliers;             // 存储内点的点索引集合对象inliers

    pcl::PCDWriter writer;

    int read_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename);
    void ground_fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients,
                    pcl::PointIndices::Ptr inliers, float thick);
    int cloud_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b);
};