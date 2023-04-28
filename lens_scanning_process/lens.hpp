#ifndef LENS_HPP_
#define LENS_HPP_

#include <string.h>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <thread>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PointIndices.h>
// #include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>


using namespace std;


int read_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename); // 读取数据

void segment_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, 
                    string save_path, string seg_method);   // 直通滤波

void contour_line_coor_find(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<float> &contour_x_coor); // 抽取所有轮廓的x坐标

void contour_line_circle_fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, 
                             vector<vector<float>> &coefficient);    // 轮廓圆数据拟合

int cloud_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b); // 点云可视化

void generate_circle(float x, float y, float z, float c_r, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);   // 生成圆形点云

void circleLeastFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<float> &coeff);   // 最小二乘圆拟合

void viewer_plot(const vector<vector<float>> &coefficient); // 二维曲线可视化

std::vector<double> mean_filter(const std::vector<double>& data, int window_size);  // 一维均值滤波

#endif