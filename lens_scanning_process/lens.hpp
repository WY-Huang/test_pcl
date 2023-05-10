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

#include <pcl/features/normal_3d.h>	
#include <pcl/filters/statistical_outlier_removal.h>  // 统计滤波
#include <pcl/filters/convolution_3d.h> // 高斯滤波
#include <pcl/filters/median_filter.h> // 中值滤波

using namespace std;


int read_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename); // 读取数据

void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, string save_path);   // 保存点云

void segment_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, 
                    string save_path, string seg_method);   // 直通滤波

void contour_line_coor_find(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<float> &contour_x_coor); // 抽取所有轮廓的x坐标

void contour_line_circle_fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, 
                             vector<vector<float>> &coefficient);    // 轮廓圆数据拟合

void segment_index_get(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<float> &segment_interval_in, 
                        vector<vector<int>> &seg_idx_out); // 分段并获得分段索引

void cylinder_fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<vector<int>> &seg_idx_in, 
                    vector<float> &cyfit_d, const vector<float> &real_d);    // 圆柱拟合

int cloud_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b); // 点云可视化

void generate_circle(float x, float y, float z, float c_r, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);   // 生成圆形点云

void circleLeastFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<float> &coeff);   // 最小二乘圆拟合

void viewer_plot(const vector<vector<float>> &coefficient); // 二维曲线可视化

std::vector<double> mean_filter(const std::vector<double>& data, int window_size);  // 一维均值滤波

void statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);   // 统计滤波

void gaussian_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);   // 高斯滤波

void contour_line_median_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_ptr);   // 对所有轮廓进行圆拟合

void cylinder_fit_err(vector<float> &real_d, vector<float> &fit_d); // 计算直径拟合误差


#endif