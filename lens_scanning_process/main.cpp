#include "lens.hpp"


int main()
{
    // 读取原始点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // string file_name = "/home/wanyel/contours/lens_scanning/20230505/2023_05_05_14_50_55_877.pcd";
    // read_cloud(cloud, file_name);

    // 点云直通滤波
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
    // string save_seg = "/home/wanyel/contours/lens_scanning/20230505/2023_05_05_14_50_55_877_zSeg.pcd";
    // segment_cloud(cloud, cloud_seg, save_seg, "z_seg");
    // read_cloud(cloud_seg, save_seg);

    // 逐轮廓中值滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_median(new pcl::PointCloud<pcl::PointXYZ>);
    string save_median = "/home/wanyel/contours/lens_scanning/20230505/2023_05_05_14_50_55_877_median.pcd";
    read_cloud(cloud_median, save_median);
    // contour_line_median_filter(cloud_seg, cloud_median);

    // 找出所有轮廓线的X坐标
    // vector<float> contour_line_coor;
    // contour_line_coor_find(cloud_seg, contour_line_coor);

    // 对所有轮廓进行圆拟合
    // pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    // vector<vector<float>> coefficients;
    // contour_line_circle_fit(cloud_seg, all_cloud_ptr, coefficients);

    // 分段圆柱拟合
    vector<float> segment_interval = {0, 12.0, 13.5, 22.5, 49.5, 61.5, 65.5, 69.0, 91.5, 95.0, 104.5, 118.0, 130.0};  // 根据x坐标分段
    vector<vector<int>> seg_idx;
    segment_index_get(cloud_median, segment_interval, seg_idx);

    cylinder_fit(cloud_median, seg_idx);

    // 可视化
    // cloud_viewer(cloud_seg, all_cloud_ptr);
    // viewer_plot(coefficients);
}