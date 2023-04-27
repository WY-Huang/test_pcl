#include "lens.hpp"


int main()
{
    // 读取原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    string file_name = "/home/wanyel/contours/lens_scanning/2023_04_27_10_17_02_849.pcd";
    read_cloud(cloud, file_name);

    // 点云直通滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
    string save_seg = "/home/wanyel/contours/lens_scanning/2023_04_27_10_17_02_849_zSeg.pcd";
    segment_cloud(cloud, cloud_seg, save_seg, "z_seg");

    // 找出所有轮廓线的X坐标
    vector<float> contour_line_coor;
    contour_line_coor_find(cloud_seg, contour_line_coor);

    // 对所有轮廓进行圆拟合
    pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    contour_line_circle_fit(cloud_seg, all_cloud_ptr);

    // 可视化
    cloud_viewer(cloud_seg, all_cloud_ptr);
}