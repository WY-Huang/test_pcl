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

using namespace std;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    // 读入点云PCD文件
    reader.read("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_sample_100_filter.pcd", *cloud);

    // string filename = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_sample_100_filter.ply";
    // pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud);

    // pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZ>("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_sample_100_filter.pcd", *cloud, false);
    cout<< "Point cloud data: " << cloud->points.size() << " points" << endl;

    // 创建分割时所需要的模型系数对象coefficients及存储内点的点索引集合对象inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必须配置，设置分割的模型类型、所用随机参数估计方法
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold(0.01);// 距离阈值 单位m。距离阈值决定了点被认为是局内点时必须满足的条件,距离阈值表示点到估计模型的距离最大值

    // float angle = 10;                     
    // float EpsAngle= pcl::deg2rad(angle);   // 角度转弧度
    // Eigen::Vector3f Axis(0.0, 0.0, 1.0); 

    // seg.setAxis(Axis);                     // 指定的轴
    // seg.setEpsAngle(EpsAngle);             // 夹角阈值(弧度制)

    seg.setInputCloud(cloud);//输入点云
    seg.segment(*inliers, *coefficients);//实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (-1);
    }
    //***********************************************************************
    //-----------输出平面模型的系数 a,b,c,d-----------
    cout << "Model coefficients: " << coefficients->values[0] << " "
        << coefficients->values[1] << " "
        << coefficients->values[2] << " "
        << coefficients->values[3] << endl;
    cout << "Model inliers: " << inliers->indices.size() << endl;
 
    //***********************************************************************
    // 提取地面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*cloud_filtered);

    cout << "Ground cloud after filtering: " << endl;
    cout<< *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("/home/wanyel/contours/20220926/pointCloud_20220913092246086_mod_sample_100_ground.pcd", *cloud_filtered, false);

    // 提取除地面外的物体
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    cout << "Object cloud after filtering: " << endl;
    cout << *cloud_filtered << endl;

    writer.write<pcl::PointXYZ>("/home/wanyel/contours/20220926/pointCloud_20220913092246086_mod_sample_100_object.pcd", *cloud_filtered, false);

    // 点云可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
    //左边窗口显示输入的点云,右边的窗口显示分割后的点云
    int v1(0), v2(0);
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_in, "cloud_in", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_in", v1);

    viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered, "cloud_out", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "cloud_out", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_out", v2);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        // boost::this_thread::sleep(boost::posix_time::microseconds(1000));
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    return (0);
}

