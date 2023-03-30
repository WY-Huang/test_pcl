#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

//构造平面点云  
void creat_plane_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr)
{
    // x: 14-82 y: -20-20 z:83.0

    for (float x_p = 14.1; x_p <= 81.7; x_p += 0.1)
    {
        for (float y_p = -20.0; y_p <= 19.6; y_p += 0.1)
        {
            pcl::PointXYZ basic_point;

            basic_point.x = x_p;
            basic_point.y = y_p;
            basic_point.z = 82.2;

            basic_cloud_ptr->points.push_back(basic_point);

        }

        basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
        basic_cloud_ptr->height = 1;
    }
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    creat_plane_pointcloud(cloud);

    pcl::io::savePCDFile ("/home/wanyel/vs_code/test_pcl/imgs/raw_pcd/2022_09_26_08_41_08_601_plane.pcd", *cloud);

    // 点云读取及合并
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile("/home/wanyel/vs_code/test_pcl/imgs/raw_pcd/2022_09_26_08_41_08_601_mod1.pcd", *cloud_a) == -1)
    {
        cout << "点云数据读取失败！" << endl;
    }

    *cloud_final = (*cloud) + (*cloud_a);
    pcl::io::savePCDFile ("/home/wanyel/vs_code/test_pcl/imgs/raw_pcd/2022_09_26_08_41_08_601_final.pcd", *cloud_final);
}