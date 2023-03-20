#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main ()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/wanyel/contours/20221130/2022_11_30_15_57_38_531.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    int count = 0;
    float value = 0;
    for (const auto& point: *cloud)
    {   
        if (point.x != value)
        {
            value = point.x;
            count ++;
        }
        // std::cout << " " << point.x
        //           << " " << point.y
        //           << " " << point.z << std::endl;
        
    }
    std::cout << "轮廓数量：" << count << std::endl;
    return 0;
}