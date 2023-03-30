#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/filters/extract_indices.h>
// #include <pcl/common/io.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/visualization/cloud_viewer.h>

using namespace std;


int read_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename) { 
	// 读取ply文件
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1) 
    {
		cout << "Could not read file." << endl;
		return -1;
	}
	cout << "Load file success -> " << filename << endl;
	cout << "The number of raw point clouds: " << cloud->points.size() << endl;
	return 0;
}


void segment_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, 
                    string save_path="")
{
    // 根据 xyz 坐标进行阈值分割
    vector<int> pointIdxVec;

    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        float x0 = cloud_in->points[i].x;
        float y0 = cloud_in->points[i].y;
        float z0 = cloud_in->points[i].z;

        if (x0 > -12 && x0 < 29 && y0 > 18 && z0 > -61.5)   //  分割阈值
        {
            pointIdxVec.push_back(i);
        }
    }
    pcl::copyPointCloud(*cloud_in, pointIdxVec, *cloud_out);

    if (save_path.empty() == false)
    {
        pcl::PLYWriter writer;
        
        writer.write(save_path, *cloud_out, false, false);
        cout << "Save segment file success -> " << save_path << endl;
    }
    cout << "The number of segment point clouds: " << cloud_out->points.size() << endl;
}


void fast_uniform_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                        string save_path="")
{
	// 实现从点云中每every_k_points个点采样一次
	int every_k_points = 15;
	pcl::Indices indices;
	for (size_t i = 0; i < cloud_in->size(); i += every_k_points)
	{
		indices.push_back(i);
	}
	pcl::copyPointCloud(*cloud_in, indices, *cloud_out);

    if (save_path.empty() == false)
    {
        pcl::PLYWriter writer;
        
        writer.write(save_path, *cloud_out, false, false);
        cout << "Save sample file success -> " << save_path << endl;
    }
    cout << "The number of sample point clouds: " << cloud_out->points.size() << endl;
}


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
            basic_point.z = -61.5;

            basic_cloud_ptr->points.push_back(basic_point);

        }

        basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
        basic_cloud_ptr->height = 1;
    }
}


int main()
{
    // 读取原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    string file_name = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_1.ply";
    read_cloud(cloud, file_name);

    // 点云直通滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
    string save_seg = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_seg.ply";
    segment_cloud(cloud, cloud_seg, save_seg);

    // 点云下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    string save_filter = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_filter.ply";
    fast_uniform_sample(cloud_seg, cloud_filter, save_filter);

}

    // // 点云可视化
    // boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("显示点云"));
    // //左边窗口显示输入的点云,右边的窗口显示分割后的点云
    // int v1(0), v2(0);
    // viewer->createViewPort(0, 0, 0.5, 1, v1);
    // viewer->createViewPort(0.5, 0, 1, 1, v2);
    // viewer->setBackgroundColor(0, 0, 0, v1);
    // viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(cloud, 255, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZ>(cloud, color_in, "cloud_in", v1);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_in", v1);

    // viewer->addPointCloud<pcl::PointXYZ>(cloud0, "cloud_out", v2);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud_out", v2);
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_out", v2);

    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    //     boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    // }

    // return (0);