#include "ground_seg.hpp"


Ground_segment::Ground_segment(string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    read_cloud(cloud, filename);                // 读取点云
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    float ground_thick = 0.1;       // 目标物体与地面的间距
    ground_fit(cloud, coefficients, inliers, ground_thick);   // 拟合地面
 
    //***********************************************************************
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 1> 提取地面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*cloud_filtered);

    cout << "Ground cloud after filtering: " << endl;
    cout<< *cloud_filtered << std::endl;
    
    writer.write<pcl::PointXYZ>("/home/wanyel/contours/20220926/pointCloud_20220913092246086_mod_sample_100_ground.pcd", *cloud_filtered, false);

    // 2> 提取除地面外的物体
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    cout << "Object cloud after filtering: " << endl;
    cout << *cloud_filtered << endl;

    writer.write<pcl::PointXYZ>("/home/wanyel/contours/20220926/pointCloud_20220913092246086_mod_sample_100_object.pcd", *cloud_filtered, false);

    // 3> 分割地面后的目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_ground(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Vector4f plane; // 定义平面参数，平面方程中的常数 d 就是原点到平面的距离（归一化法线）
    for (int i=0; i < plane.size(); i++)
    {
        plane[i] = coefficients -> values[i];
        if (i == (plane.size()-1))
        {
            plane[i] = plane[i] - ground_thick;
        }
    }
    cout << plane << endl;
    
    pcl::IndicesPtr indices(new vector<int>()); // 保存裁剪点的索引
    pcl::PlaneClipper3D<pcl::PointXYZ> clipper(plane);
    clipper.setPlaneParameters(plane);
    clipper.clipPointCloud3D(*cloud, *indices);

    pcl::ExtractIndices<pcl::PointXYZ> extract_object;
    extract_object.setInputCloud(cloud);
    extract_object.setIndices(indices);
    extract_object.setNegative(false);
    extract_object.filter(*filter_ground);

    cout << "The filtered points data:  " << filter_ground->points.size() << endl;
    writer.write<pcl::PointXYZ>("/home/wanyel/contours/20220926/pointCloud_20220913092246086_mod_sample_100_object_filter.pcd", *filter_ground, false);

    cloud_viewer(cloud, filter_ground);
}

Ground_segment::~Ground_segment()
{

}

// 读取ply文件
int Ground_segment::read_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename) 
{ 
	// pcl::PCDReader reader;
    // 读入点云PCD文件
    // reader.read("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_sample_100_filter.pcd", *cloud);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1) 
    {
		cout << "Could not read file." << endl;
		return -1;
	}
	cout << "Load file success -> " << filename << endl;
	cout << "The number of raw point clouds: " << cloud->points.size() << endl;

	return 0;
}

// 地面点云拟合
void Ground_segment::ground_fit(pcl::PointCloud<pcl::PointXYZ>::Ptr ground, pcl::ModelCoefficients::Ptr coefficients,
                                pcl::PointIndices::Ptr inliers, float thick)
{

    pcl::SACSegmentation<pcl::PointXYZ> seg;    // 创建分割对象
    seg.setOptimizeCoefficients(true);          // 可选择配置，设置模型系数需要优化
    seg.setModelType(pcl::SACMODEL_PLANE);      // 必须配置，设置分割的模型类型、所用随机参数估计方法
    seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations (10000);
    seg.setDistanceThreshold(thick);             // 距离阈值表示点到估计模型的距离最大值，单位m

    // float angle = 10;                     
    // float EpsAngle= pcl::deg2rad(angle);     // 角度转弧度
    // Eigen::Vector3f Axis(0.0, 0.0, 1.0); 
    // seg.setAxis(Axis);                       // 指定的轴
    // seg.setEpsAngle(EpsAngle);               // 夹角阈值(弧度制)

    seg.setInputCloud(ground);                  // 输入点云
    seg.segment(*inliers, *coefficients);       // 实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients

    //-----------输出平面模型的系数 a,b,c,d-----------
    std::cout << "Model coefficients: " << coefficients->values[0] << " "
         << coefficients->values[1] << " "
         << coefficients->values[2] << " "
         << coefficients->values[3] << std::endl;
    std::cout << "Model inliers: " << inliers->indices.size() << std::endl;
}

// 点云可视化
int Ground_segment::cloud_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("Cloud Visualization"));
    //左边窗口显示输入的点云,右边的窗口显示分割后的点云
    int v1(0), v2(0);
    viewer->createViewPort(0, 0, 0.5, 1, v1);
    viewer->createViewPort(0.5, 0, 1, 1, v2);
    viewer->setBackgroundColor(0, 0, 0, v1);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_in(cloud_a, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_a, color_in, "cloud_in", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_in", v1);

    viewer->addPointCloud<pcl::PointXYZ>(cloud_b, "cloud_out", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "cloud_out", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_out", v2);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

    return 0;
}