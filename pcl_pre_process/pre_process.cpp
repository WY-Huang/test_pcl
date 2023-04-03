#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>	        //getMinMax3D()函数所在头文件
#include <pcl/features/normal_3d.h>
#include <pcl/surface/poisson.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <boost/thread/thread.hpp>
#include <pcl/surface/gp3.h>
#include <pcl/surface/convex_hull.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>

using namespace std;
typedef pcl::PointXYZ PointT;


// 读取ply文件
int read_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename) 
{ 
	if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1) 
    {
		cout << "Could not read file." << endl;
		return -1;
	}
	cout << "Load file success -> " << filename << endl;
	cout << "The number of raw point clouds: " << cloud->points.size() << endl;
	return 0;
}


// 根据 xyz 坐标进行阈值分割(直通濾波)
void segment_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, 
                    string save_path="", string seg_method="")
{
    vector<int> pointIdxVec;

    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        float x0 = cloud_in->points[i].x;
        float y0 = cloud_in->points[i].y;
        float z0 = cloud_in->points[i].z;

        if (seg_method == "xyz_seg")
        {
            if (x0 > -12 && x0 < 29 && y0 > 18 && z0 > -61.5)   //  分割阈值
            {
                pointIdxVec.push_back(i);
            }
        }
        else if (seg_method == "z_seg")
        {
            if (z0 > -61.5 && z0 < -61.44)   //  分割阈值
            {
                pointIdxVec.push_back(i);
            }
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


// 实现从点云中每every_k_points个点采样一次
void fast_uniform_sample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                        string save_path="")
{
	int every_k_points = 100;
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
    // min (-11.512,18.552,-61.498)  max (28.076,81.3,-41.848)
    for (float x_p = -11.512; x_p <= 28.076; x_p += 0.1)
    {
        for (float y_p = 18.552; y_p <= 81.3; y_p += 0.1)
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


// 点云拼接
void cloud_concatenate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, string save_path="")
{
    *cloud_out = (*cloud_a) + (*cloud_b);
    if (save_path.empty() == false)
    {
        pcl::PLYWriter writer;
        
        writer.write(save_path, *cloud_out, false, false);
        cout << "Save concatenate file success -> " << save_path << endl;
    }
    cout << "The number of concatenate point clouds: " << cloud_out->points.size() << endl;
}


// 点云按z轴投影为平面点云
void cloud_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                    string save_path="")
{
    pcl::PointXYZ new_point;
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        new_point.x = cloud_in -> points[i].x;
        new_point.y = cloud_in -> points[i].y;
        new_point.z = 0;

        cloud_out -> points.push_back(new_point);
    }

    cloud_out->width = (int)cloud_out->points.size();
    cloud_out->height = 1;

    if (save_path.empty() == false)
    {
        pcl::PLYWriter writer;
        
        writer.write(save_path, *cloud_out, false, false);
        cout << "Save plane file success -> " << save_path << endl;
    }
    cout << "The number of plane point clouds: " << cloud_out->points.size() << endl;
}


// pcl数据转为cv格式
void point_pcl_to_cv(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<cv::Point2f> &cv_out)
{
    cv::Point2f cv_point;
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        cv_point.x = cloud_in -> points[i].x;
        cv_point.y = cloud_in -> points[i].y;

        cv_out.push_back(cv_point);
    }
}


// 点云筛选
void cloud_filter_contour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                    std::vector<cv::Point2f> contour, string save_path="")
{
    pcl::Indices indices;
    cv::Point2f pt;
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        pt.x = cloud_in -> points[i].x;
        pt.y = cloud_in -> points[i].y;
        double re_value = cv::pointPolygonTest(contour, pt, false);
        
        if (re_value >= 0)
        {
            indices.push_back(i);
        }
    }
    pcl::copyPointCloud(*cloud_in, indices, *cloud_out);

    cloud_out->width = (int)cloud_out->points.size();
    cloud_out->height = 1;

    if (save_path.empty() == false)
    {
        pcl::PLYWriter writer;
        
        writer.write(save_path, *cloud_out, false, false);
        cout << "Save plane_filter file success -> " << save_path << endl;
    }
    cout << "The number of plane_filter point clouds: " << cloud_out->points.size() << endl;
}


// 点云在Z平面上镜像
void cloud_mirror(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                    string save_path="")
{
    pcl::PointXYZ new_point;
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        new_point.x = cloud_in -> points[i].x;
        new_point.y = cloud_in -> points[i].y;
        new_point.z = -61.48 * 2 - (cloud_in -> points[i].z);

        cloud_out->points.push_back(new_point);
    }

    *cloud_out = (*cloud_in) + (*cloud_out);

    cloud_out->width = (int)cloud_out->points.size();
    cloud_out->height = 1;

    if (save_path.empty() == false)
    {
        pcl::PLYWriter writer;
        
        writer.write(save_path, *cloud_out, false, false);
        cout << "Save cloud_mirror file success -> " << save_path << endl;
    }
    cout << "The number of cloud_mirror point clouds: " << cloud_out->points.size() << endl;
}


int main()
{
    // 读取原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    string file_name = "/home/wanyel/contours/20220926/PointCloud_20220913092246086.ply";
    read_cloud(cloud, file_name);

    // // 点云直通滤波
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
    // string save_seg = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_seg.ply";
    // segment_cloud(cloud, cloud_seg, save_seg, "xyz_seg");

    // 点云下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    string save_filter = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_sample_100.ply";
    fast_uniform_sample(cloud, cloud_filter, save_filter);

    // // 读取原始点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    // string file_name = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_filter.ply";
    // read_cloud(cloud_filter, file_name);

    bool complete = true;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);

    if (complete)
    {
        // pcl::PointXYZ min;	                        // xyz的最小值
        // pcl::PointXYZ max;	                        // xyz的最大值
        // pcl::getMinMax3D(*cloud_filter, min, max);	// 获取所有点中的坐标最值
        // std::cout << "min: " << min << "\nmax: " << max << endl;

        // 按z轴坐标分割出部分点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg_z(new pcl::PointCloud<pcl::PointXYZ>);
        string save_seg_z = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_z_seg.ply";
        segment_cloud(cloud_filter, cloud_seg_z, save_seg_z, "z_seg");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_z(new pcl::PointCloud<pcl::PointXYZ>);
        string save_plane_z = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_z_plane.ply";
        cloud_to_plane(cloud_seg_z, cloud_plane_z, save_plane_z);

        // 提取投影平面点云的凸多边形边界
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_vex_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> hull;        //创建凸包对象
        hull.setInputCloud(cloud_plane_z);          //设置输入点云
        hull.setDimension(2);                       //设置输入数据的维度(2D)
        hull.reconstruct(*cloud_vex_hull);          //计算2D凸包结果

        std::cerr << "凸多边形的点数: " << cloud_vex_hull->points.size() << std::endl;

        pcl::PLYWriter writer;
        writer.write("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_z_contour.ply", *cloud_vex_hull, true);

        // 生成平面点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        creat_plane_pointcloud(cloud_plane);
        
        // pcl轮廓转为cv
        std::vector<cv::Point2f> contour_hull;
        point_pcl_to_cv(cloud_vex_hull, contour_hull);

        // 根据轮廓筛选平面点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_filter(new pcl::PointCloud<pcl::PointXYZ>);
        string save_plane_filter = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_z_plane_filter.ply";
        cloud_filter_contour(cloud_plane, cloud_plane_filter, contour_hull, save_plane_filter);
        
        // 拼接到滤波后的点云上
        string save_concate = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_concate.ply";
        cloud_concatenate(cloud_filter, cloud_plane_filter, cloud_final, save_concate);
    }
    else
    {
        string save_cloud_mirr = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_mirr.ply";
        cloud_mirror(cloud_filter, cloud_final, save_cloud_mirr);
    }

    // 4）法线估计
    pcl::NormalEstimation<PointT,pcl::Normal> normalEstimation;                 // 创建法线估计的对象
    normalEstimation.setInputCloud(cloud_final);                             // 输入点云
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);     // 创建用于最近邻搜索的KD-Tree
    normalEstimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);// 定义输出的点云法线

    normalEstimation.setKSearch(10);                                            // 使用当前点周围最近的10个点
    // normalEstimation.setRadiusSearch(0.03);                                  // 对于每一个点都用半径为3cm的近邻搜索方式
    normalEstimation.compute(*normals);                                         // 计算法线

    std::cout<<"normals: "<<normals->size()<<", "<<"normals fields: "<<pcl::getFieldsList(*normals)<<std::endl;
    // pcl::io::savePCDFileASCII("/home/wanyel/vs_code/test_pcl/imgs/cloud/cloud_4_normals.pcd",*normals);
	
	// 将点云位姿、颜色、法线信息连接到一起
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_final, *normals, *cloud_with_normals);
    // pcl::io::savePCDFileASCII("/home/wanyel/vs_code/test_pcl/imgs/cloud/cloud_4_with_normals.pcd",*cloud_with_normals);
	
    // 5)定义搜索树对象，进行3D重建，<<泊松重建有水密性要求>>
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);
    
    pcl::PolygonMesh triangles ;            //创建多边形网格，用于存储结果

    int poisson = 1;
    if (poisson)
    {
        //创建Poisson对象，并设置参数
        pcl::Poisson<pcl::PointNormal> pn ;
        pn.setConfidence(false);            // 是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。
        pn.setDegree(2);                    // 设置参数degree[1,5],值越大越精细，耗时越久。
        pn.setDepth(8);                     // 树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。
        pn.setIsoDivide(8);                 // 用于提取ISO等值面的算法的深度
        pn.setManifold(false);              // 是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加
        pn.setOutputPolygons(false);        // 是否输出多边形网格（而不是三角化移动立方体的结果）
        pn.setSamplesPerNode(3.0);          // 设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑
        pn.setScale(1.25);                  // 设置用于重构的立方体直径和样本边界立方体直径的比率。
        pn.setSolverDivide(8);              // 设置求解线性方程组的Gauss-Seidel迭代方法的深度
        //pn.setIndices();

        //设置搜索方法和输入点云
        pn.setSearchMethod(tree2);
        pn.setInputCloud(cloud_with_normals);
        
        //执行重构
        pn.performReconstruction(triangles);

        //保存网格图
        pcl::io::savePLYFile("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_poisson_mesh.ply", triangles);
    }
    else
    {
        // 贪婪三角化
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象

        // 设置三角化参数
        gp3.setSearchRadius(0.01);                     //设置搜索时的半径，也就是KNN的球半径
        gp3.setMu (2.5);                              //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
        gp3.setMaximumNearestNeighbors (100);         //设置样本点最多可搜索的邻域个数，典型值是50-100

        // gp3.setMinimumAngle(M_PI/18);              // 设置三角化后得到的三角形内角的最小的角度为10°
        // gp3.setMaximumAngle(2*M_PI/3);             // 设置三角化后得到的三角形内角的最大角度为120°

        gp3.setMaximumSurfaceAngle(M_PI/2);           // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
        gp3.setNormalConsistency(false);              //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

        gp3.setInputCloud (cloud_with_normals);       //设置输入点云为有向点云
        gp3.setSearchMethod (tree2);                  //设置搜索方式
        gp3.reconstruct (triangles);                  //重建提取三角化

        //保存网格图
        pcl::io::savePLYFile("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_greedyTri_mesh.ply", triangles);
    }

    // 显示网格化结果
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(triangles, "mesh");

    while (!viewer->wasStopped())
    {
    	viewer->spinOnce(100);
    	// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
    return 1;

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