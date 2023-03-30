#include <chrono>
#include <thread>
#include <boost/thread/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/random.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>

#include <vtkPointSource.h>

typedef pcl::PointXYZ PointT;
using namespace std;

// 生成球面点云，随机分布
void generate_sphere()
{

	// -------------------------生成位于球面上的点云---------------------------
	vtkNew<vtkPointSource> pointSource;
	pointSource->SetCenter(0.0, 0.0, 0.0);
	pointSource->SetNumberOfPoints(100000);
	pointSource->SetRadius(1.0);
	pointSource->SetDistributionToShell();  // 设置点分布在球面上。
	pointSource->Update();
	// ---------------------------转为PCD点云并保存----------------------------
	vtkSmartPointer<vtkPolyData> polydata = pointSource->GetOutput(); // 获取VTK中的PolyData数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
	pcl::PCDWriter w;
	w.writeBinaryCompressed("/home/wanyel/vs_code/test_pcl/imgs/srand_sphere.pcd", *cloud);
	// -------------------------------结果可视化-------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->setWindowName("生成球形点云");
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud"); // 设置点云大小
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
	}
}

// 生成球体点云，均匀实体
void generate_uniform_sphere()
{
  // 定义球体参数
  const float radius = 1.0f;
  const int num_points_per_sphere = 100000;
  const float sphere_area = 4 * M_PI * pow(radius, 2);

  // 计算点云密度
  const float density = static_cast<float>(num_points_per_sphere) / sphere_area;

  // 创建PCL点云对象
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

  // 使用PCL的随机数生成器来生成均匀分布的点云
  pcl::common::UniformGenerator<float> gen(-1.0f, 1.0f);  // 生成器用于产生球面上均匀分布的点

  // 生成点云
  for (int i = 0; i < num_points_per_sphere; i++) {
    PointT point;
    float x, y, z;
    do {
      x = gen.run();
      y = gen.run();
      z = gen.run();
    } while (sqrt(x*x + y*y + z*z) > 1.0f);  // 点的距离必须小于等于1，以便点位于球体内部

    // 缩放并平移点到球体上
    point.x = x * radius;
    point.y = y * radius;
    point.z = z * radius;

    // 将点添加到点云中
    cloud->points.push_back(point);
  }

  // 设置点云大小并保存到磁盘
  cloud->width = cloud->points.size();
  cloud->height = 1;
  pcl::io::savePCDFileBinary("/home/wanyel/vs_code/test_pcl/imgs/cloud/uniform_sphere.pcd", *cloud);

}

// 以椭圆为边线沿z轴拉伸获取其点云，并赋予红绿蓝渐变色。
void create_ellipse_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud_ptr) {
    
    uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf(float(angle / 180 * M_PI));
            basic_point.y = sinf(float(angle / 180 * M_PI));
            basic_point.z = z;

            pcl::PointXYZRGB point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back(point);
        }
        if (z < 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }
    }
    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
}

// 构造圆柱面点云  
void create_cylinder_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& basic_cloud_ptr)
{
    uint8_t r(255), g(15), b(15);
    for (float z = -1.0; z <= 1.0; z += 0.05)
    {
        for (float angle = 0.0; angle <= 360.0; angle += 5.0)
        {
            pcl::PointXYZRGB basic_point;

            basic_point.x = cosf(pcl::deg2rad(angle));
            basic_point.y = sinf(pcl::deg2rad(angle));
            basic_point.z = z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            basic_point.rgb = *reinterpret_cast<float*>(&rgb);
            basic_cloud_ptr->points.push_back(basic_point);
        }
        if (z < 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }

        basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
        basic_cloud_ptr->height = 1;
    }
}

// 构造球面点云  
void creat_sphere_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr)
{
    uint8_t r(255), g(15), b(15);
    float radius = 1;

    for (float angle1 = 0.0; angle1 <= 180.0; angle1 += 2.0)
    {
        for (float angle2 = 0.0; angle2 <= 360.0; angle2 += 2.0)
        {
            pcl::PointXYZRGB basic_point;

            basic_point.x = radius * sinf(pcl::deg2rad(angle1)) * cosf(pcl::deg2rad(angle2));
            basic_point.y = radius * sinf(pcl::deg2rad(angle1)) * sinf(pcl::deg2rad(angle2));
            basic_point.z = radius * cosf(pcl::deg2rad(angle1));

            // Add noise
            int add_noise = 0;
            if (add_noise)
            {
                basic_point.x = basic_point.x + 0.002 * rand() / double(RAND_MAX);
                basic_point.y = basic_point.y + 0.002 * rand() / double(RAND_MAX);
                basic_point.z = basic_point.z + 0.002 * rand() / double(RAND_MAX);
            }

            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            basic_point.rgb = *reinterpret_cast<float*>(&rgb);
            basic_cloud_ptr->points.push_back(basic_point);

        }
        if (radius != 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }

        basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
        basic_cloud_ptr->height = 1;
    }
}

// 构造椭球面点云 spheroid 
void creat_spheroid_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr)
{
    uint8_t r(255), g(15), b(15);
    float r_a = 5.0;
    float r_b = 4.0;
    float r_c = 3.0;

    for (float angle1 = 0.0; angle1 <= 180.0; angle1 += 0.5)
    {
        for (float angle2 = 0.0; angle2 <= 360.0; angle2 += 2.0)
        {
            pcl::PointXYZRGB basic_point;

            basic_point.x = r_a * sinf(pcl::deg2rad(angle1)) * cosf(pcl::deg2rad(angle2));
            basic_point.y = r_b * sinf(pcl::deg2rad(angle1)) * sinf(pcl::deg2rad(angle2));
            basic_point.z = r_c * cosf(pcl::deg2rad(angle1));

            // Add noise
            int add_noise = 0;
            if (add_noise)
            {
                basic_point.x = basic_point.x + 0.01 * rand() / double(RAND_MAX);
                basic_point.y = basic_point.y + 0.01 * rand() / double(RAND_MAX);
                basic_point.z = basic_point.z + 0.01 * rand() / double(RAND_MAX);
            }

            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            basic_point.rgb = *reinterpret_cast<float*>(&rgb);
            basic_cloud_ptr->points.push_back(basic_point);

        }
        if (r_c != 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }

        basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
        basic_cloud_ptr->height = 1;
    }
}



int main(int argc, char** argv)
{
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_downSampled(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>);
    
    int generate = 1;
    if (generate)
    {
        // generate_sphere();
        // generate_uniform_sphere();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        // create_ellipse_pointcloud(cloud_xyzrgb);
        // create_cylinder_pointcloud(cloud_xyzrgb);
        creat_sphere_pointcloud(cloud_xyzrgb);
        // creat_spheroid_pointcloud(cloud_xyzrgb);

        pcl::copyPointCloud(*cloud_xyzrgb, *cloud);
        pcl::io::savePCDFile ("/home/wanyel/vs_code/test_pcl/imgs/cloud/cloud_0_sphere.pcd", *cloud);
    }
    else
    {
        // 加载规则形状点云
        if (pcl::io::loadPCDFile("/home/wanyel/vs_code/test_pcl/imgs/raw_pcd/2022_09_26_08_41_08_601_final.pcd", *cloud) == -1)
        {
            cout << "点云数据读取失败！" << endl;
        }
    }
	
    std::cout << "Orginal points number: " << cloud->points.size() << std::endl;

    /*==================================================================================*/
	// 1）下采样
    pcl::VoxelGrid<PointT> downSampled;                 // 创建滤波对象
    downSampled.setInputCloud (cloud);                  // 设置需要过滤的点云给滤波对象
    downSampled.setLeafSize (0.01f, 0.01f, 0.01f);      // 设置滤波时创建的体素体积为1cm的立方体
    downSampled.filter (*cloud_downSampled);            // 执行滤波处理，存储输出
    std::cout << "DownSampled points number: " << cloud_downSampled->points.size() << std::endl;
    pcl::io::savePCDFile ("/home/wanyel/vs_code/test_pcl/imgs/cloud/cloud_1_downsampledPC.pcd", *cloud_downSampled);

	// 2）统计滤波
    pcl::StatisticalOutlierRemoval<PointT> statisOutlierRemoval;       // 创建滤波器对象
    statisOutlierRemoval.setInputCloud (cloud_downSampled);            // 设置待滤波的点云
    statisOutlierRemoval.setMeanK (50);                                // 设置在进行统计时考虑查询点临近点数
    statisOutlierRemoval.setStddevMulThresh (3.0);                     // 设置判断是否为离群点的阀值:均值+1.0*标准差
    statisOutlierRemoval.filter (*cloud_filtered);                     // 滤波结果存储到cloud_filtered
    std::cout << "Cloud_filtered points number: " << cloud_filtered->points.size() << std::endl;
    pcl::io::savePCDFile ("/home/wanyel/vs_code/test_pcl/imgs/cloud/cloud_2_filteredPC.pcd", *cloud_filtered);

	// 3）对点云重采样  
    pcl::search::KdTree<PointT>::Ptr treeSampling (new pcl::search::KdTree<PointT>);// 创建用于最近邻搜索的KD-Tree
    pcl::PointCloud<PointT> mls_point;              // 输出MLS
    pcl::MovingLeastSquares<PointT,PointT> mls;     // 定义最小二乘实现的对象mls
    mls.setComputeNormals(false);                   // 设置在最小二乘计算中是否需要存储计算的法线
    mls.setInputCloud(cloud_filtered);              // 设置待处理点云
    mls.setPolynomialOrder(2);                      // 拟合2阶多项式拟合
    // mls.setPolynomialFit(false);                 // 设置为false可以 加速 smooth
    mls.setSearchMethod(treeSampling);              // 设置KD-Tree作为搜索方法
    mls.setSearchRadius(0.05);                      // 单位m.设置用于拟合的K近邻半径
    mls.process(mls_point);                         // 输出
    // 输出重采样结果
    cloud_smoothed = mls_point.makeShared();
    std::cout<<"Cloud_smoothed: "<<cloud_smoothed->size() <<std::endl;
    //save cloud_smoothed
    pcl::io::savePCDFileASCII("/home/wanyel/vs_code/test_pcl/imgs/cloud/cloud_3_smoothed.pcd",*cloud_smoothed);

    // 4）法线估计
    pcl::NormalEstimation<PointT,pcl::Normal> normalEstimation;                 // 创建法线估计的对象
    normalEstimation.setInputCloud(cloud_smoothed);                             // 输入点云
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);     // 创建用于最近邻搜索的KD-Tree
    normalEstimation.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);// 定义输出的点云法线
    // K近邻确定方法，使用k个最近点，或者确定一个以r为半径的圆内的点集来确定都可以，两者选1即可
    normalEstimation.setKSearch(30);                                            // 使用当前点周围最近的10个点
    // normalEstimation.setRadiusSearch(0.03);                                  // 对于每一个点都用半径为3cm的近邻搜索方式
    normalEstimation.compute(*normals);                                         // 计算法线
    // 输出法线
    std::cout<<"normals: "<<normals->size()<<", "<<"normals fields: "<<pcl::getFieldsList(*normals)<<std::endl;
    // pcl::io::savePCDFileASCII("/home/wanyel/vs_code/test_pcl/imgs/cloud/cloud_4_normals.pcd",*normals);
	
	// 将点云位姿、颜色、法线信息连接到一起
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud_smoothed, *normals, *cloud_with_normals);
    pcl::io::savePCDFileASCII("/home/wanyel/vs_code/test_pcl/imgs/cloud/cloud_4_with_normals.pcd",*cloud_with_normals);
	
    // 5)定义搜索树对象
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
        pcl::io::savePLYFile("/home/wanyel/vs_code/test_pcl/imgs/cloud/poisson_5_mesh.ply", triangles);
    }
    else
    {
        // 贪婪三角化
        pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;   // 定义三角化对象

        // 设置三角化参数
        gp3.setSearchRadius(0.1);  //设置搜索时的半径，也就是KNN的球半径
        gp3.setMu (2.5);  //设置样本点搜索其近邻点的最远距离为2.5倍（典型值2.5-3），这样使得算法自适应点云密度的变化
        gp3.setMaximumNearestNeighbors (100);    //设置样本点最多可搜索的邻域个数，典型值是50-100

        // gp3.setMinimumAngle(M_PI/18); // 设置三角化后得到的三角形内角的最小的角度为10°
        // gp3.setMaximumAngle(2*M_PI/3); // 设置三角化后得到的三角形内角的最大角度为120°

        gp3.setMaximumSurfaceAngle(M_PI/2); // 设置某点法线方向偏离样本点法线的最大角度45°，如果超过，连接时不考虑该点
        gp3.setNormalConsistency(false);  //设置该参数为true保证法线朝向一致，设置为false的话不会进行法线一致性检查

        gp3.setInputCloud (cloud_with_normals);     //设置输入点云为有向点云
        gp3.setSearchMethod (tree2);   //设置搜索方式
        gp3.reconstruct (triangles);  //重建提取三角化

        // 保存网格图
        pcl::io::saveVTKFile("/home/wanyel/vs_code/test_pcl/imgs/cloud/greedy_mesh.vtk", triangles);
        pcl::io::savePLYFile ("/home/wanyel/vs_code/test_pcl/imgs/cloud/greedy_mesh.ply", triangles);
    }
	
    // 显示法线结果
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL Viewer"));
    // viewer->setBackgroundColor (0, 0, 0);
    // pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    // viewer->addPointCloud<PointT> (cloud, rgb, "smooth cloud");
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "smooth cloud");
    // viewer->addPointCloudNormals<PointT, pcl::Normal> (cloud, normals, 20, 0.05, "normals");

    // viewer->initCameraParameters ();

    // 显示网格化结果
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPolygonMesh(triangles, "mesh");

    while (!viewer->wasStopped())
    {
    	viewer->spinOnce(100);
    	// boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
    return 1;
}
