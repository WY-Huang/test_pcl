#include "lens.hpp"


// 读取pcd文件
int read_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filename) 
{ 
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) 
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
            if (z0 < 15)   //  分割阈值,  && z0 < -61.44
            {
                pointIdxVec.push_back(i);
            }
        }
        
    }
    pcl::copyPointCloud(*cloud_in, pointIdxVec, *cloud_out);

    if (save_path.empty() == false)
    {
        pcl::PCDWriter writer;
        
        writer.write(save_path, *cloud_out, false);
        cout << "Save segment file success -> " << save_path << endl;
    }
    cout << "The number of segment point clouds: " << cloud_out->points.size() << endl;
}

// 抽取所有轮廓的x坐标
void contour_line_coor_find(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<float> &contour_x_coor)
{
    float temp_value = 10000.1;
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        float x0 = cloud_in->points[i].x;
        if (x0 != temp_value)
        {
            temp_value = x0;
            contour_x_coor.push_back(temp_value);
            // cout << temp_value << "\t";
        }
    }
}

// 对所有轮廓进行圆拟合
void contour_line_circle_fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_ptr,
                             vector<vector<float>> &coefficient)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    float temp_value_start = 10000.1;
    float temp_value_end = 10000.2;
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        float x0 = cloud_in->points[i].x;
        if (x0 != temp_value_start)
        {
            temp_value_end = temp_value_start;
            temp_value_start = x0;
            if (i != 0)
            {
                vector<float> coeff;
                circleLeastFit(basic_cloud_ptr, coeff);
                // cout << "coeff:\n" << "y: " << coeff[0] << "\tz: " << coeff[1] << "\tr: " << coeff[2] <<endl;
                coefficient.push_back(coeff);
                // 生成圆点云
                pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud_fit(new pcl::PointCloud<pcl::PointXYZ>);
                generate_circle(temp_value_end, coeff[0], coeff[1], coeff[2], circle_cloud_fit);

                *all_cloud_ptr += * circle_cloud_fit;
            }
            basic_cloud_ptr->clear();
        }

        if (cloud_in->points[i].x == temp_value_start)
        {
            pcl::PointXYZ basic_point;

            basic_point.x = cloud_in->points[i].x;
            basic_point.y = cloud_in->points[i].y;
            basic_point.z = cloud_in->points[i].z;

            basic_cloud_ptr->points.push_back(basic_point);
        }
   
    }
    cout << "The number of circle_fit point clouds: " << all_cloud_ptr->points.size() << endl;
    pcl::PCDWriter writer;     
    writer.write("/home/wanyel/contours/lens_scanning/2023_04_27_10_17_02_849_all_circle.pcd", *all_cloud_ptr, false);
/*
    // 1)圆拟合方法1
    pcl::SACSegmentation<pcl::PointXYZ> seg;    // 创建分割对象
    seg.setOptimizeCoefficients(true);          // 可选择配置，设置模型系数需要优化
    seg.setModelType(pcl::SACMODEL_CIRCLE2D);      // 必须配置，设置分割的模型类型、所用随机参数估计方法
    seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations (10000);
    seg.setDistanceThreshold(0.1);            // 距离阈值表示点到估计模型的距离最大值，单位m

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);   // 创建分割时所需要的模型系数对象coefficients
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);             // 存储内点的点索引集合对象inliers
    seg.setInputCloud(basic_cloud_ptr);                  // 输入点云
    seg.segment(*inliers, *coeff);       // 实现分割，并存储分割结果到点集合inliers及存储平面模型系数coefficients
    cout << "coeff: " << coeff->values[0] << coeff->values[1] << coeff->values[2] << endl;
    /*
    // 2)圆拟合方法2
    //----------------------RANSAC框架----------------------------   
	pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr circle2D(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(basic_cloud_ptr));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(circle2D);	// 定义RANSAC算法对象
	ransac.setDistanceThreshold(0.1);	// 设置距离阈值
	ransac.setMaxIterations(100);		// 设置最大迭代次数
	ransac.computeModel();				// 拟合二维圆

	vector<int> inliers;				// 用于存放内点索引的vector
	ransac.getInliers(inliers);			// 获取内点索引

	//------------根据内点索引提取拟合的二维圆点云----------------
	// pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::copyPointCloud<pcl::PointXYZ>(*cloud_in, inliers, *circle_cloud);

	//--------------输出模型参数：圆心坐标和半径------------------
	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);
    cout << "coeff: " << coeff << endl;

    // 3)圆拟合方法3
    vector<float> coeff;
    circleLeastFit(basic_cloud_ptr, coeff);
    cout << "coeff:\n" << "y: " << coeff[0] << "\tz: " << coeff[1] << "\tr: " << coeff[2] <<endl;
    // 生成圆点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_cloud_fit(new pcl::PointCloud<pcl::PointXYZ>);
    generate_circle(0, coeff[0], coeff[1],coeff[2], circle_cloud_fit);
    cloud_viewer(basic_cloud_ptr, circle_cloud_fit);
*/
}

// 点云可视化
int cloud_viewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b)
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
    // viewer->addPointCloud<pcl::PointXYZ>(cloud_b, "cloud_out", v1);
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

// 二维可视化
void viewer_plot(const vector<vector<float>> &coefficient)
{
    // 创建PCL可视化对象
    pcl::visualization::PCLPlotter Plotter("2D Curve");

    // 创建一组曲线点
    std::vector<double> x_values;
    std::vector<double> y_values;
    for (int i=0; i<coefficient.size(); i++)
    {
        x_values.push_back(double(i));
        y_values.push_back(double(2.0 * coefficient[i][2]));
    }

    std::vector<double> y_values_mean;
    y_values_mean = mean_filter(y_values, 11);

    // 添加曲线到可视化对象
    Plotter.addPlotData(x_values, y_values, "curve", vtkChart::LINE);
    Plotter.addPlotData(x_values, y_values_mean, "curve", vtkChart::LINE);

    // 显示可视化对象
    while (!Plotter.wasStopped()) {
        Plotter.spinOnce();
        std::this_thread::sleep_for(std::chrono::microseconds(10000));
    }
}

// 创建圆形点云
void generate_circle(float x_in, float y_in, float z_in, float c_r, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    // 圆心坐标
    float cx = x_in;
    float cy = y_in;
    float cz = z_in;

    // 圆的半径
    float r = c_r;

    // 圆的参数方程：x = r*cos(theta), y = r*sin(theta)
    // 可以根据需要设置 theta 的步长，这里假设为 0.01
    for (float theta = 0; theta < 2*M_PI; theta += 0.01)
    {
        // 计算当前点的坐标
        float x = cx;
        float y = cy + r*cos(theta);
        float z = cz + r*sin(theta);

        // 创建点对象，并将其添加到点云对象中
        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;
        cloud->push_back(point);
    }
    // pcl::PCDWriter writer;  
    // writer.write("/home/wanyel/contours/lens_scanning/2023_04_27_10_17_02_849_x0_circle.pcd", *cloud, false);

}

// 最小二乘圆拟合
void circleLeastFit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector<float> &coeff)
{
    float sum_x = 0.0f, sum_y = 0.0f;
    float sum_x2 = 0.0f, sum_y2 = 0.0f;
    float sum_x3 = 0.0f, sum_y3 = 0.0f;
    float sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
    int N = cloud->points.size();
    for (int i = 0; i < N; i++)
    {
        float x = cloud->points[i].y;
        float y = cloud->points[i].z;
        float x2 = x * x;
        float y2 = y * y;
        sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }
    float C, D, E, G, H;
    float a, b, c;
    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

    coeff.push_back(a / (-2));
    coeff.push_back(b / (-2));
    coeff.push_back(sqrt(a * a + b * b - 4 * c) / 2);

}

// 一维均值滤波
std::vector<double> mean_filter(const std::vector<double>& data, int window_size)
{
    std::vector<double> filtered_data(data.size());
    for (int i = 0; i < data.size(); ++i)
    {
        double sum = 0.0;
        int count = 0;
        for (int j = i - window_size; j <= i + window_size; ++j)
        {
            if (j >= 0 && j < data.size())
            {
                sum += data[j];
                ++count;
            }
        }
        filtered_data[i] = sum / count;
    }
    return filtered_data;
}

// 分段并获得分段索引
void segment_index_get(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<float> &segment_interval_in, 
                        vector<vector<int>> &seg_idx_out)
{   
    seg_idx_out.resize(12);     // 创建12个区间存放每段的索引
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        float x0 = cloud_in->points[i].x;

        for (int k = 0; k < (segment_interval_in.size() - 1); ++k)
        {
            if (x0 > segment_interval_in[k] && x0 < segment_interval_in[k+1])
            {
                seg_idx_out[k].push_back(i);
                break;
            }
        }

    }
}

// 圆柱拟合
void cylinder_fit(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, vector<vector<int>> &seg_idx_in)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    

    for (int i = 0; i < seg_idx_in.size(); ++i)
    {
        pcl::copyPointCloud(*cloud_in, seg_idx_in[i], *seg_cloud_ptr);
        // save_cloud(seg_cloud_ptr, "/home/wanyel/contours/lens_scanning/20230505/cloud_" + std::to_string(i) + ".pcd");

        //-----------------------------法线估计--------------------------------
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;	// 创建法向量估计对象
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        n.setSearchMethod(tree);						       // 设置搜索方式
        n.setInputCloud(seg_cloud_ptr);						   // 设置输入点云
        n.setKSearch(30);								       // 设置K近邻搜索点的个数
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        n.compute(*normals);						           // 计算法向量，并将结果保存到normals中
        //----------------------------圆柱拟合--------------------------------
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;// 创建圆柱体分割对象
        seg.setInputCloud(seg_cloud_ptr);								// 设置输入点云
        seg.setInputNormals(normals);								    // 设置输入法向量
        seg.setOptimizeCoefficients(true);								// 设置对估计的模型系数需要进行优化
        seg.setModelType(pcl::SACMODEL_CYLINDER);						// 设置分割模型为圆柱体模型
        seg.setMethodType(pcl::SAC_RANSAC);								// 设置采用RANSAC算法进行参数估计
        seg.setNormalDistanceWeight(0.01);								// 设置表面法线权重系数
        seg.setMaxIterations(10000);									// 设置迭代的最大次数
        seg.setDistanceThreshold(0.25);									// 设置内点到模型距离的最大值
        seg.setRadiusLimits(10.0, 22.0);								// 设置圆柱模型半径的范围
        
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);	// 保存分割结果索引
        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);	// 保存圆柱体模型系数
        seg.segment(*inliers_cylinder, *coefficients_cylinder);			// 执行分割，将分割结果的索引保存到inliers_cylinder中，同时存储模型系数coefficients_cylinder

        pcl::PointCloud<pcl::PointXYZ>::Ptr seg_cloud_i(new pcl::PointCloud<pcl::PointXYZ>);    // 保存分割结果
        pcl::copyPointCloud(*seg_cloud_ptr, *inliers_cylinder, *seg_cloud_i);

        //---------------------------------结果可视化-------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));
        string cylinder_r;
        // cylinder_r.assign(coefficients_cylinder->values.begin(), coefficients_cylinder->values.end()); // 字符串可这样赋值
        for (const auto& value : coefficients_cylinder->values) 
        {
            cylinder_r += std::to_string(value) + ", ";
        }

        viewer->setBackgroundColor(0.7, 0.7, 0.7);
        viewer->addText("cylinder params: " + cylinder_r, 100, 10, "v1 text");
        viewer->setWindowName("Cycler Fit " + std::to_string(i));
        viewer->addPointCloud<pcl::PointXYZ>(seg_cloud_i, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
        viewer->addCylinder(*coefficients_cylinder, "cycler", 0); // 可视化拟合出来的圆柱模型

        // while (!viewer->wasStopped())
        // {
        //     viewer->spinOnce(100);
        
        // }
        viewer->spinOnce(100);
        if (i == 11)
        {
            viewer->spin();
        }
    }
    // viewer->spin();
    
}

// 统计滤波
void statistical_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);       
    sor.setMeanK(50);                  //统计时考虑查询点的邻近点数
    sor.setStddevMulThresh(1);         //判断是否为离群点的阈值，里边的数字表示标准差的倍数，1个标准差以上就是离群点。
    // 当判断点的k近邻平均距离(mean distance)大于全局的1倍标准差+平均距离(global distances mean and standard)，则为离群点。

    sor.filter (*cloud_out);           //存储内点
}

// 高斯滤波
void gaussian_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    //-----------基于高斯核函数的卷积滤波实现------------------------
	pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> kernel;
	kernel.setSigma(4);                     // 高斯函数的标准方差，决定函数的宽度
	kernel.setThresholdRelativeToSigma(4);  // 设置相对Sigma参数的距离阈值
	kernel.setThreshold(0.05);              // 设置距离阈值，若点间距离大于阈值则不予考虑

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_in);

	//---------设置Convolution 相关参数---------------------------
	pcl::filters::Convolution3D<pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>> convolution;
	convolution.setKernel(kernel);
	convolution.setInputCloud(cloud_in);
	convolution.setNumberOfThreads(8);
	convolution.setSearchMethod(tree);
	convolution.setRadiusSearch(0.01);

	convolution.convolve(*cloud_out);
}

// 保存点云
void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, string save_path)
{
    pcl::PCDWriter writer;
        
    writer.write(save_path, *cloud_out, false);
    cout << "Save segment file success -> " << save_path << endl;

    cout << "The number of segment point clouds: " << cloud_out->points.size() << endl;
}

// 对所有轮廓进行中值滤波
void contour_line_median_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    float temp_value_start = 10000.1;
    float temp_value_end = 10000.2;
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        float x0 = cloud_in->points[i].x;
        if (x0 != temp_value_start)
        {
            temp_value_end = temp_value_start;
            temp_value_start = x0;
            if (i != 0)
            {
                //---------------中值滤波（需要使用有序点云）-----------------
                // pcl::MedianFilter <pcl::PointXYZ> median;
                // median.setInputCloud(basic_cloud_ptr);
                // median.setWindowSize(10);           // 设置过滤器的窗口大小
                // median.setMaxAllowedMovement(0.1f); // 一个点允许沿z轴移动的最大距离 
                // pcl::PointCloud<pcl::PointXYZ>::Ptr median_filtered(new pcl::PointCloud<pcl::PointXYZ>);
                // median.filter(*median_filtered);
                vector<double> z_value;
                for (int k = 0; k < basic_cloud_ptr->points.size(); ++k)
                {
                    z_value.push_back(basic_cloud_ptr->points[k].z);
                }
                vector<double> z_median_value = mean_filter(z_value, 10);

                pcl::PointCloud<pcl::PointXYZ>::Ptr median_filtered(new pcl::PointCloud<pcl::PointXYZ>);
                for (int j = 0; j < basic_cloud_ptr->points.size(); ++j)
                {
                    pcl::PointXYZ basic_point;
                    basic_point.x = basic_cloud_ptr->points[j].x;
                    basic_point.y = basic_cloud_ptr->points[j].y;
                    basic_point.z = float(z_median_value[j]);

                    median_filtered->points.push_back(basic_point);
                }

                *all_cloud_ptr += * median_filtered;
            }
            basic_cloud_ptr->clear();
        }

        if (cloud_in->points[i].x == temp_value_start)
        {
            pcl::PointXYZ basic_point;

            basic_point.x = cloud_in->points[i].x;
            basic_point.y = cloud_in->points[i].y;
            basic_point.z = cloud_in->points[i].z;

            basic_cloud_ptr->points.push_back(basic_point);
        }
   
    }
    cout << "The number of median_fit point clouds: " << all_cloud_ptr->points.size() << endl;
    pcl::PCDWriter writer;     
    writer.write("/home/wanyel/contours/lens_scanning/20230505/2023_05_05_14_50_55_877_median.pcd", *all_cloud_ptr, false);
}