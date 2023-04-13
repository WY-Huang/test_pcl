#include "pcl_volume.hpp"

// 球面体积及面积计算demo
void volume_demo()
{
    vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(1);
    sphereSource->SetPhiResolution(100);
    sphereSource->SetThetaResolution(100);
    sphereSource->Update();

    vtkSmartPointer<vtkTriangleFilter> triangleFilter= vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(sphereSource->GetOutput());
    triangleFilter->Update();

    vtkSmartPointer<vtkMassProperties> polygonProperties = vtkSmartPointer<vtkMassProperties>::New();
    polygonProperties->SetInputData(triangleFilter->GetOutput());
    polygonProperties->Update();

    double area = polygonProperties->GetSurfaceArea();
    double vol = polygonProperties->GetVolume();
    double maxArea = polygonProperties->GetMaxCellArea();//最大单元面积
	double minArea = polygonProperties->GetMinCellArea();//最小单元面积

	cout << "最大单元面积：" << maxArea << endl;
	cout << "最小单元面积：" << minArea << endl;
	cout << "计算体积为：" << vol << endl;
	cout << "计算表面积为：" << area << endl;

}

// 计算网格化点云的体积及面积
void volume_cal(double &vol_std, double &area_std)
{
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_poisson_mesh.ply");
    reader->Update();
	vtkSmartPointer<vtkTriangleFilter> tri = vtkSmartPointer<vtkTriangleFilter>::New();
	tri->SetInputData(reader->GetOutput());
	tri->Update();
	vtkSmartPointer<vtkMassProperties> poly = vtkSmartPointer<vtkMassProperties>::New();
	poly->SetInputData(tri->GetOutput());
	poly->Update();

	double vol = poly->GetVolume();             //体积
	double area = poly->GetSurfaceArea();       //表面积
    double maxArea = poly->GetMaxCellArea();    //最大单元面积
	double minArea = poly->GetMinCellArea();    //最小单元面积

    double vol_err = fabs(vol_std - vol) / vol_std;
    double area_err = fabs(area_std - area) / area;

	double density = 65341.0 / area_std;

	cout << "点云密度为：" << density << endl;
	cout << "计算体积为：" << vol << "\t误差为：" << (vol_std - vol) << "\t百分比为：" << vol_err << endl;
	cout << "计算表面积为：" << area << "\t误差为：" << (area_std - area) << "\t百分比为：" << area_err << endl;

	//============================可视化==========================
    // 半径是R的圆球的体积计算公式是：：V＝4πR^3 / 3   半径是R的圆球的面积公式：S=4πR^2
	//---------------用于渲染多边形几何数据-------------------
	vtkSmartPointer<vtkPolyDataMapper> cylinderMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	cylinderMapper->SetInputConnection(reader->GetOutputPort()); 
	vtkSmartPointer<vtkActor> cylinderActor = vtkSmartPointer<vtkActor>::New();
	cylinderActor->SetMapper(cylinderMapper);
	cylinderActor->GetProperty()->SetColor(1.0, 0.75, 0.0);
	//-----------------管理场景渲染过程-----------------------
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(cylinderActor);
	renderer->SetBackground(1.0, 1.0, 1.0);
	//-------------------设置窗口参数------------------------
	vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
	renWin->AddRenderer(renderer);
	renWin->SetSize(640, 480);//设置窗口大小，以像素为单元
	renWin->Render();
	renWin->SetWindowName("Render");
	//---------------------交互机制--------------------------
	vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
	iren->SetRenderWindow(renWin);
	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	iren->SetInteractorStyle(style);
	iren->Initialize();
	iren->Start();
	//======================可视化结束===========================
    // system("pause");
    // return (0);
}

// 凸包算法计算体积
void convex_volume_cal()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile<pcl::PointXYZ>("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_mirr.ply", *cloud);

	pcl::ConvexHull<pcl::PointXYZ> hull;                  
	hull.setInputCloud(cloud);                   
	hull.setDimension(3);		// 设置凸包维度
	hull.setComputeAreaVolume(true);

	std::vector<pcl::Vertices> polygons;		// polygons保存的是所有凸包多边形的顶点在surface_hull中的下标
	pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);	// surface_hull是所有凸包多边形的顶点
	hull.reconstruct(*surface_hull, polygons);  // 凸包点云存放在surface_hull中,polygons中的Vertices存放一组点的索引，索引是surface_hull中的点对应的索引

	double convex_volume = hull.getTotalVolume();

	cout << surface_hull->size() << endl;
	cout << "凸包体积： " << convex_volume << endl;

	// ---------------------- Visualizer -------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->setBackgroundColor(0, 0, 0);

	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 255, 0);
	// viewer->addPointCloud(cloud, color_handler, "sample cloud");
	// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handlerK(surface_hull, 255, 0, 0);
	viewer->addPointCloud(surface_hull, color_handlerK, "point");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "point");

	// viewer->addPolygon<pcl::PointXYZ>(surface_hull, 0, 0, 255, "polyline");
	viewer->addPolygonMesh<pcl::PointXYZ>(surface_hull, polygons, "polyline");
	viewer->setRepresentationToWireframeForAllActors();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}
}      
