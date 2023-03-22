#include <iostream>
#include <math.h>

#include <vtkPLYReader.h>
#include <vtkTriangleFilter.h>
#include <vtkMassProperties.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
//可视化相关头文件
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkProperty.h>
#include <vtkAutoInit.h> 
#include <vtkActor.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace std;

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
    reader->SetFileName("/home/wanyel/vs_code/test_pcl/imgs/cloud/poisson_5_mesh.ply");
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

int main(int argc, char** argv)
{
    double r = 1.0;
    double vol_std = 4 * M_PI * r * r * r / 3;
    double area_std = 4 * M_PI * r * r;

    // 梯形台尺寸：66.61*38.85 20.33*20.28 19.50    V=[S1+S2+√(S1*S2)]*h/3 
    double vol_std_prism = (66.61*38.85 + 20.33*20.28 + sqrt(66.61*38.85 + 20.33*20.28)) * 19.50 / 3;
    cout << "梯形台理论体积：" << vol_std_prism << endl;

    cout << "半径为1的球体标准体积：" << vol_std << endl;
    cout << "半径为1的球体标准表面积：" << area_std << endl;
    cout << "===============================" << endl;
    // volume_demo();
    // cout << "===============================" << endl;
    volume_cal(vol_std, area_std);

}

