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

#include <pcl/surface/convex_hull.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

using namespace std;


void volume_demo();      // 球面体积及面积计算demo

void volume_cal(double &vol_std, double &area_std);     // 计算网格化点云的体积及面积

void convex_volume_cal();       // 凸包算法计算体积