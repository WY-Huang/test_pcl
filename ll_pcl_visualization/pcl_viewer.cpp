#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
  
int user_data;
// 设置pcl背景，添加一个球体
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0, 238, 41);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;
}
// 屏幕上添加文本，显示窗口刷新次数
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 100, 100, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}
    
int main ()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile ("/home/wanyel/vs_code/test_pcl/imgs/cloud/uniform_sphere.pcd", *cloud);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
  
    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    // pcl::visualization::PCLVisualizer viewer_b;
    // viewer_b.setBackgroundColor (0, 238, 41);
 
    //use the following functions to get access to the underlying more advanced/powerful
    //PCLVisualizer
  
    //This will only get called once
    viewer.runOnVisualizationThreadOnce (viewerOneOff);
 
    //This will get called once per visualization iteration
    viewer.runOnVisualizationThread (viewerPsycho);
    while (!viewer.wasStopped ())
    {
    //you can also do cool processing here
    //FIXME: Note that this is running in a separate thread from viewerPsycho
    //and you should guard against race conditions yourself...
    user_data++;
    // std::cout << user_data << std::endl;
    }
    return 0;
}