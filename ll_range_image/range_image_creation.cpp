#include <string.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/range_image_visualizer.h>
 
int main () {

    pcl::PointCloud<pcl::PointXYZ> pointCloud;

    std::string filename = "/home/wanyel/contours/20221130/2022_11_30_16_37_29_911.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ>(filename, pointCloud);
 
    // Generate the data
    // for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    //     for (float z=-0.5f; z<=0.5f; z+=0.01f) {
    //         pcl::PointXYZ point;
    //         point.x = 2.0f - y;
    //         point.y = y;
    //         point.z = z;
    //         pointCloud.push_back(point);
    //     }
    // }
    // pointCloud.width = pointCloud.size();
    // pointCloud.height = 1;
 
    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
    float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;
  
    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
    std::cout << rangeImage << "\n";

    float* ranges = rangeImage.getRangesArray();
    unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeImage.width, rangeImage.height);
    pcl::io::saveRgbPNGFile("depth.png", rgb_image, rangeImage.width, rangeImage.height);
}