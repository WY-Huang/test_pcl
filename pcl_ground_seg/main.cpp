#include "ground_seg.hpp"


int main(int argc, char** argv)
{

    string filename = "/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_sample_100_filter.ply";
    Ground_segment ground_segment(filename);

    // pcl::PCDWriter writer;
    // writer.write<pcl::PointXYZ>("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_sample_100_filter.pcd", *cloud, false);

}

