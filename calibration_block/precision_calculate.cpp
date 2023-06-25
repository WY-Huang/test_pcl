#include <string.h>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include <thread>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PointIndices.h>

using namespace std;


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

// 保存pcd点云
void save_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, string save_path)
{
    pcl::PCDWriter writer;
        
    writer.write(save_path, *cloud_out, false);
    cout << "Save segment file success -> " << save_path << endl;

    cout << "The number of save point clouds: " << cloud_out->points.size() << endl;
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
        else if (seg_method == "yz_seg")
        {
            if (z0 < 15 && y0 > 36 && y0 < 62)   //  分割阈值,  y:36-62,
            {
                pointIdxVec.push_back(i);
            }
        }
        else if (seg_method == "z_seg")
        {
            if (z0 < 0)   //  分割阈值,  && z0 < -61.44
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


// 计算vector的均值、方差、标准差，最值
float calculateStandardDeviation(const std::vector<float>& data) 
{
    int n = data.size();
    cout << "=======================================================" << endl;
    cout << "ContourNums: " << n << endl;

    float sum = 0.0, mean, variance = 0.0;

    // 计算最大值和最小值
    float maxValue = *max_element(data.begin(), data.end());
    float minValue = *min_element(data.begin(), data.end());
    cout << "maxValue: " << maxValue << " mm" << endl;
    cout << "minValue: " << minValue << " mm" << endl;

    // 计算vector元素的总和
    for (float val : data) {
        sum += val;
    }

    // 计算均值
    mean = sum / n;
    cout << "Mean: " << mean << " mm" << endl;

    // 计算方差
    for (float val : data) {
        variance += pow(val - mean, 2);
    }
    variance /= n;
    cout << "variance: " << variance << " mm" << endl;

    // 计算标准差
    float standardDeviation = sqrt(variance);
    cout << "standardDeviation: " << standardDeviation << " mm" << endl;

    return standardDeviation;
}

// 对所有轮廓进行z均值计算
void contour_line_mean_cal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    vector<float> mean_all_line;

    float temp_value_start = 10000.1;
    float temp_value_end = 10000.2;
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        float x0 = cloud_in->points[i].x;
        if (x0 != temp_value_start)
        {
            temp_value_end = temp_value_start;
            temp_value_start = x0;
            // 单个轮廓的处理操作
            if (i != 0)     
            {
                float z_value = 0.0;
                for (int k = 0; k < basic_cloud_ptr->points.size(); ++k)
                {
                    z_value += basic_cloud_ptr->points[k].z;
                }
                mean_all_line.push_back(z_value / basic_cloud_ptr->points.size());

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
    // 均值及标准差计算
    float mean_all = calculateStandardDeviation(mean_all_line);

}

// 点云拼接
void cloud_concatenate()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);
    string file_a = "/home/wanyel/contours/Calibration_block/precision_calculate/2023_05_12_09_20_15_819_seg.pcd";
    read_cloud(cloud_a, file_a);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
    string file_b = "/home/wanyel/contours/Calibration_block/precision_calculate/2023_05_12_09_22_41_592_seg.pcd";
    read_cloud(cloud_b, file_b);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_c(new pcl::PointCloud<pcl::PointXYZ>);
    string file_c = "/home/wanyel/contours/Calibration_block/precision_calculate/2023_05_12_09_24_08_293_seg.pcd";
    read_cloud(cloud_c, file_c);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZ>);
    string file_all = "/home/wanyel/contours/Calibration_block/precision_calculate/2023_05_12_10_44_35_cloud_all.pcd";

    *cloud_all = (*cloud_a) + (*cloud_b) + (*cloud_c);
    save_cloud(cloud_all, file_all);
    
}

// 两个直线点云的距离计算（逐点计算均值）
void cloud_z_mean(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b)
{
    std::vector<float> DisZ;
    for (int i = 0; i < cloud_a->points.size(); ++i)
    {
        for (int k = 0; k < cloud_b->points.size(); ++k)
        {
            float dis_z = cloud_b->points[k].z - cloud_a->points[i].z;
            DisZ.push_back(dis_z);
        }
    }

    float standardDeviation = calculateStandardDeviation(DisZ);
}

// 对所有直线轮廓进行长度计算，按X方向的轮廓，计算轮廓起始点和终点间的距离
void straight_line_length_cal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    vector<float> diff_all_line;

    float temp_value_start = 10000.1;
    float temp_value_end = 10000.2;
    for (int i = 0; i < cloud_in->points.size(); ++i)
    {
        float x0 = cloud_in->points[i].x;
        if (x0 != temp_value_start)
        {
            temp_value_end = temp_value_start;
            temp_value_start = x0;
            // 单个轮廓的处理操作
            if (i != 0)     
            {
                // float y_begin = basic_cloud_ptr->begin()->y;
                // float y_end = (basic_cloud_ptr->end()-1)->y;
                float y_value = basic_cloud_ptr->begin()->y - (basic_cloud_ptr->end()-1)->y;
                diff_all_line.push_back(y_value);
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
    // 均值及标准差计算
    float mean_all = calculateStandardDeviation(diff_all_line);

}

int main()
{
    // 读取原始点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);
    string file_a = "/home/wanyel/USER_DATA/2023_06_25_10_33_34_798_30mm.pcd";
    read_cloud(cloud_a, file_a);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b(new pcl::PointCloud<pcl::PointXYZ>);
    string file_b = "/home/wanyel/USER_DATA/2023_06_25_10_33_34_798_60mm.pcd";
    read_cloud(cloud_b, file_b);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZ>);
    // *cloud_all = (*cloud_a) + (*cloud_b);
    // string file_all = "/home/wanyel/contours/Calibration_block/precision_calculate/2023_05_12_merge.pcd";
    // save_cloud(cloud_all, file_all);

    // 点云直通滤波
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_seg(new pcl::PointCloud<pcl::PointXYZ>);
    // string save_seg = "/home/wanyel/contours/Calibration_block/precision_calculate/2023_05_12_10_44_35_683_Zseg.pcd";
    // // segment_cloud(cloud, cloud_seg, save_seg, "z_seg");
    // read_cloud(cloud_seg, save_seg);

    // contour_line_mean_cal(cloud_seg);

    // 点云合并
    // cloud_concatenate();

    // 计算两个直线点云的平均距离
    // cloud_z_mean(cloud_a, cloud_b);

    // Y向平均长度
    // straight_line_length_cal(cloud_a);
    // straight_line_length_cal(cloud_b);

    // z均值计算
    contour_line_mean_cal(cloud_a);
    contour_line_mean_cal(cloud_b);
}
