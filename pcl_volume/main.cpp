#include "pcl_volume.hpp"


int main(int argc, char** argv)
{
    // // 球体体积及表面积
    // double r = 1.0;
    // double vol_std = 4 * M_PI * r * r * r / 3;
    // double area_std = 4 * M_PI * r * r;

    // // 椭球体体积
    // double vol_std_spheroid = 4 * M_PI * 5 * 4 * 3 / 3;

    // 梯形台尺寸：66.61*38.85 20.33*20.28 19.50    V=[S1+S2+√(S1*S2)]*h/3 
    double vol_std_prism = (62.33*39.34 + 17.8*19.28 + sqrt(62.33*39.34 + 17.8*19.28)) * 19.80 / 3;
    cout << "梯形台理论体积：" << vol_std_prism << endl;

    // cout << "半径为1的球体标准体积：" << vol_std << endl;
    // cout << "半径为1的球体标准表面积：" << area_std << endl;
    // cout << "半轴长为3，4，5的椭球体标准体积：" << vol_std_spheroid << endl;
    // cout << "===============================" << endl;
    // // volume_demo();
    // // cout << "===============================" << endl;
    // volume_cal(vol_std, area_std);

    convex_volume_cal();


}