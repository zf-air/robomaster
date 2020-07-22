#include<iostream>
#include"buff.h"
#define debugging 0 //1开启输出调试信息

using namespace std;
using namespace cv;

#if debugging
clock_t tt; long long int suum = 0; int cnt = 0;
#endif

int main() {
    buff_detect buff;

    buff.src = buff.capture.open("red.avi");
    buff.rate = buff.capture.get(CAP_PROP_FPS);
    buff.wait = 1 / buff.rate;

    while(buff.capture.read(buff.src)){
        imshow("src",buff.src);
        waitKey(1);
    }
//    while (buff.capture.read(buff.src)) {
//#if debugging
//        tt = clock();
//#endif
//        buff.t += buff.wait; //当前时间

//        buff.ROI_fun(buff.n_center_rotRect, buff.big_center, buff.src, buff.ROI_flag, buff.len);
//        buff.n_center = buff.n_center_rotRect.center;
//        buff.pointss.push_back(buff.n_center);
//        buff.calculate_R(buff.pointss, buff.c_center, buff.r_cam);
//#if debugging
//        circle(buff.src, buff.n_center, 4, Scalar(0, 0, 255), 2, LINE_AA); //绘制n_center
//        circle(buff.src, buff.c_center, 4, Scalar(255, 0, 0), 2, LINE_AA); //绘制c_center
//#endif
//        buff.quadrant_judge(buff.n_center, buff.c_center, buff.quadrant);

//        if (buff.count_num <= 5) { //旋转方向判断
//            buff.quadrantAndrot_judge(buff.j1_center, buff.j2_center, buff.n_center, buff.quadrant, &buff.f[0]);
//            buff.count_num++;
//        } else if (buff.count_num > 5) {
//            buff.calculate_Ang(buff.angle, buff.rot_flag, buff.nn_wor, buff.r_wor, buff.t, buff.t1);

//            buff.getTarget2dPoinstion(buff.n_center_rotRect, buff.point2d, buff.quadrant); //修改point2d的四个点

//            buff.solvePnP4Points(buff.point2d, buff.rvec, buff.tvec); //solvePnP后相机三维坐标系有疑问

//            buff.world2camera(buff.nn_wor, buff.cam_pre, buff.rvec, buff.tvec);

//            buff.getCamera2Ptz_r(buff.yaw, buff.pitch, buff.camera2ptz_r);

//            buff.camera2ptz(buff.cam_pre, buff.ptz_pre, buff.camera2ptz_r, buff.camera2ptz_t);

//            //重力补偿
//            buff.gravity_compensation(buff.v, buff.ptz_pre);

//            //计算角度
//            buff.yaw = atan(buff.ptz_pre.at<double>(0, 0) / buff.ptz_pre.at<double>(2, 0));
//            buff.pitch = -atan(buff.ptz_pre.at<double>(1, 0) / buff.ptz_pre.at<double>(2, 0));
//            buff.yaw *= 180 / CV_PI;
//            buff.pitch *= 180 / CV_PI;
//#if debugging
//            cout << "yaw pitch" << endl;
//            cout << buff.yaw << " " << buff.pitch << endl;
//#endif
//        }

//        if (buff.count_num == 5) {//记录5次结果
//            if (buff.f[0] > buff.f[1]) { //判断顺逆时针哪个出现的多
//                buff.rot_flag = 0; //顺
//            } else {
//                buff.rot_flag = 1; //逆
//            }
//            buff.count_num++; //num自加一保证接下来不会进入
//        }

//        if (buff.pointss.size() > 5) { //能量机关击打程序退出条件，n_center连续三次位置不变
//            if ((buff.pointss[buff.pointss.size() - 1] == buff.pointss[buff.pointss.size() - 2]) && (buff.pointss[buff.pointss.size() - 2] == buff.pointss[buff.pointss.size() - 3])) {
//                break;
//            }
//        }
//#if debugging
//        suum += (long long int(clock()) - tt);
//        cnt++;
//        cout << double(suum) / double(cnt) << "ms" << endl;
//        cv::imshow("src", buff.src);
//#endif
//        cv::waitKey(1);
//    }
    cv::waitKey(1);
    return 0;
}
