#include<iostream>
#include"buff.h"
//#define debugging 1 //1开启输出调试信息

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

    while (buff.capture.read(buff.src)) {
#if debugging
        tt = clock();
#endif
        buff.t += buff.wait; //当前时间

        if (buff.count_num > 100) { //能量机关击打程序退出条件，n_center连续三次位置不变
            if (buff.j[0] == buff.j[1] && buff.j[1] == buff.j[2]) break;
        }

        buff.ROI_fun(buff.n_center_rotRect, buff.big_center, buff.src, buff.ROI_flag, buff.len); //图像处理及ROI更新
        if (buff.ROI_flag == 0) continue; //判断是否找到ROI区域，如果没找到就跳过本次循环进入下一次循环。

        if(buff.pointss.size() <= 40 + 12) buff.pointss.emplace_back(buff.n_center); //前多个点存入pointss用于计算圆心
        if (buff.pointss.size() > 40 && buff.count_num <= 12) { //存入多个点后
            buff.calculate_R(buff.pointss, buff.c_center, buff.r_cam); //计算圆心
        }


#if debugging
        circle(buff.src, buff.n_center, 4, Scalar(0, 255, 255), 2, LINE_AA); //绘制n_center
        circle(buff.src, buff.c_center, 4, Scalar(255, 0, 0), 2, LINE_AA); //绘制c_center
        string O = '(' + to_string(buff.c_center.x) + "," + to_string(buff.c_center.y) + ')'; //标记圆心
        putText(buff.src, "R" + O, buff.c_center - Point2f(95, 0), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, LINE_AA);
#endif


        if (buff.pointss.size() > 40 && buff.count_num <= 12) { //旋转方向判断
            buff.rot_judge();
            buff.count_num++;
        } else if (buff.count_num > 12) {
            buff.calculate_Ang(); //转动预测及自定义坐标系

            buff.getTarget2dPoinstion(); //修改point2d的四个点

            buff.solvePnP4Points(); //solvePnP

            buff.world2camera(); //坐标系转换

            //buff.getCamera2Ptz_r(buff.yaw, buff.pitch, buff.camera2ptz_r);
            //buff.camera2ptz(buff.cam_pre, buff.ptz_pre, buff.camera2ptz_r, buff.camera2ptz_t);

            buff.gravity_compensation(buff.cam_pre); //重力补偿

            //角度计算
            buff.yaw = atan(buff.cam_pre.at<double>(0, 0) / buff.cam_pre.at<double>(2, 0));
            buff.pitch = -atan(buff.cam_pre.at<double>(1, 0) / buff.cam_pre.at<double>(2, 0));
            buff.yaw *= 180 / CV_PI;
            buff.pitch *= 180 / CV_PI;
#if debugging
            //cout << "yaw pitch" << endl;
            //cout << buff.yaw << " " << buff.pitch << endl;
            putText(buff.src, "    yaw         pitch", Point(80, 200), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, LINE_AA);
            putText(buff.src, to_string(buff.yaw) + " " + to_string(buff.pitch), Point(80, 220), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, LINE_AA);
#endif
        }



        if (buff.count_num == 12) {//记录多次结果
            if (buff.f[0] > buff.f[1]) { //判断顺逆时针哪个出现的多
                buff.rot_flag = 0; //顺
            } else {
                buff.rot_flag = 1; //逆
            }
#if debugging
            if(!buff.rot_flag) cout << "顺" << endl; else cout << "逆" << endl;
#endif
            buff.count_num++; //num自加一保证接下来不会进入
        }

#if debugging
        suum += (long long int(clock()) - tt);
        cnt++;
        //cout << double(suum) / double(cnt) << "ms" << endl;
        putText(buff.src, to_string(double(suum) / double(cnt)) + "ms", Point(80, 150), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1, LINE_AA);
        cv::imshow("src", buff.src);
#endif

        cv::waitKey(1);
    }
    return 0;
}
