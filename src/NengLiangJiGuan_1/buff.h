#ifndef BUFF_H
#define BUFF_H

#pragma once
#include<opencv2/opencv.hpp>

//这些在.h里也要写
using namespace std;
using namespace cv;

class buff_detect {

private:
    bool buff_color = 0; //0红，1蓝
    Mat cameraMatrix, distCoeffs;
    const double width_target = 400, height_target = 200; //装甲板真实的宽和高mm，此处目前不是真实值
    const double camera2platformY = 50, camera2platformZ = 500; //摄像头到云台的真实距离mm， 此处目前不是真实值

public:
    const double g = 9.7966; //郑州的重力加速度，详见https://wenku.baidu.com/view/f4c2112c0066f5335a81211b.html
    const double w = 1.0;
    const double v = 21000; //子弹速度mm/s
    double yaw = 0, pitch = 0;
    Mat camera2ptz_t = (cv::Mat_<double>(3, 1) << 0, -camera2platformY, camera2platformZ);
    Mat camera2ptz_r = Mat::zeros(3, 3, CV_64FC1);
    VideoCapture capture;
    Mat src, nn_wor;
    double rate, wait;
    RotatedRect n_center_rotRect;
    Point2f n_center, big_center, c_center, j1_center, j2_center;
    vector<Point2f> pointss, point2d;
    int ROI_flag = 0, quadrant = 0, count_num = 0, f[2] = { 0 };
    double r = 0, t = 0, t1 = 0.1, angle = 0, r_cam = 0, len = 0, r_wor = 1500;
    //此处r_wor为现实中风车的半径，即圆心到装甲板中心点的距离，1500mm
    bool rot_flag;
    Mat rvec, tvec, cam_pre, ptz_pre;
    //相机坐标系下预测点坐标 //云台坐标系下预测点坐标

private:
    double getDistance(Point A, Point B);
    double getDistance(Mat A, Mat B);

public:
    buff_detect();
    void ROI_fun(cv::RotatedRect& n_center_rotRect, cv::Point2f& big_center, cv::Mat src, int& ROI_flag, double& len);
    void calculate_R(const std::vector<cv::Point2f> points, cv::Point2f& center, double& radius);
    void quadrantAndrot_judge(Point2f& j1_center, Point2f& j2_center, Point2f& n_center, int& quadrant, int* f);
    void getTarget2dPoinstion(const cv::RotatedRect& rect, vector<Point2f>& target2d, int quadrant);
    void solvePnP4Points(const std::vector<cv::Point2f>& points2d, cv::Mat& rot, cv::Mat& trans);
    void camera2world(Mat& point_in_camera, Mat& point_in_world, Mat rot, Mat tran);
    void world2camera(Mat& point_in_world, Mat& point_in_camera, Mat rot, Mat tran);
    void getCamera2Ptz_r(double theta1, double theta2, Mat& camera2ptz_t_r);
    void camera2ptz(Mat& point_in_camera, Mat& point_in_ptz, Mat rot, Mat tran);
    void gravity_compensation(double v, Mat& ptz_pre);
    void calculate_Ang(double& angle, bool rot_flag, Mat& nn_wor, double r_wor, double& t, double& t1);
    void quadrant_judge(Point2f n_center, Point2f c_center, int& quadrant);
};
#endif // BUFF_H
