#include<iostream>
#include<cmath>
#include<queue>
#include<opencv2/opencv.hpp>
#include"buff.h"

using namespace std;
using namespace cv;

buff_detect::buff_detect() {
    FileStorage fs("out_camera_data_org.xml", FileStorage::READ); //此标定文件不是真实相机参数文件，故结果不准。
    fs["camera_matrix"] >> this->cameraMatrix;
    fs["distortion_coefficients"] >> this->distCoeffs;
}


void buff_detect::ROI_fun(cv::RotatedRect& n_center_rotRect, cv::Point2f& big_center, cv::Mat src, int& ROI_flag, double& len) { //ROI
    struct contours_S {
        int num;
        double s;
        bool operator < (const contours_S& a) const {
            return s < a.s;
        }
    } temp_S;
    Mat src_roi;
    vector<Mat> imgChannels;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    int structElementSize = 0, z = 0;
    double Light_Contour_Area = 0, width = 0, height = 0, area = 0;
    Point2f offset;
    if (ROI_flag) {
        Point2f roi_len = Point2f(1.2 * len / 2, 1.2 * len / 2); //之前用动态ROI会卡是因为输出ROI图像src_roi导致的
        offset = Point2f(big_center.x - roi_len.x, big_center.y - roi_len.y);
        if (offset.x >= 0 && offset.y >= 0 && (offset.x + 2 * roi_len.x) <= src.cols && (offset.y + 2 * roi_len.y) <= src.rows) {
            src_roi = src(Rect(offset.x, offset.y, 2 * roi_len.x, 2 * roi_len.y));
            //imshow("src_roi", src_roi); //那就不要输出roi图像
        } else {
            src_roi = src;
            ROI_flag = 0;
        }
    } else {
        src_roi = src;
    }
    split(src_roi, imgChannels);
    if (buff_color == 0) { //0红1蓝
        src_roi = imgChannels.at(2) - imgChannels.at(0);
        threshold(src_roi, src_roi, 100, 255, THRESH_BINARY); //二值化
    } else {
        src_roi = imgChannels.at(0) - imgChannels.at(2);
        threshold(src_roi, src_roi, 100, 255, THRESH_BINARY); //蓝色未调
    }

    structElementSize = 0;
    Mat element = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1), Point(structElementSize, structElementSize));
    dilate(src_roi, src_roi, element); //膨胀
    structElementSize = 3;
    element = getStructuringElement(MORPH_RECT, Size(2 * structElementSize + 1, 2 * structElementSize + 1), Point(structElementSize, structElementSize));
    morphologyEx(src_roi, src_roi, MORPH_CLOSE, element); //闭操作

    //**调试**
    cv::imshow("图像处理后图片", src_roi);
    //**调试**

    findContours(src_roi, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0)); //轮廓检测
    vector<RotatedRect> minRect(contours.size());
    if (ROI_flag) {
        priority_queue <contours_S> qu;
        for (register size_t i = 0; i < contours.size(); i++) {
            temp_S.num = i;
            temp_S.s = contourArea(contours[i]);
            qu.push(temp_S);
        }
        contours_S max_S = qu.top();
        minRect[max_S.num] = minAreaRect(Mat(contours[max_S.num]));
        big_center = minRect[max_S.num].center; //外轮廓中心big_center
        len = minRect[max_S.num].size.height > minRect[max_S.num].size.width ? minRect[max_S.num].size.height : minRect[max_S.num].size.width;
        z = hierarchy[max_S.num][2];
        if (z == -1 || hierarchy[z][0] != -1 || hierarchy[z][1] != -1) {
            //hierarchy包含4个整型数据，分别表示:后一个轮廓的序号、前一个轮廓的序号、子轮廓的序号、父轮廓的序号。
            //有子轮廓、子轮廓没有后一个轮廓、子轮廓没有前一个轮廓。
            ROI_flag = 0;
            return;
        }
        minRect[z] = minAreaRect(Mat(contours[z]));
        n_center_rotRect = minRect[z]; //当前坐标n_center

        //**调试**
        Point2f roi_len = Point2f(1.2 * len / 2, 1.2 * len / 2);
        rectangle(src, Rect(offset.x, offset.y, 2 * roi_len.x, 2 * roi_len.y), Scalar(255, 255, 255), 1, LINE_AA);
        //**调试**

        n_center_rotRect.center += offset, big_center += offset; //加上roi和原图的偏移offset
    } else {
        for (register size_t i = 0; i < contours.size(); i++) {
            Light_Contour_Area = contourArea(contours[i]);
            minRect[i] = minAreaRect(Mat(contours[i]));
            width = minRect[i].size.width;
            height = minRect[i].size.height;
            area = width * height;
            drawContours(src, contours, i, Scalar(255, 0, 0), 1, LINE_AA);
            if (area / Light_Contour_Area > 2) {
                big_center = minRect[i].center; //外轮廓中心big_center
                len = minRect[i].size.height > minRect[i].size.width ? minRect[i].size.height : minRect[i].size.width;
                z = hierarchy[i][2]; //子轮廓
                if (z == -1) continue;
                if (hierarchy[z][0] == -1 && hierarchy[z][1] == -1 && hierarchy[z][2] == -1) { //要击打的装甲板的边框条件
                    //hierarchy包含4个整型数据，分别表示:后一个轮廓的序号、前一个轮廓的序号、子轮廓的序号、父轮廓的序号。
                    //没有后一个轮廓、没有前一个轮廓、没有子轮廓。
                    minRect[z] = minAreaRect(Mat(contours[z]));
                    n_center_rotRect = minRect[z]; //当前坐标n_center
                    ROI_flag = 1;
                }
            }
        }
    }
    n_center = n_center_rotRect.center;
    n_angle = n_center_rotRect.angle;
    j[0] = j[1]; j[1] = j[2]; j[2] = n_center;
    //判断象限
    float w = n_center_rotRect.size.width;
    float h = n_center_rotRect.size.height;
    Point2f temp[4];
    n_center_rotRect.points(temp);
    if (w < h) { //w < h 一三象限
        Point2f p1 = (temp[0] + temp[1]) / 2;
        Point2f p2 = (temp[2] + temp[3]) / 2;
        float dis1 = getDistance(p1, big_center);
        float dis2 = getDistance(p2, big_center);
        if (dis1 < dis2) { //第一象限
            quadrant = 1;
        } else { //第三象限
            quadrant = 3;
        }
    } else { //w > h 二四象限
        Point2f p1 = (temp[0] + temp[3]) / 2;
        Point2f p2 = (temp[1] + temp[2]) / 2;
        float dis1 = getDistance(p1, big_center);
        float dis2 = getDistance(p2, big_center);
        if (dis1 < dis2) { //第二象限
            quadrant = 2;
        } else { //第四象限
            quadrant = 4;
        }
    }
}

void buff_detect::rot_judge() {
    /*j1_angle = j2_angle;
    j2_angle = n_angle;
    if (j1_angle == 0) return;
    if (j1_angle < j2_angle) f[0]++; else f[1]++; //角度是负的，角度越来越大是顺时针，越来越小是逆时针，0顺1逆。*/
    j1_center = j2_center;
    j2_center = n_center;
    if (j1_center == Point2f(0, 0)) return;
    if (quadrant == 1) {
        if (j2_center.x > j1_center.x && j2_center.y > j1_center.y) f[0]++;
        else f[1]++;
    } else if (quadrant == 2) {
        if (j2_center.x > j1_center.x && j2_center.y < j1_center.y) f[0]++;
        else f[1]++;
    } else if (quadrant == 3) {
        if (j2_center.x < j1_center.x && j2_center.y < j1_center.y) f[0]++;
        else f[1]++;
    } else if (quadrant == 4) {
        if (j2_center.x < j1_center.x && j2_center.y > j1_center.y) f[0]++;
        else f[1]++;
    }
}

void buff_detect::getTarget2dPoinstion() {
    Point2f vertices[4], temp[4];
    n_center_rotRect.points(temp);

    for (int i = 0; i < 4; i++) { //**调试**
        putText(src, to_string(i), temp[i], FONT_HERSHEY_COMPLEX, 0.6, Scalar(255, 255, 255), 1, LINE_AA);
    }

    if (quadrant == 1) vertices[0] = temp[1], vertices[1] = temp[0], vertices[2] = temp[3], vertices[3] = temp[2];
    if (quadrant == 2) vertices[0] = temp[0], vertices[1] = temp[3], vertices[2] = temp[2], vertices[3] = temp[1];
    if (quadrant == 3) vertices[0] = temp[3], vertices[1] = temp[2], vertices[2] = temp[1], vertices[3] = temp[0];
    if (quadrant == 4) vertices[0] = temp[2], vertices[1] = temp[1], vertices[2] = temp[0], vertices[3] = temp[3];

    for (int i = 0; i < 4; i++) { //**调试**
        putText(src, to_string(i), vertices[i] + Point2f(45, 45), FONT_HERSHEY_COMPLEX, 0.6, Scalar(255, 255, 255), 1, LINE_AA);
    }

    point2d.clear();
    point2d.emplace_back(vertices[0]);
    point2d.emplace_back(vertices[1]);
    point2d.emplace_back(vertices[2]);
    point2d.emplace_back(vertices[3]);
}

void buff_detect::solvePnP4Points() {
    if (width_target < 10e-5 || height_target < 10e-5) {
        rvec = cv::Mat::eye(3, 3, CV_64FC1);
        tvec = cv::Mat::zeros(3, 1, CV_64FC1);
    }
    std::vector<cv::Point3f> point3d;
    double half_x = width_target / 2.0;
    double half_y = height_target / 2.0;
    point3d.emplace_back(Point3f(-half_x, -half_y, 0));
    point3d.emplace_back(Point3f(half_x, -half_y, 0));
    point3d.emplace_back(Point3f(half_x, half_y, 0));
    point3d.emplace_back(Point3f(-half_x, half_y, 0));
    cv::Mat r;
    cv::solvePnP(point3d, point2d, cameraMatrix, distCoeffs, r, tvec);
    Rodrigues(r, rvec);
}

double buff_detect::getDistance(Point A, Point B) { //获取点间距离
    double dis;
    dis = pow((A.x - B.x), 2) + pow((A.y - B.y), 2);
    return sqrt(dis);
}

double buff_detect::getDistance(Mat A, Mat B) { //获取点间距离
    double dis;
    dis = pow((A.at<double>(0, 0) - B.at<double>(0, 0)), 2) + pow((A.at<double>(1, 0) - B.at<double>(1, 0)), 2)
        + pow((A.at<double>(2, 0) - B.at<double>(2, 0)), 2);
    return sqrt(dis);
}

void buff_detect::camera2world(Mat& point_in_camera, Mat& point_in_world, Mat rot, Mat tran) {
    point_in_world = rot.inv() * (point_in_camera - tran);
}

void buff_detect::world2camera() {
    cam_pre = rvec * nn_wor - tvec;
}

void buff_detect::getCamera2Ptz_r(double theta1, double theta2, Mat& camera2ptz_t_r) {
    theta1 = theta1 * CV_PI / 180;
    theta2 = theta2 * CV_PI / 180;
    double theta3;
    theta3 = atan(-tan(theta2) * cos(theta1));
    camera2ptz_t_r.at<double>(0, 0) = cos(theta1);
    camera2ptz_t_r.at<double>(0, 1) = -sin(theta2);
    camera2ptz_t_r.at<double>(0, 2) = 0;
    camera2ptz_t_r.at<double>(1, 0) = sin(theta3) * sin(theta1);
    camera2ptz_t_r.at<double>(1, 1) = cos(theta3);
    camera2ptz_t_r.at<double>(1, 2) = sin(theta3) * cos(theta1);
    camera2ptz_t_r.at<double>(2, 0) = cos(theta3) * sin(theta1);
    camera2ptz_t_r.at<double>(2, 1) = -sin(theta3);
    camera2ptz_t_r.at<double>(2, 2) = cos(theta3) * cos(theta1);
}

void buff_detect::camera2ptz(Mat& point_in_camera, Mat& point_in_ptz, Mat rot, Mat tran) {
    point_in_ptz = rot * (point_in_camera - tran);
}

void buff_detect::gravity_compensation(Mat& ptz_pre) {
    double d = sqrt(pow(ptz_pre.at<double>(0, 0), 2) + pow(ptz_pre.at<double>(2, 0), 2));
    //当子弹竖直速度降为0时刚好打中预测点
    double theta;
    double t;
    t = sqrt(2 * abs(ptz_pre.at<double>(1, 0)) / g);
    theta = acos(d / (v * t));
    ptz_pre.at<double>(1, 0) = -d * tan(theta);
}

void buff_detect::calculate_R(const std::vector<cv::Point2f> points, cv::Point2f& center, double& radius) { //拟合圆
    radius = 0.0f;
    double sum_x = 0.0f, sum_y = 0.0f;
    double sum_x2 = 0.0f, sum_y2 = 0.0f;
    double sum_x3 = 0.0f, sum_y3 = 0.0f;
    double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;
    int N = points.size();
    for (register int i = 0; i < N; i++) {
        double x = points[i].x, y = points[i].y;
        double x2 = x * x, y2 = y * y;
        sum_x += x, sum_y += y;
        sum_x2 += x2, sum_y2 += y2;
        sum_x3 += x2 * x, sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2, sum_x2y1 += x2 * y;
    }
    double C, D, E, G, H, a, b, c;
    C = N * sum_x2 - sum_x * sum_x;
    D = N * sum_xy - sum_x * sum_y;
    E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
    G = N * sum_y2 - sum_y * sum_y;
    H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;
    center.x = a / (-2);
    center.y = b / (-2);
    radius = sqrt(a * a + b * b - 4 * c) / 2;
}

void buff_detect::calculate_Ang() {
    if(big_armor)//大能量机关
        angle = ((-0.41667 * cos(1.884 * (t + t1)) + 1.305 * (t + t1)) - (-0.41667 * cos(1.884 * t) + 1.305 * t)); //计算t到t + t1的定积分
    else //小能量机关
        angle = w * t1;
    if (!rot_flag) { //rot_flag为0顺时针，为1逆时针
        //顺时针
        nn_wor = (cv::Mat_<double>(3, 1) << r_wor * cos(CV_PI / 2 - angle), r_wor * sin(CV_PI / 2 - angle) - r_wor, 0);
        //变换后的当前点坐标，加上公式角度计算
    } else {
        //逆时针
        nn_wor = (cv::Mat_<double>(3, 1) << r_wor * cos(CV_PI / 2 + angle), r_wor * sin(CV_PI / 2 + angle) - r_wor, 0);
        //变换后的当前点坐标，加上公式角度计算
    }
}
