#ifndef CPP_LANE_LINE_DETECATION_H
#define CPP_LANE_LINE_DETECATION_H

#include <iostream>
#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std::chrono_literals;
using namespace std;
using namespace cv;

class LaneLineDetection : public rclcpp::Node
{

public:
    LaneLineDetection();
    ~LaneLineDetection(){};

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_image_pub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    string image_sub_topic_name;
    string res_dir;

    Mat camera_matirx; //内参矩阵，需初始化
    Mat dist_coeffs;   //畸变矩阵，需初始化
    Mat M;             //透视变换矩阵
    Mat Minv;          //逆透视变换矩阵
    Mat left_curve_img;
    Mat right_curve_img;
    Mat keep_straight_img;

private:
    void loadParam();
    void initNode();
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void detectionForward(const Mat &src, Mat &dst);
    void get_M_Minv();
    void labSelect(const Mat &src, Mat &dst, const char &channel = 'b',
                   const int &thresh_min = 0, const int &thresh_max = 255);
    void hlsSelect(const Mat &src, Mat &dst, const char &channel = 'l',
                   const int &thresh_min = 0, const int &thresh_max = 255);
    void thresholdForword(Mat img, Mat &dst);
    Mat polyfit(vector<Point> &in_point, int n);
    vector<Point> polyval(const Mat &mat_k, const vector<Point> &src, int n);
    void findLine(const Mat &src, vector<Point> &lp,
                  vector<Point> &rp, int &rightx_current,
                  int &leftx_current, double &vehicle_offset,
                  double &curvature);
    void drawArea(const Mat &src, Mat &dst,
                  vector<Point> &lp, vector<Point> &rp,
                  const Mat &Minv, double &vehicle_offset, 
                  double &curvature);
};

#endif //CPP_LANE_LINE_DETECATION_H