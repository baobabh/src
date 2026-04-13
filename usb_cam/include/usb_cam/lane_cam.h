#ifndef LANE_CAM_H
#define LANE_CAM_H

#include <Eigen/Dense>
#include <cmath>
#include <string>
#include <opencv2/core.hpp>

#include "usb_cam/usb_cam.h"
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>


struct CircleFit {
  double xc;   // 원 중심 x
  double yc;   // 원 중심 y
  double r;    // 반지름
  bool ok;     // 추정 성공 여부
  CircleFit() : xc(0.0), yc(0.0), r(0.0), ok(false) {}
};

class LaneCam : public usb_cam::UsbCam {
public:
    static LaneCam& Instance(ros::NodeHandle& nh);
    LaneCam(ros::NodeHandle& nh);
private:
    ros::Publisher pub_lane_;
    ros::Publisher pub_detect_lane_comp_;
    void timer_callback(const ros::TimerEvent&);
    void detect_lane(const sensor_msgs::Image& img);
    cv::Mat fn_lane_threshold(const cv::Mat& image, int val_threshold = 200);
    Eigen::Vector2d pixelToWorld(double u, double v) const;
    std::pair<std::string, cv::Point> decideLaneDirection(const cv::Mat& binaryImage) const;
    std::vector<cv::Point> extractLanePoints1(const cv::Mat& binaryImage, const std::string& direction, int tol = 2) const;
    std::vector<cv::Point> extractLanePoints2(const cv::Mat& binaryImage, const std::string& direction, int tol = 2) const;
    CircleFit estimateCircle(const std::vector<cv::Point2d>& pts);

    //검출하는 점의 갯수
    int point_num;
    // 카메라 파라미터들
    double res_x, res_y;
    double cx, cy;
    double fov_h, fov_v;
    double fx, fy;
    double h_cam;
    double roll_cam;

    double d;
    double pi;

    Eigen::Matrix3d R_axes;
    Eigen::Matrix3d R_roll;
    Eigen::Matrix3d R_cb;
    Eigen::Matrix3d R_cnv;
    Eigen::Matrix3d R_inv;

    void initCameraParams();  // 초기화 함수
};
#endif
