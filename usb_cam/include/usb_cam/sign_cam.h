/*
#ifndef SIGN_CAM_H
#define SIGN_CAM_H

#include "usb_cam/usb_cam.h"
#include <sensor_msgs/Image.h>

class SignCam : public usb_cam::UsbCam {
public:
    static SignCam& Instance(ros::NodeHandle& nh);
    SignCam(ros::NodeHandle& nh);
private:
    void timer_callback(const ros::TimerEvent&);
    
};
#endif
*/

#ifndef SIGN_CAM_H
#define SIGN_CAM_H

#include "usb_cam/usb_cam.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/ml.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <dynamic_reconfigure/server.h>


/*/
#include <ecp_preproc/LevelCrossParamsConfig.h>
#include <ecp_preproc/TrafficParamsConfig.h>
#include <ecp_preproc/DetectSignConfig.h>
*/
#include <ecp_preproc/BoundingBox.h>

#include <map>

class SignCam : public usb_cam::UsbCam {
public:
    static SignCam& Instance(ros::NodeHandle& nh);
    SignCam(ros::NodeHandle& nh);
private:
    void timer_callback(const ros::TimerEvent&);
    void unified_sign_detect(const sensor_msgs::Image& img);
    //void cbGetLevelCrossParams(ecp_preproc::LevelCrossParamsConfig &config, uint32_t level);
    //void cbGetTrafficParams(ecp_preproc::TrafficParamsConfig &config, uint32_t level);
    //void cbGetSignParams(ecp_preproc::DetectSignConfig &config, uint32_t level);

    void detect_traffic(const cv::Mat& img);
    std::pair<int, ros::Time> fn_traffic_count_fixed_light(const cv::Mat& img_trafficROI);
    // 유틸 함수들
    float fn_cal_distance(const cv::Point2f& pt1, const cv::Point2f& pt2);
    std::vector<std::vector<float>> fn_tracking_find_contour(const cv::Mat& img_mask);
    std::pair<std::vector<std::vector<float>>, cv::Mat> fn_tracking_traffic(
        const cv::Mat& img_trafficROI, const cv::Mat& img_mask,
        std::vector<std::vector<float>>& traffic_obj,
        int min_dis_limit, int miss_limit);

    //void cbGetCheckLevelCross(const std_msgs::UInt8::ConstPtr& msg);
    //void cbGetCheckTraffic(const std_msgs::UInt8::ConstPtr& msg);

    void detect_levelcross(const cv::Mat& img);

    void detect_sign(const cv::Mat& img);
    ecp_preproc::BoundingBox signDetecting(const cv::Mat& img, ros::Time& t2, ros::Time& t3);
    void loadModels();
    int choose_best_model_size(int roi_size);
    static const std::vector<std::string> CLASS_NAMES;
    std::map<int, cv::Ptr<cv::ml::KNearest>> model_dict_;
    void visualizeAndPublish(const ecp_preproc::BoundingBox& output, const cv::Mat& img);

    // Debug
    bool is_debug_mode_;
    bool publish_image_;

    
    
    // 차단바 관련
    uint8_t checkLevelCross_;
    bool crossbar_detect_;
    bool prev_crossbar_detect_;
    int levelcross_done_;
    cv::Scalar lower_red_crossbar_, upper_red_crossbar_;
    cv::Scalar lower_red_traffic_, upper_red_traffic_;
    int min_w_, max_w_, min_h_, max_h_;
    float min_r_, max_r_;
    cv::Mat kernel_;

    ros::Subscriber sub_check_level_cross_;
    ros::Publisher pub_detect_level_cross_;
    ros::Publisher pub_detect_level_cross_bar_;
    //dynamic_reconfigure::Server<ecp_preproc::LevelCrossParamsConfig> *srv_image_crossbar_;

    // 신호등 관련
    uint8_t checkTraffic_;
    int green_detect_count_;
    int green_sign_detect_;
    cv::Mat cv_image_;
    int traffic_done_;

    cv::Scalar lower_yellow_, upper_yellow_, lower_green_, upper_green_;
    float x_left_, x_right_, y_top_, y_bottom_;
    int x_min_, x_dif_, y_min_, y_dif_;
    cv::Mat kernel3_;

    std::vector<std::vector<float>> list_light_;
    std::vector<std::vector<float>> list_red_, list_yellow_, list_green_;

    ros::Subscriber sub_check_traffic_;
    ros::Publisher pub_detect_traffic_;
    ros::Publisher pub_detect_traffic_signal_;
    //dynamic_reconfigure::Server<ecp_preproc::TrafficParamsConfig> *srv_image_traffic_;

    // 표지판 관련
    int prev_traffic_sign_;
    int traffic_sign_count_;
    int sign_done_;
    int stop_count_, tunnel_count_;

    cv::Scalar HSV_RED_LOWER_, HSV_RED_UPPER_;
    cv::Scalar HSV_RED_LOWER1_, HSV_RED_UPPER1_;
    cv::Scalar HSV_YELLOW_LOWER_, HSV_YELLOW_UPPER_;
    cv::Scalar HSV_BLUE_LOWER_, HSV_BLUE_UPPER_;

    ros::Publisher pub_traffic_;
    ros::Publisher pub_viz_, pub_viz_1_;
    //dynamic_reconfigure::Server<ecp_preproc::DetectSignConfig> *srv_image_sign_;

    ros::Subscriber sub_image_;
    ros::Timer timer_;
};
#endif