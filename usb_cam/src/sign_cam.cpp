/*
#include "usb_cam/sign_cam.h"


SignCam& SignCam::Instance(ros::NodeHandle& nh) 
{
    static SignCam instance(nh);  // 이제 ros::NodeHandle을 넘겨줍니다
    return instance;
}

SignCam::SignCam(ros::NodeHandle& nh) : usb_cam::UsbCam()  // 부모 생성자 호출
{
    // 생성자 부분.. topic 정의 부분..
    timer_ = nh.createTimer(ros::Duration(0.5), &SignCam::timer_callback, this);
}
void SignCam::timer_callback(const ros::TimerEvent&)
{
    // image 를 Timer로 가져오는 부분. 여기서 grab_image로 이미지 가져와서 detect_sign(*img_msg)로 전달!
    grab_image(img_msg);
    if (img_msg->data.empty()) {
        ROS_WARN("SignCam: empty image");
        return;
    }
    // detect_sign(*img_msg);
}

// void SignCam::detect_sign(const sensor_msgs::Image& msg)
// {
   
// }
*/


#include "usb_cam/sign_cam.h"
#include <map>


static std::map<std::string, int> ECP_SIGN_STATE = {
    {"none", 0},
    {"stop", 1},
    {"tunnel", 2}
};


SignCam& SignCam::Instance(ros::NodeHandle& nh) 
{
    static SignCam instance(nh);  // 이제 ros::NodeHandle을 넘겨줍니다
    return instance;
}

SignCam::SignCam(ros::NodeHandle& nh) : usb_cam::UsbCam()  // 부모 생성자 호출
{
    // Debug flag
    is_debug_mode_ = nh.param("is_debug_mode", false);
    publish_image_ = nh.param("publish_image", true);

    // 차단바 초기화
    checkLevelCross_ = 1;
    prev_crossbar_detect_ = false;
    levelcross_done_ = 1;

    lower_red_crossbar_ = cv::Scalar(0, 131, 68);
    upper_red_crossbar_ = cv::Scalar(188, 255, 255);
    min_w_ = 20;
    max_w_ = 100;
    min_h_ = 20;
    max_h_ = 100;
    min_r_ = 0.5;
    max_r_ = 1.5;
    kernel_ = cv::Mat::ones(11, 11, CV_8U);

    //sub_check_level_cross_ = nh.subscribe("/check/levelcross", 1, &SignCam::cbGetCheckLevelCross, this);
    pub_detect_level_cross_ = nh.advertise<sensor_msgs::Image>("/detect/levelcross/compressed", 1);
    pub_detect_level_cross_bar_ = nh.advertise<std_msgs::Bool>("/detect/levelcrossbar", 1);

    /*
    if (is_debug_mode_) {
        srv_image_crossbar_ = new dynamic_reconfigure::Server<ecp_preproc::LevelCrossParamsConfig>;
        // 서버 콜백 등록 필요
    }*/

    // 신호등 초기화
    checkTraffic_ = 1;
    green_detect_count_ = 0;
    traffic_done_ = 0;

    lower_red_traffic_ = cv::Scalar(0, 131, 68);
    upper_red_traffic_ = cv::Scalar(188, 255, 255);
    lower_yellow_ = cv::Scalar(30, 10, 80);
    upper_yellow_ = cv::Scalar(50, 10, 80);
    lower_green_ = cv::Scalar(82, 120, 50);
    upper_green_ = cv::Scalar(95, 255, 255);
    x_left_ = 0.75;
    x_right_ = 0.875;
    y_top_ = 0.75;
    y_bottom_ = 0.22;
    x_min_ = 45;
    x_dif_ = 35;
    y_min_ = 70;
    y_dif_ = 25;
    kernel3_ = cv::Mat::ones(3, 3, CV_8U);

    //sub_check_traffic_ = nh.subscribe("/check/traffic", 1, &SignCam::cbGetCheckTraffic, this);
    pub_detect_traffic_ = nh.advertise<sensor_msgs::Image>("/detect/traffic/compressed", 1);
    pub_detect_traffic_signal_ = nh.advertise<std_msgs::UInt8>("/detect/traffic_signal", 1);

    /*
    if (is_debug_mode_) {
        srv_image_traffic_ = new dynamic_reconfigure::Server<ecp_preproc::TrafficParamsConfig>;
        // 콜백 연결 필요
    }*/

    // 표지판 초기화
    
    prev_traffic_sign_ = 0;
    traffic_sign_count_ = 0;
    sign_done_ = 1;
    stop_count_ = 0;
    tunnel_count_ = 0;

    HSV_RED_LOWER_ = cv::Scalar(0, 100, 50);
    HSV_RED_UPPER_ = cv::Scalar(10, 255, 255);
    HSV_RED_LOWER1_ = cv::Scalar(160, 100, 50);
    HSV_RED_UPPER1_ = cv::Scalar(179, 255, 255);
    HSV_YELLOW_LOWER_ = cv::Scalar(10, 80, 50);
    HSV_YELLOW_UPPER_ = cv::Scalar(40, 255, 255);
    HSV_BLUE_LOWER_ = cv::Scalar(80, 160, 50);
    HSV_BLUE_UPPER_ = cv::Scalar(140, 255, 255);

    loadModels();

    pub_traffic_ = nh.advertise<std_msgs::UInt8>("/detect/traffic", 1);
    pub_viz_ = nh.advertise<sensor_msgs::Image>("/HOG_KNN/detections_image_topic/compressed", 1);
    pub_viz_1_ = nh.advertise<sensor_msgs::Image>("/TEST/compressed", 1);

    /*
    if (is_debug_mode_) {
        srv_image_sign_ = new dynamic_reconfigure::Server<ecp_preproc::DetectSignConfig>;
        // 콜백 연결 필요
    }*/

    // sub_image_ = nh.subscribe("/sign_cam/sign_image_raw/compressed", 1, &SignCam::cbImage, this);

    ROS_INFO("Unified sign detector node started.");

    // 생성자 부분.. topic 정의 부분..
    timer_ = nh.createTimer(ros::Duration(0.5), &SignCam::timer_callback, this);
    
    /*
    if (is_debug_mode_) {
        srv_image_crossbar_ = new dynamic_reconfigure::Server<ecp_preproc::LevelCrossParamsConfig>;
        srv_image_crossbar_->setCallback(boost::bind(&SignCam::cbGetLevelCrossParams, this, _1, _2));

        srv_image_traffic_ = new dynamic_reconfigure::Server<ecp_preproc::TrafficParamsConfig>;
        srv_image_traffic_->setCallback(boost::bind(&SignCam::cbGetTrafficParams, this, _1, _2));

        srv_image_sign_ = new dynamic_reconfigure::Server<ecp_preproc::DetectSignConfig>;
        srv_image_sign_->setCallback(boost::bind(&SignCam::cbGetSignParams, this, _1, _2));
    }*/
} 


void SignCam::timer_callback(const ros::TimerEvent&)
{
    // image 를 Timer로 가져오는 부분. 여기서 grab_image로 이미지 가져와서 unified_sign_detect(*img_msg)로 전달!
    grab_image(img_msg);
    ROS_INFO("Grab Image!");
    if (img_msg->data.empty()) {
        ROS_WARN("SignCam: empty image");
        return;
    }
    unified_sign_detect(*img_msg);
    //ROS_INFO("I receive the Image !!");
}


void SignCam::unified_sign_detect(const sensor_msgs::Image& img)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_image = cv_ptr -> image;

    if (cv_image.empty()){
        ROS_WARN("unified_sign_detector: empty image");
        return;
        }
 
    cv::rotate(cv_image, cv_image, cv::ROTATE_180);
    // 로그 출력
    ROS_INFO("[State, 0: Run 1:Stop] Traffic: %d, Sign: %d, LevelCross: %d",
            traffic_done_, sign_done_, levelcross_done_);

    // FSM 흐름
    if (sign_done_ == 2) {
        ROS_INFO("All Detections COMPLETED");
        return;
    }

    if (traffic_done_ == 0) {
        detect_traffic(cv_image);
        return;
    }
    
    if (sign_done_ == 0) {
        detect_sign(cv_image);
    }

    if (levelcross_done_ == 0) {
        detect_levelcross(cv_image);
    }
}


void SignCam::detect_levelcross(const cv::Mat& img)
{
    if (levelcross_done_ == 1) return;

    // 타이머 시작
    double LevelCross_T1 = ros::Time::now().toSec();
    double LevelCross_T2 = 0.0, LevelCross_T3 = 0.0, LevelCross_T4 = 0.0;

    int count = 0;
    crossbar_detect_ = false;

    // 이미지 복사 및 ROI 설정
    cv::Mat img_ori = img.clone();
    int img_height = img_ori.rows;
    int img_width = img_ori.cols;

    cv::Mat img_levelcrossROI = img_ori(cv::Rect(img_width / 6, 0, img_width * 5 / 6, img_height)).clone();
    cv::Mat img_hsv;
    cv::cvtColor(img_levelcrossROI, img_hsv, cv::COLOR_BGR2HSV);

    // HSV 필터 - 빨간색
    cv::Scalar lower_red = lower_red_crossbar_;
    cv::Scalar upper_red = upper_red_crossbar_;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 11));

    cv::Mat img_mask_r;
    cv::inRange(img_hsv, lower_red, upper_red, img_mask_r);
    cv::morphologyEx(img_mask_r, img_mask_r, cv::MORPH_OPEN, kernel);

    cv::Mat img_red;
    cv::bitwise_and(img_levelcrossROI, img_levelcrossROI, img_red, img_mask_r);

    LevelCross_T2 = ros::Time::now().toSec();

    // 컨투어 검출
    std::vector<std::vector<cv::Point>> list_contour;
    cv::findContours(img_mask_r, list_contour, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : list_contour) {
        cv::drawContours(img_red, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(255, 0, 0), 2);
        cv::Rect rect = cv::boundingRect(contour);
        int w = rect.width;
        int h = rect.height;
        float aspect_ratio = static_cast<float>(w) / static_cast<float>(h);

        cv::putText(img_red, "w: " + std::to_string(w) + ", h: " + std::to_string(h) +
                    ", aspect_ratio: " + std::to_string(aspect_ratio),
                    cv::Point(rect.x + 2, rect.y + h + 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 255), 1);

        if (w > min_w_ && w < max_w_ && h > min_h_ && h < max_h_ &&
            aspect_ratio > min_r_ && aspect_ratio < max_r_) {
            cv::drawContours(img_red, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
            count++;
        }
    }

    if (count >= 3) {
        crossbar_detect_ = true;
        LevelCross_T3 = ros::Time::now().toSec();
    }

    if (publish_image_) {
            cv_bridge::CvImage out;
            out.header.stamp = ros::Time::now();
            out.encoding = sensor_msgs::image_encodings::BGR8;
            out.image = img_red;
            pub_detect_level_cross_.publish(out.toImageMsg());
        }

    // 상태 업데이트
    if (prev_crossbar_detect_ == true && crossbar_detect_ == false) {
        ROS_INFO("CROSSBAR ELIMINATED");
        prev_crossbar_detect_ = false;
        checkLevelCross_ = false;
        levelcross_done_ = 1;
        sign_done_ = 0;
    }

    // Bool 메시지 전송
    std_msgs::Bool msg_crossbar_detect_;
    msg_crossbar_detect_.data = crossbar_detect_;
    pub_detect_level_cross_bar_.publish(msg_crossbar_detect_);

    prev_crossbar_detect_ = crossbar_detect_;
    ROS_INFO("[CROSSBAR] prev: %d, curr: %d", prev_crossbar_detect_, crossbar_detect_);
    LevelCross_T4 = ros::Time::now().toSec();

    // 시간 로깅
    if (LevelCross_T2 != 0.0 && LevelCross_T3 != 0.0 && LevelCross_T4 != 0.0) {
        ROS_INFO("[LevelCross] Preproc: %.4f, Inference: %.4f, Publish: %.4f",
                 LevelCross_T2 - LevelCross_T1,
                 LevelCross_T3 - LevelCross_T2,
                 LevelCross_T4 - LevelCross_T3);
    }
}

void SignCam::detect_traffic(const cv::Mat& img) {
    if (traffic_done_ == 1 ){
        ROS_INFO("Traffic is done.");
        return;
    }

    ros::Time T1 = ros::Time::now();

    // ROI 설정
    cv::Mat img_trafficROI = img(cv::Range(img.rows * 1 / 9, img.rows * 3 / 4),
                                 cv::Range(img.cols * 5 / 8, img.cols * 7 / 8)).clone();

    ros::Time T2 = ros::Time::now();
    ros::Time T3;
    
    std::tie(green_sign_detect_, T3) = fn_traffic_count_fixed_light(img_trafficROI);

    // publish
    if (checkTraffic_ == 1) {
        std_msgs::UInt8 msg;
        msg.data = green_sign_detect_;
        pub_detect_traffic_signal_.publish(msg);
        //ROS_INFO("PUb Traffic Signal");
    }

    if (green_sign_detect_ == 1) {
        traffic_done_ = 1;
        sign_done_ = 0;
    }

    ros::Time T4 = ros::Time::now();
    double preproc_time = (T2 - T1).toSec();
    double inference_time = (T3 - T2).toSec();  // 이미 포함
    double publish_time = (T4 - T2).toSec();

    ROS_INFO("[TRAFFIC] Preproc: %.4f, Inference: %.4f, Publish: %.4f",
             preproc_time, inference_time, publish_time);

}


std::pair<int, ros::Time> SignCam::fn_traffic_count_fixed_light(const cv::Mat& img_trafficROI) {
    cv::Mat img_gray;
    cv::cvtColor(img_trafficROI, img_gray, cv::COLOR_BGR2GRAY);

    cv::Mat img_light;
    cv::threshold(img_gray, img_light, 100, 255, cv::THRESH_BINARY);

    ros::Time dummy1, dummy2;
    cv::Mat img_debug = img_trafficROI.clone();
    std::tie(list_light_, img_debug) = fn_tracking_traffic(img_trafficROI, img_light, list_light_, 5, 5);

    // ROI 정의
    int red_cx_min = x_min_ + 5;
    int red_cx_max = x_min_ + x_dif_ + 5;
    int red_cy_min = y_min_ - 10;
    int red_cy_max = y_min_ + y_dif_ - 5;

    int yellow_cx_min = x_min_ + 3;
    int yellow_cx_max = x_min_ + x_dif_ + 3;
    int yellow_cy_min = y_min_ + y_dif_ - 5;
    int yellow_cy_max = y_min_ + 2 * y_dif_;

    int green_cx_min = x_min_;
    int green_cx_max = x_min_ + x_dif_;
    int green_cy_min = y_min_ + 2 * y_dif_;
    int green_cy_max = y_min_ + 3 * y_dif_;


    cv::rectangle(img_debug, cv::Point(red_cx_min, red_cy_min), cv::Point(red_cx_max, red_cy_max), cv::Scalar(127, 127, 255), 1);
    cv::rectangle(img_debug, cv::Point(yellow_cx_min, yellow_cy_min), cv::Point(yellow_cx_max, yellow_cy_max), cv::Scalar(127, 255, 255), 1);
    cv::rectangle(img_debug, cv::Point(green_cx_min, green_cy_min), cv::Point(green_cx_max, green_cy_max), cv::Scalar(127, 255, 127), 1);

    // threshold 설정
    double threshold_red = 180;
    double threshold_yellow = 180;
    double threshold_green = 180;

    bool green_on = false;

    for (const auto& light : list_light_) {
        int count = light[0];
        float cx = light[2];
        float cy = light[3];
        int x0 = light[5];
        int y0 = light[6];
        int w = light[7];
        int h = light[8];

        cv::Rect roi_rect(x0, y0, w, h);
        cv::Mat roi = img_gray(roi_rect);
        double avg_brightness = cv::mean(roi)[0];

        cv::rectangle(img_debug, roi_rect, cv::Scalar(0, 0, 127), 1);
        cv::putText(img_debug, std::to_string(count) + "/" + std::to_string(int(avg_brightness)), 
                    cv::Point(x0, y0 + 9), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, cv::Scalar(255, 0, 0));

        //Red light
        if (cx > red_cx_min && cx < red_cx_max && cy > red_cy_min && cy < red_cy_max) {
            //ROS_INFO("RED is Detected");
            if (avg_brightness > threshold_red) {
                ROS_WARN("Red avg brightness: %.2f", avg_brightness);
                cv::rectangle(img_debug, roi_rect, cv::Scalar(0, 0, 255), 1);
            }
        }

        //Yellow light
        if (cx > yellow_cx_min && cx < yellow_cx_max && cy > yellow_cy_min && cy < yellow_cy_max) {
            //ROS_INFO("YELLOW is Detected");
            if (avg_brightness > threshold_yellow) {
                ROS_WARN("Yellow avg brightness: %.2f", avg_brightness);
                cv::rectangle(img_debug, roi_rect, cv::Scalar(0, 255, 255), 1);
            }
        }

        //Green light
        if (cx > green_cx_min && cx < green_cx_max && cy > green_cy_min && cy < green_cy_max) {
            ROS_INFO("GREEN is Detected");
            if (avg_brightness > threshold_green) {
                ROS_WARN("Green avg brightness: %.2f", avg_brightness);
                green_on = true;
                ROS_INFO("Green Light detected");
                cv::rectangle(img_debug, roi_rect, cv::Scalar(0, 255, 0), 1);
            }
        }
    }
    ros::Time Traffic_T0 = ros::Time::now();
    
    if (publish_image_) {
        cv_bridge::CvImage out;
        out.header.stamp = ros::Time::now();
        out.encoding = sensor_msgs::image_encodings::BGR8;
        out.image = img_debug;
        pub_detect_traffic_.publish(out.toImageMsg());
    }

    if (green_on) 
        green_detect_count_++;
    else green_detect_count_ = 0;

    if (green_detect_count_ > 3) {
        ros::Time Traffic_T1 = ros::Time::now();
        ROS_INFO("Green Light Publish Starts(Count %d)", green_detect_count_);
        return std::make_pair(1, Traffic_T1);
    }
    return std::make_pair(0, Traffic_T0);
}


float SignCam::fn_cal_distance(const cv::Point2f& pt1, const cv::Point2f& pt2) {
    return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
}

std::vector<std::vector<float>> SignCam::fn_tracking_find_contour(const cv::Mat& img_mask) {
    int length_min = 5, length_max = 20;
    float aspect_ratio_limit = 1.5;
    float area_lower_limit = 0.1;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img_mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<float>> list_all_obj;

    for (const auto& contour : contours) {
        cv::Rect rect = cv::boundingRect(contour);
        int w = rect.width;
        int h = rect.height;
        float aspect_ratio = static_cast<float>(w) / h;

        cv::Moments moment = cv::moments(contour);
        float area = moment.m00;
        float area_rect = static_cast<float>(w * h);
        float area_ratio = area / area_rect;

        if (w > length_min && h > length_min &&
            aspect_ratio > 1.0f / aspect_ratio_limit && aspect_ratio < aspect_ratio_limit &&
            area_ratio > area_lower_limit) {

            float cx = static_cast<float>(moment.m10 / moment.m00);
            float cy = static_cast<float>(moment.m01 / moment.m00);
            list_all_obj.push_back({cx, cy, area, static_cast<float>(rect.x), static_cast<float>(rect.y), static_cast<float>(w), static_cast<float>(h)});
        }
    }
    return list_all_obj;
}

std::pair<std::vector<std::vector<float>>, cv::Mat> SignCam::fn_tracking_traffic(
    const cv::Mat& img_trafficROI, const cv::Mat& img_mask,
    std::vector<std::vector<float>>& traffic_obj,
    int min_dis_limit = 15, int miss_limit = 5)
{
    // 형태학적 연산
    cv::Mat img_morphology;
    cv::morphologyEx(img_mask, img_morphology, cv::MORPH_OPEN, kernel3_);
    cv::Mat img_debug;
    cv::bitwise_and(img_trafficROI, img_trafficROI, img_debug, img_morphology);

    // 컨투어 기반 객체 탐색
    std::vector<std::vector<float>> list_all_obj = fn_tracking_find_contour(img_mask);

    if (!list_all_obj.empty()) {
        std::vector<bool> flag_continue_obj(traffic_obj.size(), false);
        std::vector<bool> flag_continue_obj_new(list_all_obj.size(), false);

        std::vector<std::vector<float>> arr_dis(list_all_obj.size(), std::vector<float>(traffic_obj.size(), 10000));
        std::vector<float> list_min_dis(list_all_obj.size(), 10000);

        for (size_t ii_new = 0; ii_new < list_all_obj.size(); ++ii_new) {
            cv::Point2f pt_new(list_all_obj[ii_new][0], list_all_obj[ii_new][1]);
            for (size_t ii = 0; ii < traffic_obj.size(); ++ii) {
                cv::Point2f pt_old(traffic_obj[ii][2], traffic_obj[ii][3]);
                arr_dis[ii_new][ii] = fn_cal_distance(pt_new, pt_old);
            }
        }

        while (true) {
            for (size_t ii_new = 0; ii_new < list_all_obj.size(); ++ii_new) {
                if (!arr_dis[ii_new].empty()) {
                    list_min_dis[ii_new] = *std::min_element(arr_dis[ii_new].begin(), arr_dis[ii_new].end());
                }
            }

            auto final_it = std::min_element(list_min_dis.begin(), list_min_dis.end());
            if (*final_it > min_dis_limit)
                break;

            int idx_obj_new = std::distance(list_min_dis.begin(), final_it);
            int idx_obj = std::distance(arr_dis[idx_obj_new].begin(),
                                        std::min_element(arr_dis[idx_obj_new].begin(), arr_dis[idx_obj_new].end()));

            std::fill(arr_dis[idx_obj_new].begin(), arr_dis[idx_obj_new].end(), 10000);
            for (auto& row : arr_dis)
                row[idx_obj] = 10000;

            flag_continue_obj[idx_obj] = true;
            flag_continue_obj_new[idx_obj_new] = true;

            traffic_obj[idx_obj][0] += 1;  // count
            traffic_obj[idx_obj][1] = 0;   // miss
            std::copy(list_all_obj[idx_obj_new].begin(), list_all_obj[idx_obj_new].end(), traffic_obj[idx_obj].begin() + 2);
        }

        // miss 증가
        for (size_t ii = 0; ii < traffic_obj.size(); ++ii) {
            if (!flag_continue_obj[ii])
                traffic_obj[ii][1] += 1;
        }

        // remove missed
        traffic_obj.erase(std::remove_if(traffic_obj.begin(), traffic_obj.end(),
                            [=](const std::vector<float>& obj) { return obj[1] >= miss_limit; }),
                          traffic_obj.end());

        // 새 객체 추가
        for (size_t ii_new = 0; ii_new < list_all_obj.size(); ++ii_new) {
            if (!flag_continue_obj_new[ii_new]) {
                std::vector<float> new_obj = {1, 0};
                new_obj.insert(new_obj.end(), list_all_obj[ii_new].begin(), list_all_obj[ii_new].end());
                traffic_obj.push_back(new_obj);
            }
        }
    }

    return std::make_pair(traffic_obj, img_debug);
}







void SignCam::detect_sign(const cv::Mat& img) {
    if (sign_done_ == 1)
        return;

    ros::Time T1 = ros::Time::now(), T2, T3, T4;

    ecp_preproc::BoundingBox detection_result = signDetecting(img, T2, T3);

    if (!detection_result.Class.empty()) {
        std::string label = detection_result.Class;
        int current_traffic_sign = ECP_SIGN_STATE[label];
        ROS_WARN("Detect Sign : %s", label.c_str());

        if (prev_traffic_sign_ == current_traffic_sign) {
            traffic_sign_count_++;
        } else {
            prev_traffic_sign_ = current_traffic_sign;
            traffic_sign_count_ = 0;
        }

        if (traffic_sign_count_ >= 1) {
            std_msgs::UInt8 msg;
            msg.data = current_traffic_sign;
            pub_traffic_.publish(msg);
            T4 = ros::Time::now();

            // FSM 처리
            if (label == "stop") {
                stop_count_++;
                if (stop_count_ >= 3) {
                    sign_done_ = 1;
                    levelcross_done_ = 0;
                    stop_count_ = 0;
                    ROS_INFO("[STOP] detected - Levelcross starts");
                }
            } else if (label == "tunnel") {
                tunnel_count_++;
                if (tunnel_count_ >= 3) {
                    sign_done_ = 2;
                    ROS_INFO("[TUNNEL] detected - Sign detect stops");
                }
            }

            if (publish_image_) {
            visualizeAndPublish(detection_result, img);
            }
        }
    }

    if ((T2 != ros::Time(0)) && (T3 != ros::Time(0)) && (T4 != ros::Time(0))) {
        ROS_INFO("[Sign] Preproc: %.4f, Inference: %.4f, Publish: %.4f",
                 (T2 - T1).toSec(), (T3 - T2).toSec(), (T4 - T3).toSec());
    }

    // 시각화 생략 또는 추후 추가
}

ecp_preproc::BoundingBox SignCam::signDetecting(const cv::Mat& img, ros::Time& t2, ros::Time& t3) {
    ecp_preproc::BoundingBox detection_result;
    detection_result.Class = "";

    // 1. HSV 필터
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat redBinary, redBinary1, yellowBinary, blueBinary, binary;
    cv::inRange(hsv, HSV_RED_LOWER_, HSV_RED_UPPER_, redBinary);
    cv::inRange(hsv, HSV_RED_LOWER1_, HSV_RED_UPPER1_, redBinary1);
    redBinary = redBinary | redBinary1;

    cv::inRange(hsv, HSV_YELLOW_LOWER_, HSV_YELLOW_UPPER_, yellowBinary);
    cv::inRange(hsv, HSV_BLUE_LOWER_, HSV_BLUE_UPPER_, blueBinary);

    binary = redBinary | blueBinary;
    cv::Mat binary_filled = binary.clone();

    // 2. Morphology & Contour    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_filled, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& cnt : contours) {
        cv::drawContours(binary_filled, std::vector<std::vector<cv::Point>>{cnt}, -1, cv::Scalar(255), cv::FILLED);
    }

    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::bitwise_and(binary_filled, gray, gray);  // ROI 강조

    // 디버그용 gray 퍼블리시
    if (pub_viz_1_.getNumSubscribers() > 0) {
        cv_bridge::CvImage out;
        out.header.stamp = ros::Time::now();
        out.encoding = sensor_msgs::image_encodings::MONO8;
        out.image = gray;
        pub_viz_1_.publish(out.toImageMsg());
    }

    t2 = ros::Time::now();

    std::vector<std::vector<cv::Point>> goodContours;
    cv::findContours(binary_filled, goodContours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    static const std::vector<std::string> CLASS_NAMES = {
        "construction", "forbid", "intersection", "left", "parking", "right", "stop", "tunnel"
    };

    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area > 800.0) {
            cv::Rect rect = cv::boundingRect(cnt);
            float aspect_ratio = static_cast<float>(rect.width) / rect.height;

            if (aspect_ratio >= 0.8 && aspect_ratio <= 1.2) {
                cv::Mat inputImage = img(rect).clone();
                int roi_size = std::max(inputImage.rows, inputImage.cols);

                int model_size = choose_best_model_size(roi_size);
                ROS_INFO(">>>ROI size = %d, using model size = %d", roi_size, model_size);
                cv::resize(inputImage, inputImage, cv::Size(model_size, model_size));
                cv::cvtColor(inputImage, inputImage, cv::COLOR_BGR2GRAY);

                // 3. HOG feature 설정
                cv::Size win(model_size, model_size);
                cv::Size block, stride, cell;
                int nbins = 9;

                switch (model_size) { 
                case 16:    block = cv::Size(8, 8);     stride = cv::Size(4, 4);    cell = cv::Size(4, 4);  break;
                case 32:    block = cv::Size(16, 16);   stride = cv::Size(8, 8);    cell = cv::Size(8, 8);  break;
                case 64:    block = cv::Size(16, 16);   stride = cv::Size(8, 8);    cell = cv::Size(8, 8);  break;
                default:    block = cv::Size(24, 24);   stride = cv::Size(8, 8);    cell = cv::Size(12, 12);break;
                }

                cv::HOGDescriptor hog(win, block, stride, cell, nbins);

                std::vector<float> descriptors;
                hog.compute(inputImage, descriptors);

                // 4. KNN 분류
                ros::Time infer_start = ros::Time::now();
                auto model_ptr = static_cast<cv::Ptr<cv::ml::KNearest>>(model_dict_[model_size]);
                cv::Mat descriptor_mat = cv::Mat(descriptors).clone().reshape(1, 1);
                descriptor_mat.convertTo(descriptor_mat, CV_32F);

                //ROS_INFO("Descriptor size = %lu, mat cols = %d", descriptors.size(), descriptor_mat.cols);
                //ROS_INFO("Descriptor type = %d (expect 5 = CV_32F)", descriptor_mat.type());

                cv::Mat result;
                model_ptr->findNearest(descriptor_mat, 1, result);
                ros::Time infer_end = ros::Time::now();
                t3 = infer_end;

                int predicted_label_idx = static_cast<int>(result.at<float>(0, 0));
                std::string class_label = CLASS_NAMES[predicted_label_idx];

                // 5. BoundingBox 생성
                detection_result.xmin = rect.x;
                detection_result.xmax = rect.x + rect.width;
                detection_result.ymin = rect.y;
                detection_result.ymax = rect.y + rect.height;
                detection_result.Class = class_label;
                detection_result.probability = 1.0;

                /*
                if (pub_viz_.getNumSubscribers() > 0) {
                    cv::Mat imgOut = img.clone();
                    cv::rectangle(imgOut, rect, cv::Scalar(200, 152, 50), 2);
                    cv::putText(imgOut, class_label, cv::Point(rect.x, rect.y + 20), cv::FONT_HERSHEY_SIMPLEX,
                                0.6, cv::Scalar(255, 255, 255), 2);
                    cv_bridge::CvImage out;
                    out.header.stamp = ros::Time::now();
                    out.encoding = sensor_msgs::image_encodings::BGR8;
                    out.image = imgOut;

                    pub_viz_.publish(out.toImageMsg());
                }*/

                return detection_result;  // 첫 번째 감지된 결과만 사용
            }
        }
    }

    t3 = ros::Time::now();
    return detection_result;
}

int SignCam::choose_best_model_size(int roi_size) {
    if (roi_size <= 16) return 16;
    if (roi_size <= 32) return 32;
    if (roi_size <= 64) return 64;
    return 128;
}

void SignCam::loadModels() {
    std::string base_path = "/home/nano/catkin_ws/src/2023-Autorace/ecp_preproc/models/";  // 필요 시 경로 수정
    std::vector<int> model_sizes = {16, 32, 64, 128};

    for (int size : model_sizes) {
        std::string filename;
        if (size == 128)
            filename = "ext_HOG_seperated_gray.yml";
        else
            filename = "ext_HOG_seperated" + std::to_string(size) + "x" + std::to_string(size) + "_gray.yml";

        std::string full_path = base_path + filename;
        cv::Ptr<cv::ml::KNearest> knn = cv::Algorithm::load<cv::ml::KNearest>(full_path);
        if (!knn->empty()) {
            model_dict_[size] = knn;
            ROS_INFO("[Model Load] %dx%d model loaded", size, size);
        } else {
            ROS_WARN("[Model Load] Failed to load model %dx%d", size, size);
        }
    }
}

void SignCam::visualizeAndPublish(const ecp_preproc::BoundingBox& output, const cv::Mat& img) {
    if (output.Class.empty()) return;

    // 이미지 복사
    cv::Mat imgOut = img.clone();
    std::string label = output.Class;

    // Bounding box 좌표
    int x_p1 = static_cast<int>(output.xmin);
    int y_p1 = static_cast<int>(output.ymin);
    int x_p3 = static_cast<int>(output.xmax);
    int y_p3 = static_cast<int>(output.ymax);

    // 사각형 그리기
    cv::rectangle(imgOut, cv::Point(x_p1, y_p1), cv::Point(x_p3, y_p3), cv::Scalar(200, 152, 50), 2);

    // 텍스트 그리기
    int font = cv::FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.6;
    int thickness = 2;
    cv::putText(imgOut, label, cv::Point(x_p1, y_p1 + 20), font, fontScale, cv::Scalar(255, 255, 255), thickness);

    // 이미지 publish
    cv_bridge::CvImage out;
    out.header.stamp = ros::Time::now();
    out.encoding = sensor_msgs::image_encodings::BGR8;
    out.image = imgOut;

    pub_viz_.publish(out.toImageMsg());
}


/*
void SignCam::cbGetLevelCrossParams(ecp_preproc::LevelCrossParamsConfig &config, uint32_t level) {
    lower_red_crossbar_ = cv::Scalar(config.hue_red_l, config.saturation_red_l, config.lightness_red_l);
    upper_red_crossbar_ = cv::Scalar(config.hue_red_h, config.saturation_red_h, config.lightness_red_h);

    min_w_ = config.min_w;
    max_w_ = config.max_w;
    min_h_ = config.min_h;
    max_h_ = config.max_h;
    min_r_ = config.min_r;
    max_r_ = config.max_r;
}

void SignCam::cbGetTrafficParams(ecp_preproc::TrafficParamsConfig &config, uint32_t level) {
    lower_red_traffic_ = cv::Scalar(config.hue_red_l, config.saturation_red_l, config.lightness_red_l);
    upper_red_traffic_ = cv::Scalar(config.hue_red_h, config.saturation_red_h, config.lightness_red_h);

    lower_yellow_ = cv::Scalar(config.hue_yellow_l, config.saturation_yellow_l, config.lightness_yellow_l);
    upper_yellow_ = cv::Scalar(config.hue_yellow_h, config.saturation_yellow_h, config.lightness_yellow_h);

    lower_green_ = cv::Scalar(config.hue_green_l, config.saturation_green_l, config.lightness_green_l);
    upper_green_ = cv::Scalar(config.hue_green_h, config.saturation_green_h, config.lightness_green_h);

    x_left_ = config.x_left;
    x_right_ = config.x_right;
    y_top_ = config.y_top;
    y_bottom_ = config.y_bottom;
    x_min_ = config.x_min;
    x_dif_ = config.x_dif;
    y_min_ = config.y_min;
    y_dif_ = config.y_dif;
}

void SignCam::cbGetSignParams(ecp_preproc::DetectSignConfig &config, uint32_t level) {
    HSV_RED_LOWER_ = cv::Scalar(config.hue_red_l, config.saturation_red_l, config.lightness_red_l);
    HSV_RED_UPPER_ = cv::Scalar(config.hue_red_h, config.saturation_red_h, config.lightness_red_h);

    HSV_RED_LOWER1_ = cv::Scalar(config.hue_red1_l, config.saturation_red1_l, config.lightness_red1_l);
    HSV_RED_UPPER1_ = cv::Scalar(config.hue_red1_h, config.saturation_red1_h, config.lightness_red1_h);

    HSV_YELLOW_LOWER_ = cv::Scalar(config.hue_yellow_l, config.saturation_yellow_l, config.lightness_yellow_l);
    HSV_YELLOW_UPPER_ = cv::Scalar(config.hue_yellow_h, config.saturation_yellow_h, config.lightness_yellow_h);

    HSV_BLUE_LOWER_ = cv::Scalar(config.hue_blue_l, config.saturation_blue_l, config.lightness_blue_l);
    HSV_BLUE_UPPER_ = cv::Scalar(config.hue_blue_h, config.saturation_blue_h, config.lightness_blue_h);
}

void SignCam::cbGetCheckLevelCross(const std_msgs::UInt8::ConstPtr& msg) {
    checkLevelCross_ = msg -> data;
}

void SignCam::cbGetCheckTraffic(const std_msgs::UInt8::ConstPtr& msg){
    checkTraffic_ = msg -> data;
}*/