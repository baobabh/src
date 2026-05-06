#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>  // capture_image는 Trigger 타입이라고 가정
#include <chrono>

class CameraSwitcher {
public:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::ServiceClient lane_cam_capture_, sign_cam_capture_;
    bool is_lane_active_;  // 현재 활성화된 카메라 추적

    CameraSwitcher(){
        is_lane_active_= true;
        // 서비스가 준비될 때까지 대기
        ros::Duration wait_time(1.0);
        while (!ros::service::waitForService("camera/lane_cam/grab_image", wait_time)) {
            ROS_WARN("Waiting for camera/lane_cam/capture_image service...");
        }
        while (!ros::service::waitForService("camera/sign_cam/grab_image", wait_time)) {
            ROS_WARN("Waiting for camera/sign_cam/capture_image service...");
        }

        // 서비스 클라이언트 생성
        lane_cam_capture_ = nh_.serviceClient<std_srvs::Trigger>("camera/lane_cam/grab_image");
        sign_cam_capture_ = nh_.serviceClient<std_srvs::Trigger>("camera/sign_cam/grab_image");

        // 0.5초 주기로 카메라 전환
        timer_ = nh_.createTimer(ros::Duration(0.5), &CameraSwitcher::switchCamera, this);
    }

    void switchCamera(const ros::TimerEvent&) {
       std_srvs::Trigger trigger;
        if (is_lane_active_) {
            if (sign_cam_capture_.call(trigger)){
                ros::Time now = ros::Time::now();
                ROS_INFO("Sign Cam captured image. time: %.4f second", now.toSec());
            }
        } else {
            if (lane_cam_capture_.call(trigger)){
                ros::Time now = ros::Time::now();
                ROS_INFO("Lane Cam captured image. time: %.4f seoncd", now.toSec());
            }
        }
        is_lane_active_ = !is_lane_active_;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_switcher");
    CameraSwitcher switcher;
    ros::spin();
    return 0;
}
