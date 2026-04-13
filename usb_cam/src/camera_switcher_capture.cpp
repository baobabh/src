#include <ros/ros.h>
#include <std_msgs/Empty.h>

class CameraSwitcher {
public:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Publisher lane_cam_trigger_pub_;
    ros::Publisher sign_cam_trigger_pub_;
    bool is_lane_active_;

    CameraSwitcher(){
        is_lane_active_ = true;

        // 토픽 퍼블리셔 생성
        lane_cam_trigger_pub_ = nh_.advertise<std_msgs::Empty>("lane_cam/lane_cam/snap_trigger", 1);
        sign_cam_trigger_pub_ = nh_.advertise<std_msgs::Empty>("sign_cam/sign_cam/snap_trigger", 1);

        // 0.25초 주기로 카메라 전환
        timer_ = nh_.createTimer(ros::Duration(0.25), &CameraSwitcher::switchCamera, this);
    }

    void switchCamera(const ros::TimerEvent&) {
        std_msgs::Empty trigger;

        if (is_lane_active_) {
            sign_cam_trigger_pub_.publish(trigger);
            ROS_INFO("Sign Cam Trigger Publish.");
        } else {
            lane_cam_trigger_pub_.publish(trigger);
            ROS_INFO("Lane Cam Trigger Publish.");
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
