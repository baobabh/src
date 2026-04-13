#include "usb_cam/lane_cam.h"
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


LaneCam& LaneCam::Instance(ros::NodeHandle& nh) 
{
    static LaneCam instance(nh);  // 이제 ros::NodeHandle을 넘겨줍니다
    return instance;
}

LaneCam::LaneCam(ros::NodeHandle& nh) : usb_cam::UsbCam()  // 부모 생성자 호출
{
    pub_lane_ = nh.advertise<std_msgs::Float32MultiArray>("/detect/lane", 1);
    pub_detect_lane_comp_ = nh.advertise<sensor_msgs::compressedImage>("/detect/lane/compressed",1);
    timer_ = nh.createTimer(ros::Duration(0.5), &LaneCam::timer_callback, this);
    initCameraParams();
    point_num = 5;
}
void LaneCam::timer_callback(const ros::TimerEvent&)
{
    grab_image(img_msg);
    if (img_msg->data.empty()) {
        ROS_WARN("LaneCam: empty image");
        return;
    }
    detect_lane(*img_msg);
}

void LaneCam::detect_lane(const sensor_msgs::Image& msg)
{
    ROS_INFO("detect_lane called");
    ROS_INFO("Image Size: %d x %d", msg.width, msg.height);
    
    std_msgs::Float32MultiArray lane_msg;
    lane_msg.data.push_back(123.456f);
    lane_msg.data.push_back(7.89f);

    pub_lane_.publish(lane_msg);
    ROS_INFO("Pub lane info");

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge 예외: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;

    // 2. 이진화
    cv::Mat binary = fn_lane_threshold(image);

    // 3. 방향 판단
    auto res = decideLaneDirection(binary);
    const std::string& direction = res.first;
    const cv::Point& point = res.second;

    if (direction == "exception") {
        ROS_WARN("방향 판단 실패");
        return;
    }

    const int mode = 1;

    // 4. 점 추출
    std::vector<cv::Point> pixelPoints;
    if(mode == 0){
        pixelPoints = extractLanePoints1(binary, direction);
    } else (
        pixelPoints = extractLanePoints2(binary, direction);
    )

    if (pixelPoints.size() != point_num) {
        ROS_WARN("대표 점 부족 (%d 개 미만)", point_num);
        return;
    }


    //점들 시각화 이미지 발행->함수화 하기
    cv::Mat vis = image.clone();
    for (const auto& p : pixelPoints) {
        cv::circle(vis, p, 5, cv::Scalar(0, 0, 255), -1); // 빨간 점
    }
    // 보조 정보도 덧붙이면 현장 디버깅에 좋음
    cv::putText(vis, direction, cv::Point(10, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);

    // OpenCV → JPEG 바이트
    std::vector<uchar> buf;
    cv::imencode(".jpg", vis, buf);

    sensor_msgs::CompressedImage out;
    out.header   = msg.header;         // 원본 타임스탬프 그대로
    out.format   = "jpeg";
    out.data.assign(buf.begin(), buf.end());
    pub_detect_lane_comp_.publish(out);

    // 5. 픽셀 → 로봇 좌표계 변환
    std::vector<double> Xb_pos, Yb_pos;

    for (const auto& pt : pixelPoints) {
        Eigen::Vector2d pos = pixelToWorld(pt.x, pt.y);
        Xb_pos.push_back(pos(0));
        Yb_pos.push_back(pos(1));
    }


    if(mode == 1){
        std::vector<cv::Point2d> pts;
        pts.reserve(std::min(Xb_pos.size(), Yb_pos.size()));
        for (size_t i = 0; i < Xb_pos.size() && i < Yb_pos.size(); ++i) {
            pts.push_back(cv::Point2d(Xb_pos[i], Yb_pos[i]));
        }

        CircleFit cf = estimateCircle(pts);
        if (cf.ok) {
            // d = | sqrt(Xc^2 + Yc^2) - r |
            const double rho = std::sqrt(cf.xc*cf.xc + cf.yc*cf.yc);
            d   = std::fabs(rho - cf.r);

            // 로봇 -> 원 중심 방향 벡터의 수직 벡터 = 접선 방향
            // tx = -Yc, ty = Xc
            const double tx = -cf.yc;
            const double ty =  cf.xc;

            // pi = atan2(tx, ty)
            pi = std::atan2(tx, ty);

            // 디버그 출력
            ROS_INFO_STREAM("[CIRCLE] xc=" << cf.xc << " yc=" << cf.yc
                             << " r=" << cf.r << " d=" << d << " pi=" << pi);
        } else {
            ROS_WARN("[CIRCLE] 원 추정 실패(점이 부족하거나 수치 불안정)");
        }
        //return; // 원형 모드 테스트만 할 때는 조기 반환
    } else {
        // 6. 최소제곱법 직선 피팅 (Yb = aXb + b)
        Eigen::MatrixXd X(point_num, 2);
        Eigen::VectorXd y(point_num);

        for (int i = 0; i < point_num; ++i) {
            X(i, 0) = Xb_pos[i];  // X
            X(i, 1) = 1.0;        // bias term
            y(i) = Yb_pos[i];     // Y
        }

        Eigen::Vector2d params = (X.transpose() * X).ldlt().solve(X.transpose() * y);
        double a = params(0);  // 기울기
        double b = params(1);  // 절편

        // 7. 거리 계산
        d = std::abs(b) / std::sqrt(a * a + 1);
        pi = -(std::atan2(a,1));

        ROS_INFO("직선 방정식: Yb = %.3f * Xb + %.3f", a, b);
    }


    
    ROS_INFO("로봇 (0,0) 기준 직선까지 최단 거리: %.3f m", d);
}

void LaneCam::initCameraParams() {
    res_x = 640.0;
    res_y = 360.0;
    cx = res_x / 2.0;
    cy = res_y / 2.0;

    fov_h = 70.42 * M_PI / 180.0;
    fov_v = 43.30 * M_PI / 180.0;

    fx = (res_x / 2.0) / std::tan(fov_h / 2.0);
    fy = (res_y / 2.0) / std::tan(fov_v / 2.0);

    h_cam = 0.145;
    roll_cam = 32.0 * M_PI / 180.0;

    d = 0;
    pi = 0;

    R_axes << 0, -1,  0,
              0,  0, -1,
              1,  0,  0;

    double c = std::cos(roll_cam);
    double s = std::sin(roll_cam);
    R_roll << 1, 0,  0,
              0, c, -s,
              0, s,  c;

    R_cb = (R_roll * R_axes).transpose();  // 원래 R_bc = R_roll * R_axes → R_cb = R_bc.T
    

    R_cnv <<  0.945834885295871,   0.0496633903097647,  14.7859591197888,
            -0.00515598051744514, 0.994561258950752,  -28.5278605128610,
            6.96599895821937e-18, -2.54787510534094e-18, 1.0;

    R_inv = R_cnv.inverse(); 
}

cv::Mat LaneCam::fn_lane_threshold(const cv::Mat& image, int val_threshold) {
    cv::Mat img_1ch;
    cv::cvtColor(image, img_1ch, cv::COLOR_BGR2GRAY);

    const float val_hist_percent = 1.0f;
    const int val_hist_size = 256;

    int val_stretch_low = 0, val_stretch_high = 255;

    if (val_hist_percent == 0.0f) {
        double minVal, maxVal;
        cv::minMaxLoc(img_1ch, &minVal, &maxVal);
        val_stretch_low = static_cast<int>(minVal);
        val_stretch_high = static_cast<int>(maxVal);
    } else {
        cv::Mat hist;
        const float range[] = { 0, 256 };
        const float* histRange = { range };
        cv::calcHist(&img_1ch, 1, 0, cv::Mat(), hist, 1, &val_hist_size, &histRange);

        std::vector<float> accumulator(val_hist_size, 0);
        accumulator[0] = hist.at<float>(0);
        for (int i = 1; i < val_hist_size; ++i)
            accumulator[i] = accumulator[i - 1] + hist.at<float>(i);

        float num_of_pixel = accumulator[val_hist_size - 1];
        float num_of_clip = num_of_pixel * (val_hist_percent / 100.0f);

        for (val_stretch_low = 0; val_stretch_low < val_hist_size; ++val_stretch_low) {
            if (accumulator[val_stretch_low] >= accumulator[0] + num_of_clip)
                break;
        }
        for (val_stretch_high = val_hist_size - 1; val_stretch_high >= 0; --val_stretch_high) {
            if (accumulator[val_stretch_high] <= (num_of_pixel - num_of_clip))
                break;
        }
    }

    cv::Mat img_stretch;
    try {
        float input_range = static_cast<float>(val_stretch_high - val_stretch_low);
        float alpha = static_cast<float>(val_hist_size - 1) / input_range;
        float beta = -val_stretch_low * alpha;

        cv::convertScaleAbs(img_1ch, img_stretch, alpha, beta);

        cv::Mat img_threshold_low, img_threshold_high;
        cv::threshold(img_1ch, img_threshold_low, val_stretch_low, 255, cv::THRESH_BINARY);
        cv::threshold(img_1ch, img_threshold_high, val_stretch_high, 255, cv::THRESH_BINARY);
        cv::bitwise_or(img_stretch, img_threshold_high, img_stretch, img_threshold_low);
    }
    catch (...) {
        img_stretch = cv::Mat::zeros(img_1ch.size(), CV_8UC1);
    }

    cv::Mat img_threshold;
    cv::threshold(img_stretch, img_threshold, val_threshold, 255, cv::THRESH_BINARY);

    return img_threshold;
}

Eigen::Vector2d LaneCam::pixelToWorld(double u, double v) const {
    // Step 1: 픽셀 → 보정된 픽셀
    Eigen::Vector3d meas(u, v, 1.0);
    Eigen::Vector3d meas_corr = R_inv * meas;
    double u_corr = meas_corr(0) / meas_corr(2);
    double v_corr = meas_corr(1) / meas_corr(2);

    // Step 2: 카메라 좌표계 방향 벡터
    double dx_cam = (u_corr - cx) / fx;
    double dy_cam = (v_corr - cy) / fy;
    Eigen::Vector3d dir_cam(dx_cam, dy_cam, 1.0);

    // Step 3: 로봇 기준 방향 벡터
    Eigen::Vector3d dir_body = R_cb * dir_cam;
    double dir_z = dir_body(2);

    // Step 4: 바닥면에 닿는 비율 계산 (카메라 높이 기준)
    double t_c = -h_cam / dir_z;

    double Xb = t_c * dir_body(0);
    double Yb = t_c * dir_body(1);

    return Eigen::Vector2d(Xb, Yb);
}

std::pair<std::string, cv::Point> LaneCam::decideLaneDirection(const cv::Mat& binaryImage) const {
    std::vector<cv::Point> whitePoints;
    cv::findNonZero(binaryImage, whitePoints);

    if (whitePoints.empty()) {
        return {"exception", cv::Point(-1, -1)};
    }

    // y가 최대인 점들만 추출 (맨 아래 라인)
    int y_max = 0;
    for (const auto& pt : whitePoints) {
        if (pt.y > y_max) y_max = pt.y;
    }

    std::vector<cv::Point> bottomLinePoints;
    for (const auto& pt : whitePoints) {
        if (pt.y == y_max) bottomLinePoints.push_back(pt);
    }

    // 왼쪽/오른쪽 후보
    cv::Point leftCandidate = *std::min_element(bottomLinePoints.begin(), bottomLinePoints.end(),
                                                [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
    cv::Point rightCandidate = *std::max_element(bottomLinePoints.begin(), bottomLinePoints.end(),
                                                 [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });

    // 픽셀 → 실좌표 변환 (이미 멤버 함수로 구현됨)
    Eigen::Vector2d leftPos = pixelToWorld(leftCandidate.x, leftCandidate.y);
    Eigen::Vector2d rightPos = pixelToWorld(rightCandidate.x, rightCandidate.y);
    Eigen::Vector2d frontPos = pixelToWorld(cx, res_y);  // 기준 앞방향

    // 삼각형 부호 있는 면적 계산 함수
    auto calcSignedArea = [](const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) -> double {
        return 0.5 * (p0.x() * (p1.y() - p2.y()) +
                      p1.x() * (p2.y() - p0.y()) +
                      p2.x() * (p0.y() - p1.y()));
    };

    Eigen::Vector2d robotPos(0, 0);
    double areaLeft = calcSignedArea(robotPos, frontPos, leftPos);
    double areaRight = calcSignedArea(robotPos, frontPos, rightPos);

    // 판단 로직
    if (areaLeft > 0 && areaRight > 0) {
        return {"left", leftCandidate};
    } else if (areaLeft < 0 && areaRight < 0) {
        return {"right", rightCandidate};
    } else if (std::abs(areaLeft) > std::abs(areaRight)) {
        return {"left", leftCandidate};
    } else if (std::abs(areaLeft) < std::abs(areaRight)) {
        return {"right", rightCandidate};
    } else {
        return {"exception", cv::Point(-1, -1)};
    }
}

std::vector<cv::Point> LaneCam::extractLanePoints2(const cv::Mat& binaryImage, const std::string& direction, int tol) const {
    std::vector<cv::Point> whitePoints;
    cv::findNonZero(binaryImage, whitePoints);//0이 아닌 부분을 추출하는 코드이나 문제시 수정

    std::vector<cv::Point> roiPoints;

    
    int box_num = 5;
    int u_min = 0;
    int u_max = 639;
    int v_min = 140;
    int v_max = 359;

    // 각 박스의 가로/세로 크기(픽셀 수)
    int d_u = (u_max - u_min + 1) / box_num;  // 너비
    int d_v = (2*(v_max - v_min + 1)) / (box_num + 1);  // 높이

    // 계단의 수직 이동 step(그림처럼 절반씩 내려감)
    int step_v = d_v / 2;
    if(direction == "right"){
        for (int i = 0; i < box_num; ++i) {
            // i번째 박스의 좌상단 좌표 (Rect는 [x, y, width, height], 우/하 경계는 배타적)
            int x0 = u_min + d_u * i;
            int y0 = v_min + step_v * i;

            // 이미지 범위로 클램핑
            x0 = std::max(0, std::min(x0, binaryImage.cols - 1));
            y0 = std::max(0, std::min(y0, binaryImage.rows - 1));

            int w = d_u;
            int h = d_v;

            // 우/하 경계가 이미지 밖으로 나가면 잘라줌
            if (x0 + w > binaryImage.cols) w = binaryImage.cols - x0;
            if (y0 + h > binaryImage.rows) h = binaryImage.rows - y0;
            if (w <= 0 || h <= 0) continue;

            cv::Rect roi(x0, y0, w, h);

            // ROI 안에서 흰 점(0이 아닌 픽셀) 찾기
            std::vector<cv::Point> local;
            cv::findNonZero(binaryImage(roi), local);

            // 로컬 좌표를 전체 좌표로 변환해서 합치기
            for (auto &p : local) p += roi.tl();
            roiPoints.insert(roiPoints.end(), local.begin(), local.end());
        }
    } else{
        for (int i = 0; i < box_num; ++i) {
            // i번째 박스의 좌상단 좌표 (Rect는 [x, y, width, height], 우/하 경계는 배타적)
            int x0 = u_max - d_u * (i + 1);
            int y0 = v_min + step_v * i;

            // 이미지 범위로 클램핑
            x0 = std::max(0, std::min(x0, binaryImage.cols - 1));
            y0 = std::max(0, std::min(y0, binaryImage.rows - 1));

            int w = d_u;
            int h = d_v;

            // 우/하 경계가 이미지 밖으로 나가면 잘라줌
            if (x0 + w > binaryImage.cols) w = binaryImage.cols - x0;
            if (y0 + h > binaryImage.rows) h = binaryImage.rows - y0;
            if (w <= 0 || h <= 0) continue;

            cv::Rect roi(x0, y0, w, h);

            // ROI 안에서 흰 점(0이 아닌 픽셀) 찾기
            std::vector<cv::Point> local;
            cv::findNonZero(binaryImage(roi), local);

            // 로컬 좌표를 전체 좌표로 변환해서 합치기
            for (auto &p : local) p += roi.tl();
            roiPoints.insert(roiPoints.end(), local.begin(), local.end());
        }
    }
    

    if (roiPoints.empty()) return {};

    // y 기준 오름차순 정렬
    std::sort(roiPoints.begin(), roiPoints.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.y < b.y;
    });

    int y_min = roiPoints.front().y;
    int y_max = roiPoints.back().y;

    // y_min / y_max 에 해당하는 점들만 추출
    std::vector<cv::Point> y_min_pts, y_max_pts;
    for (const auto& pt : roiPoints) {
        if (pt.y == y_min) y_min_pts.push_back(pt);
        if (pt.y == y_max) y_max_pts.push_back(pt);
    }

    cv::Point point1, point2;

    if (direction == "left") {
        point1 = *std::min_element(y_min_pts.begin(), y_min_pts.end(),
                                   [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
        point2 = *std::min_element(y_max_pts.begin(), y_max_pts.end(),
                                   [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
    } else if (direction == "right") {
        point1 = *std::max_element(y_min_pts.begin(), y_min_pts.end(),
                                   [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
        point2 = *std::max_element(y_max_pts.begin(), y_max_pts.end(),
                                   [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
    } else {
        return {};
    }

    int y1 = point1.y, y2 = point2.y;

    // y 중간 n개를 등간격으로 계산
    std::vector<int> ys;
    for (int i = 1; i <= (point_num -2); ++i) {
        double y_interp = y1 + (y2 - y1 + 1) * i / double(point_num);
        ys.push_back(static_cast<int>(std::round(y_interp)));
    }

    std::vector<cv::Point> additionalPoints;

    for (int y_target : ys) {
        std::vector<cv::Point> nearPoints;
        for (const auto& pt : roiPoints) {
            if (std::abs(pt.y - y_target) <= tol)
                nearPoints.push_back(pt);
        }

        if (nearPoints.empty()) continue;

        cv::Point selected;
        if (direction == "left") {
            selected = *std::min_element(nearPoints.begin(), nearPoints.end(),
                                         [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
        } else if (direction == "right") {
            selected = *std::max_element(nearPoints.begin(), nearPoints.end(),
                                         [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
        }
        additionalPoints.push_back(selected);
    }
    std::vector<cv::Point> finalPoints = { point1 };
    finalPoints.insert(finalPoints.end(), additionalPoints.begin(), additionalPoints.end());
    finalPoints.push_back(point2);

    return finalPoints;
}

std::vector<cv::Point> LaneCam::extractLanePoints1(const cv::Mat& binaryImage, const std::string& direction, int tol) const {
    std::vector<cv::Point> whitePoints;
    cv::findNonZero(binaryImage, whitePoints);//0이 아닌 부분을 추출하는 코드이나 문제시 수정

    // ROI: y in [180, 359]
    std::vector<cv::Point> roiPoints;
    for (const auto& pt : whitePoints) {
        if (pt.y >= 180 && pt.y <= 359) {
            roiPoints.push_back(pt);
        }
    }

    if (roiPoints.empty()) return {};

    // y 기준 오름차순 정렬
    std::sort(roiPoints.begin(), roiPoints.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.y < b.y;
    });

    int y_min = roiPoints.front().y;
    int y_max = roiPoints.back().y;

    // y_min / y_max 에 해당하는 점들만 추출
    std::vector<cv::Point> y_min_pts, y_max_pts;
    for (const auto& pt : roiPoints) {
        if (pt.y == y_min) y_min_pts.push_back(pt);
        if (pt.y == y_max) y_max_pts.push_back(pt);
    }

    cv::Point point1, point2;

    if (direction == "left") {
        point1 = *std::min_element(y_min_pts.begin(), y_min_pts.end(),
                                   [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
        point2 = *std::min_element(y_max_pts.begin(), y_max_pts.end(),
                                   [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
    } else if (direction == "right") {
        point1 = *std::max_element(y_min_pts.begin(), y_min_pts.end(),
                                   [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
        point2 = *std::max_element(y_max_pts.begin(), y_max_pts.end(),
                                   [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
    } else {
        return {};
    }

    int y1 = point1.y, y2 = point2.y;

    // y 중간 n개를 등간격으로 계산
    std::vector<int> ys;
    for (int i = 1; i <= (point_num -2); ++i) {
        double y_interp = y1 + (y2 - y1) * i / double(point_num);
        ys.push_back(static_cast<int>(std::round(y_interp)));
    }

    std::vector<cv::Point> additionalPoints;

    for (int y_target : ys) {
        std::vector<cv::Point> nearPoints;
        for (const auto& pt : roiPoints) {
            if (std::abs(pt.y - y_target) <= tol)
                nearPoints.push_back(pt);
        }

        if (nearPoints.empty()) continue;

        cv::Point selected;
        if (direction == "left") {
            selected = *std::min_element(nearPoints.begin(), nearPoints.end(),
                                         [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
        } else if (direction == "right") {
            selected = *std::max_element(nearPoints.begin(), nearPoints.end(),
                                         [](const cv::Point& a, const cv::Point& b) { return a.x < b.x; });
        }
        additionalPoints.push_back(selected);
    }

    std::vector<cv::Point> finalPoints = { point1 };
    finalPoints.insert(finalPoints.end(), additionalPoints.begin(), additionalPoints.end());
    finalPoints.push_back(point2);

    return finalPoints;
}

// 최소제곱 원 맞춤 (Kåsa 형태): x^2 + y^2 + A x + B y + C = 0
// A, B, C를 구한 뒤 중심과 반지름으로 변환
CircleFit LaneCam::estimateCircle(const std::vector<cv::Point2d>& pts) {
  CircleFit out;
  const size_t n = pts.size();
  if (n < 3) {
    return out; // 점 3개 미만이면 불가
  }

  // A X = b  (여기서 대문자 A는 행렬, 소문자 a,b,c는 계수와 구분)
  // 행렬 A: [ x_i  y_i  1 ],  b: [ -(x_i^2 + y_i^2) ]
  Eigen::MatrixXd Am(n, 3);
  Eigen::VectorXd b(n);

  for (size_t i = 0; i < n; ++i) {
    const double x = pts[i].x;
    const double y = pts[i].y;
    Am(i, 0) = x;
    Am(i, 1) = y;
    Am(i, 2) = 1.0;
    b(i) = -(x*x + y*y);
  }

  // 정상방정식 (A^T A) p = A^T b  풀기
  Eigen::Matrix3d N = Am.transpose() * Am;
  Eigen::Vector3d rhs = Am.transpose() * b;

  // 수치적으로 불안정하면 실패 처리
  if (N.determinant() == 0.0) {
    return out;
  }

  Eigen::Vector3d p = N.ldlt().solve(rhs);
  const double A = p(0);
  const double B = p(1);
  const double C = p(2);

  // 중심과 반지름
  const double xc = -A * 0.5;
  const double yc = -B * 0.5;
  const double rad_sq = xc*xc + yc*yc - C;

  if (!(rad_sq > 0.0) || !std::isfinite(rad_sq)) {
    return out;
  }

  out.xc = xc;
  out.yc = yc;
  out.r  = std::sqrt(rad_sq);
  out.ok = std::isfinite(out.r);
  return out;
}
