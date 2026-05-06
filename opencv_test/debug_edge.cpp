#include <opencv2/opencv.hpp>
#include <iostream>

// ======= fn_edge_map 그대로 가져오기 =======
cv::Mat fn_edge_map(const cv::Mat& bgr, const cv::Rect& roi) {
    cv::Rect R = roi & cv::Rect(0,0,bgr.cols,bgr.rows);
    if (R.width<=0 || R.height<=0) return cv::Mat();
    cv::Mat img = bgr(R);

    // HSV, Gray 변환
    cv::Mat hsv, s, gray; 
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> ch; 
    cv::split(hsv, ch); 
    s = ch[1];
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    // 블러로 노이즈 억제
    cv::GaussianBlur(s,   s,   cv::Size(5,5), 1.2);
    cv::GaussianBlur(gray,gray,cv::Size(5,5), 1.2);

    // Canny
    cv::Mat e1, e2, edge_local;
    cv::Canny(s,   e1, 50,150);
    cv::Canny(gray,e2,60,260,3, true);

    // OR 융합
    //cv::bitwise_or(e1, e2, edge_local);
    edge_local = e2.clone();


    // 모폴로지 Close (3x3)
    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::morphologyEx(edge_local, edge_local, cv::MORPH_CLOSE, k);

    // 원본 크기 맞추기
    cv::Mat edge_full = cv::Mat::zeros(bgr.size(), CV_8UC1);
    edge_local.copyTo(edge_full(R));
    return edge_full;
}

// ======= main 함수 =======
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "사용법: " << argv[0] << " <이미지 경로>" << std::endl;
        return -1;
    }

    // 이미지 읽기
    cv::Mat img = cv::imread(argv[1]);
    if (img.empty()) {
        std::cerr << "이미지를 불러올 수 없습니다: " << argv[1] << std::endl;
        return -1;
    }

    // ROI (전체 이미지 사용)
    cv::Rect roi(0, 0, img.cols, img.rows);

    // 에지 맵 생성
    cv::Mat edges = fn_edge_map(img, roi);

    // 결과 출력
    cv::imshow("원본 이미지", img);
    cv::imshow("Canny Edge 결과", edges);
    cv::waitKey(0);
    return 0;
}
