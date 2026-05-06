// robust_chessboard_detect.cpp
#include <opencv2/opencv.hpp>
#include <fstream>   // 파일 저장을 위해 필요
#include <iostream>
using namespace std;

static bool detectSB(const cv::Mat& gray, cv::Size board, vector<cv::Point2f>& pts) {
#if CV_VERSION_MAJOR >= 4
    int flags = cv::CALIB_CB_EXHAUSTIVE | cv::CALIB_CB_ACCURACY; // 탐색↑ 정확도↑
    return cv::findChessboardCornersSB(gray, board, pts, flags);
#else
    (void)gray; (void)board; (void)pts; return false;
#endif
}

static bool detectClassic(const cv::Mat& gray, cv::Size board, vector<cv::Point2f>& pts) {
    bool ok = cv::findChessboardCorners(
        gray, board, pts,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE // FAST_CHECK 제거
    );
    if (ok) {
        cv::Mat g; if (gray.channels()==3) cv::cvtColor(gray, g, cv::COLOR_BGR2GRAY); else g = gray;
        cv::cornerSubPix(g, pts, {11,11}, {-1,-1},
            {cv::TermCriteria::EPS+cv::TermCriteria::MAX_ITER, 40, 0.001});
    }
    return ok;
}

static cv::Mat unsharp(const cv::Mat& g, double sigma=1.0, double amount=1.0){
    cv::Mat blur, sharp;
    cv::GaussianBlur(g, blur, cv::Size(0,0), sigma);
    cv::addWeighted(g, 1+amount, blur, -amount, 0, sharp);
    return sharp;
}

static cv::Mat gammaCorrect(const cv::Mat& g, double gamma){
    CV_Assert(g.type()==CV_8U);
    static cv::Mat lut(1,256,CV_8U);
    for(int i=0;i<256;i++) lut.at<uchar>(i) = cv::saturate_cast<uchar>(pow(i/255.0, gamma)*255.0);
    cv::Mat out; cv::LUT(g, lut, out); return out;
}

int main(int argc, char** argv){
    if(argc < 4){
        cerr << "Usage: " << argv[0] << " <image_path> <cols> <rows>\n"
             << "       <cols>,<rows> = 내부 코너 수 (예: 8 6)\n";
        return 1;
    }

    string imgPath = argv[1];
    int cols = stoi(argv[2]), rows = stoi(argv[3]);
    cv::Size board(cols, rows);

    cv::Mat bgr = cv::imread(imgPath);
    if(bgr.empty()){ cerr << "Cannot open image: " << imgPath << "\n"; return 1; }

    // (선택) ROI: 보드가 위쪽에만 있다면 상단 60%만 검사하면 노이즈를 줄일 수 있음
    // cv::Rect roi(0, 0, bgr.cols, int(bgr.rows*0.6));
    // cv::Mat bgr_roi = bgr(roi).clone();
    cv::Mat bgr_roi = bgr;

    // 그레이
    cv::Mat gray; cv::cvtColor(bgr_roi, gray, cv::COLOR_BGR2GRAY);

    vector<pair<string, cv::Mat>> candidates;

    // 원본
    candidates.push_back({"gray", gray});

    // 대비 향상 (CLAHE) + 샤픈
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8,8));
    cv::Mat g1; clahe->apply(gray, g1);
    candidates.push_back({"clahe", g1});
    candidates.push_back({"clahe_unsharp", unsharp(g1, 1.2, 1.0)});

    // 감마로 과노출 억제(>1이면 어둡게) + 샤픈
    cv::Mat g2 = gammaCorrect(gray, 1.4);
    candidates.push_back({"gamma1.4", g2});
    candidates.push_back({"gamma1.4_unsharp", unsharp(g2, 1.0, 1.2)});

    // 2배 업스케일(칸이 작을 때 유리)
    cv::Mat g3; cv::resize(gray, g3, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
    candidates.push_back({"resize2x", g3});

    // 적응형 이진화 (배경 밝을 때 유리)
    cv::Mat g4; cv::adaptiveThreshold(gray, g4, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv::THRESH_BINARY, 35, 5);
    candidates.push_back({"adaptiveThresh", g4});

    vector<cv::Point2f> corners;
    string used;
    bool ok = false;

    for(auto& cand : candidates){
        const string& name = cand.first;
        cv::Mat img = cand.second;

        // SB 우선 시도
        if(detectSB(img, board, corners)){
            used = "SB/" + name; ok = true;
        } else if (detectClassic(img, board, corners)) {
            used = "Classic/" + name; ok = true;
        }

        if(ok){
            // 업스케일에서 찾았으면 원본 좌표계로 스케일 백
            if(name == "resize2x"){
                for(auto& p : corners){ p.x /= 2.f; p.y /= 2.f; }
            }
            // ROI를 썼다면, 전체 이미지 좌표로 보정
            // for(auto& p : corners){ p.y += roi.y; p.x += roi.x; }
            break;
        }
    }

    if(!ok){
        cout << "체커보드를 찾지 못했습니다.\n";
        // 디버그용으로 후보들을 보고 싶다면 아래 주석 해제:
        // for(auto& c : candidates){ cv::imshow(c.first, c.second); }
        // cv::waitKey();
        return 0;
    }

    // 결과 시각화
    cv::Mat vis = bgr.clone();
    cv::drawChessboardCorners(vis, board, corners, true);
    cout << "Detected " << corners.size() << " corners using " << used << "\n";
    for(size_t i=0;i<corners.size();++i){
        cout << i << "\t" << corners[i].x << "\t" << corners[i].y << "\n";
    }
    // --- 결과를 파일에 저장 ---
    std::ofstream ofs("/Users/gadeuk/Documents/졸업과제/checkerboard/improved/output.txt");
    if(!ofs.is_open()){
        std::cerr << "Failed to open file: " << "/Users/gadeuk/Documents/졸업과제/checkerboard/improved/output.txt" << std::endl;
        return 1;
    }
    ofs << "Detected " << corners.size() << " corners (u,v pixels):\n";
    for(size_t i=0;i<corners.size();++i){
        ofs << corners[i].x << "\t" << corners[i].y << "\n";
    }
    ofs.close();
    std::cout << "결과가 " << "/Users/gadeuk/Documents/졸업과제/checkerboard/improved/output.txt" << " 에 저장되었습니다 ✅\n";


    cv::imshow("result", vis);
    cv::waitKey(0);
    return 0;
}
