#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

static const string kWinParams = "Params";
static const string kWinView   = "View";

// ===== 트랙바 상태값 (초기값) =====
int use_channel = 2;      // 0: Gray, 1: S(HSV), 2: OR(Fusion)
int g_t1 = 60,  g_t2 = 260;  // Gray Canny thresholds
int s_t1 = 50,  s_t2 = 150;  // S    Canny thresholds

int blur_n = 2;           // Gaussian ksize = 2*blur_n + 1  (1,3,5,7..)
int sigma10 = 12;         // sigmaX = sigma10/10.0
int aperture_idx = 0;     // 0->3, 1->5, 2->7
int l2grad = 1;           // 0/1

int morph_on = 1;         // 0/1
int morph_n = 1;          // kernel = 2*morph_n+1

// ===== 유틸 =====
inline int odd_from_n(int n) { return 2*n + 1; }

static Mat run_canny(const Mat& src, int t1, int t2, int aperture_idx, bool l2) {
    int apertureSize = 3 + 2*aperture_idx; // 3,5,7
    Mat edges;
    Canny(src, edges, t1, t2, apertureSize, l2);
    return edges;
}

static Mat make_edge_map(const Mat& bgr) {
    // 채널 준비
    Mat gray; cvtColor(bgr, gray, COLOR_BGR2GRAY);
    Mat hsv;  cvtColor(bgr, hsv, COLOR_BGR2HSV);
    vector<Mat> ch; split(hsv, ch);
    Mat S = ch[1];

    // 블러
    int k_blur = max(1, odd_from_n(blur_n));
    double sigmaX = sigma10 / 10.0;
    if (k_blur > 1 || sigmaX > 0.0) {
        GaussianBlur(gray, gray, Size(k_blur, k_blur), sigmaX);
        GaussianBlur(S,    S,    Size(k_blur, k_blur), sigmaX);
    }

    // Canny
    Mat e_gray = run_canny(gray, g_t1, g_t2, aperture_idx, l2grad != 0);
    Mat e_s    = run_canny(S,    s_t1, s_t2, aperture_idx, l2grad != 0);

    Mat edge;
    if (use_channel == 0)      edge = e_gray;
    else if (use_channel == 1) edge = e_s;
    else                       bitwise_or(e_gray, e_s, edge);

    // 모폴로지 close
    if (morph_on) {
        int k_morph = max(1, odd_from_n(morph_n));
        Mat k = getStructuringElement(MORPH_RECT, Size(k_morph, k_morph));
        morphologyEx(edge, edge, MORPH_CLOSE, k);
    }
    return edge;
}

// 두 이미지를 같은 높이로 나란히 보기
static Mat side_by_side(const Mat& left, const Mat& right) {
    int h = 720; // 보기 편한 기본 높이
    double scaleL = (double)h / left.rows;
    double scaleR = (double)h / right.rows;
    Mat L, R;
    resize(left,  L, Size(), scaleL, scaleL, INTER_AREA);
    resize(right, R, Size(), scaleR, scaleR, INTER_NEAREST);
    Mat R3; cvtColor(R, R3, COLOR_GRAY2BGR);
    Mat cat; hconcat(L, R3, cat);
    return cat;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "사용법: " << argv[0] << " <이미지 경로>\n";
        return -1;
    }

    Mat img = imread(argv[1]);
    if (img.empty()) {
        cerr << "이미지를 불러올 수 없습니다: " << argv[1] << "\n";
        return -1;
    }

    namedWindow(kWinParams, WINDOW_AUTOSIZE);
    namedWindow(kWinView,   WINDOW_NORMAL);

    createTrackbar("Channel(0:Gray,1:S,2:OR)", kWinParams, &use_channel, 2);
    createTrackbar("Gray T1",  kWinParams, &g_t1,  1000);
    createTrackbar("Gray T2",  kWinParams, &g_t2,  1000);
    createTrackbar("S T1",     kWinParams, &s_t1,  1000);
    createTrackbar("S T2",     kWinParams, &s_t2,  1000);

    createTrackbar("Blur n(ksz=2n+1)", kWinParams, &blur_n, 10);
    createTrackbar("Sigma x(x0.1)",    kWinParams, &sigma10, 50);

    createTrackbar("Aperture(0:3,1:5,2:7)", kWinParams, &aperture_idx, 2);
    createTrackbar("L2gradient(0/1)",       kWinParams, &l2grad, 1);

    createTrackbar("Morph close(0/1)", kWinParams, &morph_on, 1);
    createTrackbar("Morph n(ksz=2n+1)", kWinParams, &morph_n, 5);

    cout << "[단축키] q/ESC: 종료, s: edge 저장(edge_output.png), p: 현재 파라미터 콘솔 출력\n";

    while (true) {
        Mat edges = make_edge_map(img);

        // 상태 텍스트
        int apertureSize = 3 + 2*aperture_idx;  // 3/5/7
        double sigmaX = sigma10 / 10.0;
        string info = format(
            "Chan:%d | G(%d,%d) S(%d,%d) | Blur:%d sig:%.1f | Ap:%d L2:%d | Morph:%d k:%d",
            use_channel, g_t1, g_t2, s_t1, s_t2, 2*blur_n+1, sigmaX, apertureSize, l2grad, morph_on, 2*morph_n+1);

        Mat view = side_by_side(img, edges);
        putText(view, info, {20, 40}, FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2, LINE_AA);
        imshow(kWinView, view);

        int key = waitKey(20);
        if (key == 27 || key == 'q' || key == 'Q') break;
        if (key == 's' || key == 'S') {
            imwrite("edge_output.png", edges);
            cout << "저장됨: edge_output.png\n";
        }
        if (key == 'p' || key == 'P') {
            cout << "[PARAMS] use_channel=" << use_channel
                 << "  gray(" << g_t1 << "," << g_t2 << ")"
                 << "  S("    << s_t1 << "," << s_t2 << ")"
                 << "  blur="  << (2*blur_n+1)
                 << "  sigmaX="<< sigmaX
                 << "  aperture=" << apertureSize
                 << "  l2=" << l2grad
                 << "  morph=" << morph_on
                 << "  morph_k=" << (2*morph_n+1)
                 << endl;
        }
    }
    return 0;
}
