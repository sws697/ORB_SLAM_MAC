#include <opencv2/opencv.hpp>
#include <iostream>

int main(int argc, char** argv)
{
    cv::Mat img;
    /*---- 1. 读入 ----*/
    if (argc >= 2) {
        img = cv::imread(argv[1]);
        if (img.empty()) {
            std::cerr << "Cannot read image: " << argv[1] << std::endl;
            return 1;
        }
    } else {
        cv::VideoCapture cap(0);          // 打开默认摄像头
        if (!cap.isOpened()) {
            std::cerr << "Cannot open camera\n";
            return 1;
        }
        cap >> img;                       // 抓一帧
    }

    /*---- 2. 处理 ----*/
    cv::Mat gray, blur, edge;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5,5), 0);
    cv::Canny(blur, edge, 50, 150);

    /*---- 3. 显示 ----*/
    cv::imshow("edge", edge);
    cv::waitKey(10);
    cv::imwrite("edge.png", edge);
    std::cout << "Saved edge.png  (w=" << edge.cols << ", h=" << edge.rows << ")\n";

    
    return 0;
}