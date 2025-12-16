#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "ORBextractor.h"
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;




/**
 * @brief 在图像上画出 KeyPoint 的绿色方框并显示
 * @param img   输入图像（CV_8UC1 或 CV_8UC3）
 * @param kps   关键点序列
 * @param winName 窗口名称
 * @param boxSize 方框半边长（像素），默认 4 → 9×9 框
 */
void showKeypoints(const cv::Mat& img,
                   const std::vector<cv::KeyPoint>& kps,
                   int waitkey,
                   const std::string& winName = "KeyPoints",
                   int boxSize = 4)
{
    cv::Mat show;
    if (img.channels() == 1) cv::cvtColor(img, show, cv::COLOR_GRAY2BGR);
    else                     show = img.clone();

    for (const auto& kp : kps)
    {
        cv::Point2f pt = kp.pt;
        cv::Point pt1 = pt - cv::Point2f(boxSize, boxSize);
        cv::Point pt2 = pt + cv::Point2f(boxSize, boxSize);
        cv::rectangle(show, pt1, pt2, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }

    cv::imshow(winName, show);
    cv::waitKey(waitkey);          // 按任意键关闭
    // cv::destroyWindow(winName);
}
int main(int argc,char** argv)
{
    cv::Mat img;
    if (argc >= 2) {
        img = cv::imread(argv[1]);
        if (img.empty()) {
            std::cerr << "Cannot read image: " << argv[1] << std::endl;
            return 1;
        }
    int nfeatures = 2000,  nlevels = 10, iniThFAST = 30, minThFAST = 12;
    float scalefactor = 1.2;
    ORB_SLAM_MAC::ORBextractor extractor(nfeatures, scalefactor, nlevels, iniThFAST, minThFAST);
    cv::Mat descriptors;
    vector<KeyPoint> keypoints;
    vector<int> vLappingArea = {INT32_MAX, INT32_MAX};
    int monobegin =extractor(img, keypoints, descriptors, vLappingArea);
    showKeypoints(img, keypoints,0);

    cout << "提取到的特征点数量：" << keypoints.size() << endl;
    }else{
        cv::VideoCapture cap(0);
        int nfeatures = 2000, nlevels = 10, iniThFAST = 30, minThFAST = 12;
        float scalefactor = 1.2;
        ORB_SLAM_MAC::ORBextractor extractor(nfeatures, scalefactor, nlevels, iniThFAST, minThFAST);
        cv::Mat descriptors;
        vector<KeyPoint> keypoints;
        vector<int> vLappingArea = {INT32_MAX, INT32_MAX};        
        while (true)
        {
        
        cap >> img;
        auto t0 = std::chrono::high_resolution_clock::now();
        extractor(img, keypoints, descriptors, vLappingArea);
        auto t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        showKeypoints(img, keypoints,1);
        cout << "提取特征点时间：" << ms << "ms"<<endl;
        // cout << img.size << endl;
        cout << "提取到的特征点数量：" << keypoints.size() << endl;
    }
    }
}