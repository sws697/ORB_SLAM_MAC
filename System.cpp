#include"System.h"
#include <iomanip>
#include <iostream>
#include <thread>

namespace ORB_SLAM_MAC
{
     System::System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const std::string &strSequence = std::string())
     {
         int i = 0;
     }

     Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp, std::string filename="")
     {
        
     }
}