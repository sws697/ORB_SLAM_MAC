#ifndef SYSTEM_H
#define SYSTEM_H


#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include<Eigen/Dense>

namespace ORB_SLAM_MAC
{
class Verbose;
class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;
class System 
{
    public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const std::string &strSequence = std::string());
    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, std::string filename="");
    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();
};
}
#endif