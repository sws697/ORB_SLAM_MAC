#ifndef TRACKING_H
#define TRACKING_H
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include"System.h"
#include"Frame.h"
#include"ORBextractor.h"
#include <mutex>
#include <unordered_set>
#include <functional>
namespace ORB_SLAM_MAC
{
class Frame;
class Tracking
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking();
    ~Tracking()=default;

    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, std::string filename);
    public:
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5
    };
    eTrackingState mState;
    eTrackingState mLastProcessedState;
    // Input sensor
    int mSensor;

    int lastID, initID;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    double t0;
    Frame mCurrentFrame;
    private:
        std::shared_ptr<const ORBextractor> mpORBextractorLeft;
};
} // namespace ORB_SLAM_MAC

#endif