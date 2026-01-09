#include "Tracking.h"

namespace ORB_SLAM_MAC
{

    Tracking::Tracking()
    {
        
    }
    Sophus::SE3f Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, std::string filename)
    {
        if (mState==NO_IMAGES_YET)
            t0=timestamp;
        if(mSensor == System::eSensor::MONOCULAR)
        {
            if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET ||(lastID - initID) < mMaxFrames)
            {

            }
            else
            {

            }
        }
    }
}