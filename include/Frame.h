#ifndef FRAME_H
#define FRAME_H
#include "System.h"
#include "ThirdParty/DBoW2/include/DBoW2/DBoW2.h"
#include "ORBVocabulary.h"
#include"ORBextractor.h"
#include<vector>


namespace ORB_SLAM_MAC
{

#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
    class Frame
    {
        public:
        Frame();
        void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);
        void ComputeBoW();

        Eigen::Vector3f GetVelocity() const;

        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Sophus::SE3<float> mTcw;
        Eigen::Matrix<float,3,3> mRwc;
        Eigen::Matrix<float,3,1> mOw;
        Sophus::SE3<float> mTlr, mTrl;
        Eigen::Matrix<float,3,3> mRlr;
        Eigen::Vector3f mtlr;


        // IMU linear velocity
        Eigen::Vector3f mVw;
        bool mbHasVelocity;
        ORBextractor mpORBextractorLeft;
    };
} // namespace ORB_SLAM_MAC

#endif