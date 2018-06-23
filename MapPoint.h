//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_MAPPOINT_H
#define SFM_LEARN_MAPPOINT_H

#include "common_include.h"
#include "Frame.h"
#include <utility>

namespace sky {

    class MapPoint {

    public:

        typedef shared_ptr<MapPoint> Ptr;
        Mat descriptor; // Descriptor for matching
        list<std::pair<Frame::Ptr,cv::Point2d>> observedFrames;//观测帧和像素坐标
        Vector3d pos;       // Position in world
        Vec3b rgb;


        MapPoint();

        MapPoint(const Vector3d &pos, const Mat &descriptor, const Vec3b &rgb) :
                pos(pos),
                descriptor(descriptor),
                rgb(rgb) {}

        inline cv::Point3f getPosCV() const {
            return cv::Point3f(pos(0, 0), pos(1, 0), pos(2, 0));
        }

        void addObervedFrame(const Frame::Ptr &observedFrame,const cv::Point2d &pixelCoor) {
            if (observedFrame)
                observedFrames.push_back(
                        std::pair<Frame::Ptr,cv::Point2d>(observedFrame,pixelCoor));
        }
    };


}


#endif //SFM_LEARN_MAPPOINT_H
