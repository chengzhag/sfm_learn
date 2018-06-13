//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_MAPPOINT_H
#define SFM_LEARN_MAPPOINT_H

#include "common_include.h"
#include "Frame.h"

namespace sky {

    class MapPoint {

    public:

        typedef shared_ptr<MapPoint> Ptr;
        Mat descriptor; // Descriptor for matching
        list<Frame::Ptr> observedFrames;   // key-frames that can observe this point
        Vector3d pos;       // Position in world

        MapPoint();

        MapPoint(const Vector3d &pos, const Mat &descriptor, const Frame::Ptr &observedFrame) :
                pos(pos),
                descriptor(descriptor) {
            addFrame(observedFrame);
        }

        inline cv::Point3f getPosCV() const {
            return cv::Point3f(pos(0, 0), pos(1, 0), pos(2, 0));
        }

        void addFrame(const Frame::Ptr &observedFrame) {
            if (!observedFrame)
                observedFrames.push_back(observedFrame);
        }
    };


}


#endif //SFM_LEARN_MAPPOINT_H
