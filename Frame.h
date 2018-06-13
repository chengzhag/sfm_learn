//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_FRAME_H
#define SFM_LEARN_FRAME_H

#include "common_include.h"
#include "Camera.h"

namespace sky {

    class Frame {
    public:
        typedef shared_ptr<Frame> Ptr;
        SE3 T_c_w;      // transform from world to camera
        Camera::Ptr camera;     // Pinhole RGBD Camera model
        Mat image;

        Frame();

        Frame(const Camera::Ptr &camera, const Mat &image) : camera(camera), image(image) {}

        ~Frame();

        static Frame::Ptr createFrame();

        // Get Camera Center
        Vector3d getCamCenter() const;

        void setPose(const SE3 &T_c_w);

        // check if a point is in this frame
        bool isInFrame(const Vector3d &pt_world);
    };

}


#endif //SFM_LEARN_FRAME_H
