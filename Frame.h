//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_FRAME_H
#define SFM_LEARN_FRAME_H

#include "common_include.h"
#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace sky {

    class Frame {
    public:
        typedef shared_ptr<Frame> Ptr;
        SE3 T_c_w;      // transform from world to camera
        Camera::Ptr camera;     // Pinhole RGBD Camera model
        int cols, rows;

        Frame();

        Frame(const Camera::Ptr &camera, const Mat &image) :
                camera(camera), cols(image.cols), rows(image.rows) {}

        ~Frame();


        Vector3d getCamCenterEigen() const;

        cv::Mat getTcwCV() {
            cv::Mat TcwCV;
            cv::eigen2cv(T_c_w.matrix(), TcwCV);
            return TcwCV;
        }

        cv::Mat getProjMatCV() {
            auto TcwCV = getTcwCV();
            Mat projMatx;
            TcwCV(cv::Range(0, 3), cv::Range(0, 4)).convertTo(projMatx, CV_32FC1);
            return projMatx;
        }


        // check if a point is in this frame
        bool isInFrame(const Vector3d &pt_world);
    };

}


#endif //SFM_LEARN_FRAME_H
