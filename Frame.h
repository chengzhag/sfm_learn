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
        SE3 Tcw;      // transform from world to camera
        Camera::Ptr camera;     // Pinhole RGBD Camera model
        int cols, rows;

        Frame();

        Frame(const Camera::Ptr &camera, const Mat &image) :
                camera(camera), cols(image.cols), rows(image.rows) {}

        ~Frame();


        Vector3d getCamCenterEigen() const;

        cv::Mat getTcwMatCV(int rtype) {
            cv::Mat TcwCV, TcwCVR;
            cv::eigen2cv(Tcw.matrix(), TcwCV);
            TcwCV.convertTo(TcwCVR, rtype);
            return TcwCVR;
        }

        cv::Mat getTcw34MatCV(int rtype) {
            auto TcwCV = getTcwMatCV(rtype);
            Mat Tcw34;
            TcwCV(cv::Range(0, 3), cv::Range(0, 4)).convertTo(Tcw34, rtype);
            return Tcw34;
        }

        cv::Mat getTwcMatCV(int rtype) {
            cv::Mat TwcCV, TwcCVR;
            cv::eigen2cv(Tcw.inverse().matrix(), TwcCV);
            TwcCV.convertTo(TwcCVR, rtype);
            return TwcCVR;
        }

        template<typename T>
        cv::Matx<T, 1, 3> getAngleAxisWcMatxCV() {
            Sophus::AngleAxisd angleAxis(Tcw.so3().matrix());
            auto axis = angleAxis.angle() * angleAxis.axis();
            cv::Matx<T, 1, 3> angleAxisCV(axis[0],axis[1],axis[2]);
            return angleAxisCV;
        };

        template<typename T>
        void setTcw(Matx<T,2,3> angleAxisAndTranslation){
            Tcw.so3()=SO3(angleAxisAndTranslation(0,0),
                            angleAxisAndTranslation(0,1),
                            angleAxisAndTranslation(0,2));
            Tcw.translation()=Vector3d(angleAxisAndTranslation(1,0),
                                         angleAxisAndTranslation(1,1),
                                         angleAxisAndTranslation(1,2));
        }




/*        cv::Mat getProjMatCV() {
            return camera->getKMatCV()*getTcw34MatCV();
        }*/


        // check if a point is in this frame
        bool isInFrame(const Vector3d &pt_world);
    };

}


#endif //SFM_LEARN_FRAME_H
