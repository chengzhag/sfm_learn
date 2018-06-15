//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_CAMERA_H
#define SFM_LEARN_CAMERA_H

#include "common_include.h"
#include "opencv2/opencv.hpp"

namespace sky {

    using namespace cv;

    class Camera {
    public:
        typedef shared_ptr<Camera> Ptr;
        float fx_, fy_, cx_, cy_;

        Camera(float fx, float fy, float cx, float cy) :
                fx_(fx), fy_(fy), cx_(cx), cy_(cy) {}

        //坐标转换

        Vector3d world2camera(const Vector3d &p_w, const SE3 &T_c_w);

        Vector3d camera2world(const Vector3d &p_c, const SE3 &T_c_w);

        Vector2d camera2pixel(const Vector3d &p_c);

        Vector3d pixel2camera(const Vector2d &p_p, double depth = 1);

        Vector3d pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth = 1);

        Vector2d world2pixel(const Vector3d &p_w, const SE3 &T_c_w);

        Point2f pixel2normal(const Point2d &p) const;

        //获取参数
        float getFocalLength() {
            return (fx_ + fy_) / 2;
        }

        cv::Point2d getPrincipalPoint() {
            return cv::Point2d(cx_, cy_);
        }

        cv::Matx<float, 3, 3> getIntrinsics() {
            return cv::Matx<float, 3, 3>(fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
        }
    };

}


#endif //SFM_LEARN_CAMERA_H
