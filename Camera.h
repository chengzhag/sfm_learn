//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_CAMERA_H
#define SFM_LEARN_CAMERA_H

#include "common_include.h"

namespace sky {

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
    };

}


#endif //SFM_LEARN_CAMERA_H
