//
// Created by pidan1231239 on 18-6-13.
//

#include "Frame.h"

namespace sky {
    Frame::Frame()
            : camera(nullptr) {

    }

    Frame::~Frame() {

    }


    Vector3d Frame::getCamCenterEigen() const {
        return T_c_w.inverse().translation();
    }


    bool Frame::isInFrame(const Vector3d &pt_world) {
        // cout<<"pt_world = "<<endl<<pt_world<<endl;
        Vector3d p_cam = camera->world2camera(pt_world, T_c_w);
        // cout<<"P_cam = "<<p_cam.transpose()<<endl;
        if (p_cam(2, 0) < 0) return false;
        Vector2d pixel = camera->world2pixel(pt_world, T_c_w);
        // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
        return pixel(0, 0) > 0 && pixel(1, 0) > 0
               && pixel(0, 0) < cols
               && pixel(1, 0) < rows;
    }

}
