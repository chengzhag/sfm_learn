//
// Created by pidan1231239 on 18-6-23.
//

#ifndef SFM_LEARN_BA_H
#define SFM_LEARN_BA_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include "Map.h"
#include "Frame.h"
#include "common_include.h"
#include <unordered_map>

namespace sky {

    using namespace std;

    class BA {

        class ReprojectCost {
            cv::Point2d observation;
        public:
            ReprojectCost(const cv::Point2d &observation) : observation(observation) {}

            template<typename T>
            bool
            operator()(const T *const intrinsic, const T *const extrinsic, const T *const pos3d, T *residuals) const {
                const T *r = extrinsic;
                const T *t = &extrinsic[3];

                T pos_proj[3];
                ceres::AngleAxisRotatePoint(r, pos3d, pos_proj);

                // Apply the camera translation
                pos_proj[0] += t[0];
                pos_proj[1] += t[1];
                pos_proj[2] += t[2];

                const T x = pos_proj[0] / pos_proj[2];
                const T y = pos_proj[1] / pos_proj[2];

                const T fx = intrinsic[0];
                const T fy = intrinsic[1];
                const T cx = intrinsic[2];
                const T cy = intrinsic[3];

                // Apply intrinsic
                const T u = fx * x + cx;
                const T v = fy * y + cy;

                residuals[0] = u - T(observation.x);
                residuals[1] = v - T(observation.y);

                return true;
            }
        };

        ceres::Problem problem;
        Map::Ptr map;

        unordered_map<Camera::Ptr, Matx14d> cameraIntrinsics;
        unordered_map<Frame::Ptr, Matx23d> frameExtrinsics;
        unordered_map<MapPoint::Ptr, Matx13d> mapPointsPos;

    public:

        BA(const Map::Ptr &map) : map(map) {}

        void loadMap() {
#ifdef DEBUG
            cout << "==============BA:loading map==============" << endl;
#endif
            for (auto &mapPoints:map->mapPoints) {
                //加载mapPointsPos
                mapPointsPos[mapPoints] = mapPoints->getPosMatx13<double>();
            }
            //加载frameExtrinsics
            for (auto &frame:map->frames) {
                auto angleAxis = frame->getAngleAxisWcMatxCV<double>();
                auto t = frame->T_c_w.translation();
                frameExtrinsics[frame] = Matx23d(angleAxis(0), angleAxis(1), angleAxis(2),
                                                 t[0], t[1], t[2]);
                //加载cameraIntrinsics
                if (cameraIntrinsics.find(frame->camera) == cameraIntrinsics.end())
                    cameraIntrinsics[frame->camera] = Matx14d(
                            frame->camera->fx, frame->camera->fy, frame->camera->cx, frame->camera->cy);
            }
#ifdef DEBUG
            cout << mapPointsPos.size() << " map points" << endl;
            int i = 0;
            for (auto &mapPoints:mapPointsPos) {
                cout << mapPoints.second << endl;
                ++i;
                if (i >= 5)break;
            }
            cout << "..." << endl << endl;

            cout << frameExtrinsics.size() << " frames" << endl;
            i = 0;
            for (auto &frame:frameExtrinsics) {
                cout << frame.second << endl;
                ++i;
                if (i >= 5)break;
            }
            cout << "..." << endl << endl;

            cout << cameraIntrinsics.size() << " cameras" << endl;
            i = 0;
            for (auto &camera:cameraIntrinsics) {
                cout << camera.second << endl;
                ++i;
                if (i >= 5)break;
            }
            cout << "..." << endl << endl;
#endif
        }

        void bundleAdjustment() {
#ifdef DEBUG
            cout << "==============BA:processing==============" << endl;
#endif
        }

        void writeMap() {
#ifdef DEBUG
            cout << "==============BA:writing map==============" << endl;
#endif
        }

    };

}


#endif //SFM_LEARN_BA_H
