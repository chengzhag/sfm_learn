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
            cout << endl << "==============BA:loading map==============" << endl << endl;
#endif
            //加载mapPointsPos
            for (auto &mapPoints:map->mapPoints) {
                mapPointsPos[mapPoints] = mapPoints->getPosMatx13<double>();
            }
            //加载frameExtrinsics和cameraIntrinsics
            for (auto &frame:map->frames) {
                auto angleAxis = frame->getAngleAxisWcMatxCV<double>();
                auto t = frame->Tcw.translation();
                frameExtrinsics[frame] = Matx23d(angleAxis(0), angleAxis(1), angleAxis(2),
                                                 t[0], t[1], t[2]);
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
            cout << "..." << endl;

            cout << frameExtrinsics.size() << " frames" << endl;
            i = 0;
            for (auto &frame:frameExtrinsics) {
                cout << frame.second << endl;
                ++i;
                if (i >= 5)break;
            }
            cout << "..." << endl;

            cout << cameraIntrinsics.size() << " cameras" << endl;
            i = 0;
            for (auto &camera:cameraIntrinsics) {
                cout << camera.second << endl;
                ++i;
                if (i >= 5)break;
            }
            cout << "..." << endl;
#endif
        }

        void bundleAdjustment() {
#ifdef DEBUG
            cout << "==============BA:processing==============" << endl;
#endif

#ifdef DEBUG
            cout << "loading frameExtrinsics..." << endl;
#endif
            for (auto &frameExtrinsic:frameExtrinsics)
                problem.AddParameterBlock(frameExtrinsic.second.val, 6);
            problem.SetParameterBlockConstant(frameExtrinsics[map->frames.front()].val);

#ifdef DEBUG
            cout << "loading cameraIntrinsics..." << endl;
#endif
            for (auto &cameraIntrinsic:cameraIntrinsics)
                problem.AddParameterBlock(cameraIntrinsic.second.val, 4);

#ifdef DEBUG
            cout << "loading mapPointsPos..." << endl;
#endif
            ceres::LossFunction *lossFunction = new ceres::HuberLoss(4);
            for (auto &mapPointPos:mapPointsPos) {
                for (auto &observedFrame:mapPointPos.first->observedFrames) {
                    ceres::CostFunction *costFunction =
                            new ceres::AutoDiffCostFunction<ReprojectCost, 2, 4, 6, 3>(
                                    new ReprojectCost(observedFrame.second));
                    problem.AddResidualBlock(
                            costFunction,
                            lossFunction,
                            cameraIntrinsics[observedFrame.first->camera].val,            // Intrinsic
                            frameExtrinsics[observedFrame.first].val,  // View Rotation and Translation
                            mapPointPos.second.val          // Point in 3D space
                    );
                }
            }

#ifdef DEBUG
            cout << "solving BA..." << endl;
#endif
            ceres::Solver::Options ceres_config_options;
            ceres_config_options.minimizer_progress_to_stdout = false;
            ceres_config_options.logging_type = ceres::SILENT;
            ceres_config_options.num_threads = 1;
            ceres_config_options.preconditioner_type = ceres::JACOBI;
            ceres_config_options.linear_solver_type = ceres::SPARSE_SCHUR;
            ceres_config_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;

            ceres::Solver::Summary summary;
            ceres::Solve(ceres_config_options, &problem, &summary);

            if (!summary.IsSolutionUsable()) {
                std::cout << "Bundle Adjustment failed." << std::endl;
            } else {
                // Display statistics about the minimization
                std::cout << std::endl
                          << "Bundle Adjustment statistics (approximated RMSE):\n"
                          << " #views: " << frameExtrinsics.size() << "\n"
                          << " #residuals: " << summary.num_residuals << "\n"
                          << " Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
                          << " Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
                          << " Time (s): " << summary.total_time_in_seconds << "\n"
                          << std::endl;
            }
        }

        void writeMap() {
#ifdef DEBUG
            cout << endl << "==============BA:writing map==============" << endl << endl;
#endif
            //写mapPointsPos
            for (auto &mapPointPos:mapPointsPos) {
                mapPointPos.first->setPos(mapPointPos.second);
            }
            //写frameExtrinsics
            for (auto &frameExtrinsic:frameExtrinsics) {
                frameExtrinsic.first->setTcw(frameExtrinsic.second);
            }
            //写cameraIntrinsics
            for (auto &cameraIntrinsic:cameraIntrinsics) {
                cameraIntrinsic.first->setIntrinsic(cameraIntrinsic.second);
            }
        }

    };

}


#endif //SFM_LEARN_BA_H
