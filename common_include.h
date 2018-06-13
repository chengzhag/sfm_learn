//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_COMMON_INCLUDE_H
#define SFM_LEARN_COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list
// for Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SO3;
using Sophus::SE3;

// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;

// std
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;
#endif //SFM_LEARN_COMMON_INCLUDE_H
