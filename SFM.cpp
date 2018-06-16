//
// Created by pidan1231239 on 18-6-13.
//

#include "SFM.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace cv;

namespace sky {

    void SFM::addImages(const vector<string> &imagesDir, Camera::Ptr camera) {

        auto imageDirIt = imagesDir.begin();
        Mat image1 = imread(*imageDirIt++);
        Mat image2 = imread(*imageDirIt++);

        initialize(image1, image2, camera);

        //3D-2D

        for (; imageDirIt != imagesDir.end();
               ++imageDirIt) {
            Mat image = imread(*imageDirIt);
#ifdef DEBUG
            cout << endl << "==============Adding image: " + *imageDirIt << "==============" << endl;
#endif
            cvv::showImage(image, CVVISUAL_LOCATION, "Adding image: " + *imageDirIt, "");

            step(image, camera);
        }

        //可视化初始化点云
#ifdef CLOUDVIEWER_DEBUG
        map->visInCloudViewer();
#endif
    }

    void SFM::initialize(Mat &image1, Mat &image2, Camera::Ptr camera) {
        //2D-2D
#ifdef DEBUG
        cout << endl << "==============2D-2D initializing==============" << endl;
#endif
        //检测特征点并匹配
        pushImage(image1, camera);
        detectAndCompute();
        pushImage(image2, camera);
        detectAndCompute();
        //筛选匹配点
        matchAndFilt();
        //解对极约束并三角化
        solve2D2DandTriangulate();
        //保存三角化后的点到地图
        convAndAddMappoints();

        //可视化重投影点
#ifdef CVVISUAL_DEBUGMODE

#endif

        //可视化初始化点云
#ifdef CLOUDVIEWER_DEBUG
        map->visInCloudViewer();
#endif


    }

    void SFM::step(Mat &image, const Camera::Ptr &camera) {

        pushImage(image, camera);

        //检测特征点并匹配

        //检测特征点
        detectAndCompute();
        //提取地图的特征点
        Mat descriptorsMap;
        vector<Point3f> points3D;
        for (MapPoint::Ptr &point:map->mapPoints) {
            if (map->frames.back()->isInFrame(point->pos)) {
                points3D.push_back(point->getPosCV());
                descriptorsMap.push_back(point->descriptor);
            }
        }
#ifdef DEBUG
        cout << "found " << points3D.size() << " 3D points in the last frame" << endl;
#endif
        matcher->match(descriptorsMap, keyFrame2->descriptors, matches, cv::noArray());
#ifdef DEBUG
        cout << "found " << matches.size() << " keypoints" << endl;
#endif

        //筛选匹配点
        filtMatches();

        vector<Point2f> points2DPnP;
        vector<Point3f> points3DPnP;
        for (auto match:matches) {
            points2DPnP.push_back(keyFrame2->keyPoints[match.trainIdx].pt);
            points3DPnP.push_back(points3D[match.queryIdx]);
        }
        Mat r, t, indexInliers;
        solvePnPRansac(points3DPnP, points2DPnP, camera->getIntrinsics(),
                       cv::noArray(), r, t, false, 100, 8.0, 0.99,
                       indexInliers);
        Mat R;
        cv::Rodrigues(r, R);
#ifdef DEBUG
        cout << "solvePnPRansac: " << indexInliers.rows << " valid points, " <<
             (float) indexInliers.rows * 100 / points2DPnP.size()
             << "% of " << points2DPnP.size() << " points are used" << endl;
#endif

        //TODO: 局部BA，参考slambook: project/0.4

        keyFrame2->frame->T_c_w = SE3(
                SO3(r.at<double>(0, 0), r.at<double>(1, 0), r.at<double>(2, 0)),
                Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
        );

        //提取最后两帧特征点

        //triangulatePoints(map->frames.back()->getProjMatCV,keyFrame2->frame->getProjMatCV())
        
    }

    void SFM::convAndAddMappoints() {//归一化齐次坐标点,转换Mat
        for (int i = 0; i < points4D.cols; ++i) {
            if (!inlierMask.at<uint8_t>(i, 0))
                continue;
            // 转换齐次坐标
            Mat x = points4D.col(i);
            x /= x.at<double>(3, 0); // 归一化

            //向地图增加点
            //获取描述子
            Mat descriptor = keyFrame2->descriptors.row(matches[i].trainIdx);
            //获取颜色
            Vec3b rgb;
            if (keyFrame1->image.type() == CV_8UC3) {
                rgb = keyFrame1->image.at<Vec3b>(keyFrame2->keyPoints[matches[i].trainIdx].pt);
                swap(rgb[0], rgb[2]);
            } else if (keyFrame1->image.type() == CV_8UC1) {
                cvtColor(keyFrame1->image.at<uint8_t>(keyFrame2->keyPoints[matches[i].trainIdx].pt),
                         rgb,
                         COLOR_GRAY2RGB);
            }
            MapPoint::Ptr mapPoint(new MapPoint(Vector3d(x.at<double>(0, 0),
                                                         x.at<double>(1, 0),
                                                         x.at<double>(2, 0)),
                                                descriptor, rgb, keyFrame1->frame
            ));
            mapPoint->addObervedFrame(keyFrame2->frame);
            map->addMapPoint(mapPoint);
        }
#ifdef DEBUG
        cout << "got " << map->mapPoints.size() << " 3D points" << endl;
#endif
    }

    void SFM::solve2D2DandTriangulate() {//求解对极约束
        Mat essentialMatrix;
        essentialMatrix = findEssentialMat(keyFrame1->matchPoints, keyFrame2->matchPoints,
                                           keyFrame2->frame->camera->getFocalLength(),
                                           keyFrame2->frame->camera->getPrincipalPoint(),
                                           RANSAC, 0.999, 1.0, inlierMask);
#ifdef DEBUG
        int nPointsFindEssentialMat = countNonZero(inlierMask);
        cout << "findEssentialMat: " << nPointsFindEssentialMat << " valid points, " <<
             (float) nPointsFindEssentialMat * 100 / keyFrame1->matchPoints.size()
             << "% of " << keyFrame1->matchPoints.size() << " points are used" << endl;
#endif
        //可视化用于三角化的点
#ifdef CVVISUAL_DEBUGMODE
        vector<DMatch> inlierMatches;
        vector<cv::KeyPoint> inlierkeyFrame1->keyPoints, inlierkeyFrame2->keyPoints;
        for (int i = 0; i < matches.size(); ++i) {
            if (!inlierMask.at<uint8_t>(i, 0))
                continue;
            inlierMatches.push_back(matches[i]);
            inlierMatches.back().trainIdx = inlierkeyFrame1->keyPoints.size();
            inlierMatches.back().queryIdx = inlierkeyFrame2->keyPoints.size();
            inlierkeyFrame1->keyPoints.push_back(keyFrame1->keyPoints[matches[i].queryIdx]);
            inlierkeyFrame2->keyPoints.push_back(keyFrame2->keyPoints[matches[i].trainIdx]);
        }
        cvv::debugDMatch(keyFrame1->image, inlierkeyFrame1->keyPoints, keyFrame2->image, inlierkeyFrame2->keyPoints, inlierMatches, CVVISUAL_LOCATION,
                         "match used in triangulation");
#endif

        //解frame2的R、t并计算se3,三角化
        Mat R, t;
        recoverPose(essentialMatrix, keyFrame1->matchPoints, keyFrame2->matchPoints,
                    keyFrame2->frame->camera->getIntrinsics(), R, t, 100, inlierMask,
                    points4D);
        Eigen::Matrix3d eigenR2;
        cv2eigen(R, eigenR2);
        keyFrame2->frame->T_c_w = SE3(
                eigenR2,
                Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
        );
#ifdef DEBUG
        int nPointsRecoverPose = countNonZero(inlierMask);
        cout << "recoverPose: " << nPointsRecoverPose << " valid points, " <<
             (float) nPointsRecoverPose * 100 / keyFrame1->matchPoints.size()
             << "% of " << keyFrame1->matchPoints.size() << " points are used" << endl;
/*        cout << "2D-2D frame2 R: " << R.size << endl << R << endl;
        cout << "2D-2D frame2 t: " << t.size << endl << t << endl;
        cout << "2D-2D frame2 SE3: " << endl << frame2->T_c_w << endl;
        cout << "2D-2D frame2 Tcw: " << endl << frame2->getTcwCV() << endl << endl;
        cout << "2D-2D frame2 ProjMat: " << endl << frame2->getProjMatCV() << endl << endl;*/
#endif


    }

    void SFM::matchAndFilt() {
        matcher->match(keyFrame1->descriptors, keyFrame2->descriptors, matches, noArray());
#ifdef DEBUG
        cout << "found " << matches.size() << " keypoints" << endl;
#endif
        //筛选匹配点
        filtMatches();

#ifdef CVVISUAL_DEBUGMODE
        cvv::debugDMatch(keyFrame1->image, keyFrame1->keyPoints, keyFrame2->image, keyFrame2->keyPoints, matches, CVVISUAL_LOCATION,
                         "2D-2D points matching");
#endif

        for (auto match:matches) {
            keyFrame1->matchPoints.push_back(keyFrame1->keyPoints[match.queryIdx].pt);
            keyFrame2->matchPoints.push_back(keyFrame2->keyPoints[match.trainIdx].pt);
        }
    }

    void SFM::filtMatches() {
        auto minMaxDis = minmax_element(
                matches.begin(), matches.end(),
                [](const DMatch &m1, const DMatch &m2) {
                    return m1.distance < m2.distance;
                });
        auto minDis = minMaxDis.first->distance;
        auto maxDis = minMaxDis.second->distance;
        vector<DMatch> goodMatches;
        for (auto match:matches) {
            if (match.distance <= 5 * minDis)
                goodMatches.push_back(match);
        }
        matches=goodMatches;
#ifdef DEBUG
        cout << "found " << matches.size() << " good matches" << endl;
#endif
    }

    void SFM::detectAndCompute() {
#ifdef DEBUG
        cout << "finding keypoints and extracting descriptors..." << endl;
#endif
        feature2D->detect(keyFrame2->image, keyFrame2->keyPoints, noArray());
        feature2D->compute(keyFrame2->image, keyFrame2->keyPoints, keyFrame2->descriptors);
    }

    void SFM::pushImage(Mat &image, const Camera::Ptr &camera) {
        //释放中间变量和关键帧
        if (keyFrame1) {
            keyFrame1->image.release();
            keyFrame1->descriptors.release();
        }
        inlierMask.release();
        matches.resize(0);
        points4D.release();
        //加载新帧
        keyFrame1 = keyFrame2;
        Frame::Ptr frame(new Frame(camera, image));
        map->addFrame(frame);
        keyFrame2 = KeyFrame::Ptr(new KeyFrame(frame, image));
    }
}
