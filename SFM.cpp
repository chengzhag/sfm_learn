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


        initialize(*imageDirIt++, *imageDirIt++, camera);

        //3D-2D

        for (; imageDirIt != imagesDir.end();
               ++imageDirIt) {
#ifdef DEBUG
            cout << endl << "==============Adding image: " + *imageDirIt << "==============" << endl;
#endif
            Mat image = imread(*imageDirIt);
            cvv::showImage(image, CVVISUAL_LOCATION, "Adding image: " + *imageDirIt, "");
            Frame::Ptr frame(new Frame(camera, image));

            //检测特征点并匹配
#ifdef DEBUG
            cout << endl << "finding keypoints and extracting descriptors..." << endl;
#endif
            //检测当前帧的特征点
            vector<cv::KeyPoint> keypoints;
            vector<DMatch> matches;
            Mat descriptors2;

            feature2D->detect(image, keypoints, cv::noArray());
            feature2D->compute(image, keypoints, descriptors2);

            //提取地图的特征点
            Mat descriptors1;
            vector<MapPoint::Ptr> localMap;
            for (MapPoint::Ptr &point:map->mapPoints) {
                if (map->frames.back()->isInFrame(point->pos)) {
                    localMap.push_back(point);
                    descriptors1.push_back(point->descriptor);
                }
            }
            matcher->match(descriptors1, descriptors2, matches, cv::noArray());
#ifdef DEBUG
            cout << "found " << matches.size() << " keypoints" << endl;
#endif


            map->addFrame(frame);
        }
    }

    void SFM::initialize(const string &dirImage1, const string &dirImage2, Camera::Ptr camera) {
        Mat image1 = imread(dirImage1);
        Mat image2 = imread(dirImage2);
        Frame::Ptr frame1(new Frame(camera, image1));
        Frame::Ptr frame2(new Frame(camera, image2));
        map->addFrame(frame1);
        map->addFrame(frame2);

        vector<cv::KeyPoint> keypoints1, keypoints2;
        vector<DMatch> matches;
        Mat descriptors1, descriptors2;


        //2D-2D
#ifdef DEBUG
        cout << endl << "==============2D-2D initializing==============" << endl;
#endif

        //检测特征点并匹配
#ifdef DEBUG
        cout << endl << "finding keypoints and extracting descriptors..." << endl;
#endif
        feature2D->detect(image1, keypoints1, cv::noArray());
        feature2D->detect(image2, keypoints2, cv::noArray());
        feature2D->compute(image1, keypoints1, descriptors1);
        feature2D->compute(image2, keypoints2, descriptors2);
        matcher->match(descriptors1, descriptors2, matches, cv::noArray());
#ifdef DEBUG
        cout << "found " << matches.size() << " keypoints" << endl;
#endif
        //筛选匹配点
/*        auto minMaxDis = std::minmax_element(
                matches.begin(), matches.end(),
                [](const cv::DMatch &m1, const cv::DMatch &m2) {
                    return m1.distance < m2.distance;
                });
        auto minDis = minMaxDis.first->distance;
        auto maxDis = minMaxDis.second->distance;
        vector<DMatch> goodMatches;
        for (auto match:matches) {
            if (match.distance <= 5 * minDis)
                goodMatches.push_back(match);
        }*/
        vector<DMatch> &goodMatches = matches;
#ifdef DEBUG
        cout << "found " << goodMatches.size() << " good matches" << endl << endl;
#endif
#ifdef CVVISUAL_DEBUGMODE
        cvv::debugDMatch(image1, keypoints1, image2, keypoints2, goodMatches, CVVISUAL_LOCATION,
                         "2D-2D points matching");
#endif



        //求解对极约束
        vector<Point2f> points1;
        vector<Point2f> points2;
        for (auto match:goodMatches) {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }
        Mat essentialMatrix, inlierMask;
        essentialMatrix = findEssentialMat(points1, points2,
                                           camera->getFocalLength(),
                                           camera->getPrincipalPoint(),
                                           RANSAC, 0.999, 1.0, inlierMask);
#ifdef DEBUG
        int nValidPoints = countNonZero(inlierMask);
        cout << "findEssentialMat: " << nValidPoints << " valid points, " <<
             (float) nValidPoints * 100 / points1.size()
             << "% of " << points1.size() << " points are used" << endl << endl;
#endif
        //可视化用于三角化的点
#ifdef CVVISUAL_DEBUGMODE
        vector<DMatch> inlierMatches;
        vector<cv::KeyPoint> inlierKeypoints1, inlierKeypoints2;
        for (int i = 0; i < goodMatches.size(); ++i) {
            if (!inlierMask.at<uint8_t>(i, 0))
                continue;
            inlierMatches.push_back(goodMatches[i]);
            inlierMatches.back().trainIdx = inlierKeypoints1.size();
            inlierMatches.back().queryIdx = inlierKeypoints2.size();
            inlierKeypoints1.push_back(keypoints1[goodMatches[i].queryIdx]);
            inlierKeypoints2.push_back(keypoints2[goodMatches[i].trainIdx]);
        }
        cvv::debugDMatch(image1, inlierKeypoints1, image2, inlierKeypoints2, inlierMatches, CVVISUAL_LOCATION,
                         "match used in triangulation");
#endif

        //解frame2的R、t并计算se3,三角化
        Mat R, t, points4D;
        recoverPose(essentialMatrix, points1, points2,
                    camera->getIntrinsics(), R, t, 100, inlierMask,
                    points4D);
        Eigen::Matrix3d eigenR2;
        cv2eigen(R, eigenR2);
        frame2->T_c_w = SE3(
                eigenR2,
                Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
        );
#ifdef DEBUG
        nValidPoints = countNonZero(inlierMask);
        cout << "recoverPose: " << nValidPoints << " valid points, " <<
             (float) nValidPoints * 100 / points1.size()
             << "% of " << points1.size() << " points are used" << endl << endl;
/*        cout << "2D-2D frame2 R: " << R.size << endl << R << endl;
        cout << "2D-2D frame2 t: " << t.size << endl << t << endl;
        cout << "2D-2D frame2 SE3: " << endl << frame2->T_c_w << endl;*/

        cout << "got" << points4D.cols << " 3D points" << endl;
#endif



        //归一化齐次坐标点,转换Mat
        for (int i = 0; i < points4D.cols; ++i) {
            if (!inlierMask.at<uint8_t>(i, 0))
                continue;
            // 转换齐次坐标
            Mat x = points4D.col(i);
            x /= x.at<double>(3, 0); // 归一化

            //向地图增加点
            //获取描述子
            Mat descriptor = descriptors2.row(goodMatches[i].trainIdx);
            //获取颜色
            Vec3b rgb;
            if (image1.type() == CV_8UC3) {
                rgb = image1.at<Vec3b>(keypoints2[goodMatches[i].trainIdx].pt);
                swap(rgb[0], rgb[2]);
            } else if (image1.type() == CV_8UC1) {
                cvtColor(image1.at<uint8_t>(keypoints2[goodMatches[i].trainIdx].pt),
                         rgb,
                         COLOR_GRAY2RGB);
            }
            MapPoint::Ptr mapPoint(new MapPoint(Vector3d(x.at<double>(0, 0),
                                                         x.at<double>(1, 0),
                                                         x.at<double>(2, 0)),
                                                descriptor, rgb, frame1
            ));
            mapPoint->addObervedFrame(frame2);
            map->addMapPoint(mapPoint);
        }

        //可视化重投影点
#ifdef CVVISUAL_DEBUGMODE

#endif

        //可视化初始化点云
#ifdef CLOUDVIEWER_DEBUG
        map->visInCloudViewer();
#endif


    }
}
