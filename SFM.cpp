//
// Created by pidan1231239 on 18-6-13.
//

#include "SFM.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace cv;

namespace sky {

    void SFM::addImages(const vector<string> &imagesDir, Camera::Ptr camera) {

        auto imageDirIt = imagesDir.begin();
        Frame::Ptr frame1(new Frame(camera, imread(*imageDirIt++)));
        Frame::Ptr frame2(new Frame(camera, imread(*imageDirIt++)));
        map->insertFrame(frame1);
        map->insertFrame(frame2);

        vector<cv::KeyPoint> keypoints1, keypoints2;
        vector<DMatch> matches;
        Mat descriptors1, descriptors2;


        //2D-2D
#ifdef DEBUG
        cout << endl << "2D-2D initializing..." << endl << endl;
#endif

        //检测特征点并匹配
        feature2D->detect(frame1->image, keypoints1, cv::noArray());
        feature2D->detect(frame2->image, keypoints2, cv::noArray());
        feature2D->compute(frame1->image, keypoints1, descriptors1);
        feature2D->compute(frame2->image, keypoints2, descriptors2);
        matcher->match(descriptors1, descriptors2, matches, cv::noArray());
#ifdef DEBUG
        cout << "found " << matches.size() << " keypoints" << endl;
#endif
        //筛选匹配点
        auto minMaxDis = std::minmax_element(
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
        }
#ifdef DEBUG
        cout << "found " << goodMatches.size() << " good matches" << endl;
#endif
        cvv::debugDMatch(frame1->image, keypoints1, frame2->image, keypoints2, goodMatches, CVVISUAL_LOCATION,
                         "2D-2D points matching");




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
        //设置frame1的初始化se3
        Mat R1 = Mat::eye(3, 3, CV_64FC1), t1 = Mat::zeros(3, 1, CV_64FC1);
        frame1->T_c_w = SE3(
                SO3(R1.at<double>(0, 0), R1.at<double>(1, 0), R1.at<double>(2, 0)),
                Vector3d(t1.at<double>(0, 0), t1.at<double>(1, 0), t1.at<double>(2, 0))
        );
#ifdef DEBUG
        int nValidPoints = countNonZero(inlierMask);
        cout << nValidPoints << " valid points, " <<
             (float) nValidPoints * 100 / points1.size()
             << "%of " << points1.size() << " points are used in 'findEssentialMat'" << endl << endl;
        cout << "2D-2D frame1 R: " << R1.size << endl << R1 << endl;
        cout << "2D-2D frame1 t: " << t1.size << endl << t1 << endl;
        cout << "2D-2D frame1 SE3: " << endl << frame1->T_c_w << endl;
#endif
        //解frame2的R、t并计算se3,三角化
        Mat R2, t2, points4D;
        recoverPose(essentialMatrix, points1, points2,
                    camera->getIntrinsics(), R2, t2, 100, inlierMask,
                    points4D);
        frame2->T_c_w = SE3(
                SO3(R2.at<double>(0, 0), R2.at<double>(1, 0), R2.at<double>(2, 0)),
                Vector3d(t2.at<double>(0, 0), t2.at<double>(1, 0), t2.at<double>(2, 0))
        );
#ifdef DEBUG
        nValidPoints = countNonZero(inlierMask);
        cout << nValidPoints << " valid points, " <<
             (float) nValidPoints * 100 / points1.size()
             << "%of " << points1.size() << " points are used in 'recoverPose'" << endl << endl;
        cout << "2D-2D frame2 R: " << R2.size << endl << R2 << endl;
        cout << "2D-2D frame2 t: " << t2.size << endl << t2 << endl;
        cout << "2D-2D frame2 SE3: " << endl << frame2->T_c_w << endl;
        cout << "got 4D points sized " << points4D.size << endl;
#endif



        //归一化齐次坐标点,转换Mat
        vector<Point3f> points3D;
        for (int i = 0; i < points4D.cols; ++i) {
            if (!inlierMask.at<uint8_t>(i, 0))
                continue;
            // 转换齐次坐标
            Mat x = points4D.col(i);
            x /= x.at<double>(3, 0); // 归一化
            Point3d p(
                    x.at<double>(0, 0),
                    x.at<double>(1, 0),
                    x.at<double>(2, 0)
            );
            points3D.push_back(p);

            //向地图增加点
            //获取描述子
            Mat descriptor = descriptors2.row(goodMatches[i].trainIdx);
            //获取颜色
            Vec3b rgb;
            if (frame1->image.type() == CV_8UC3) {
                rgb = frame1->image.at<Vec3b>(keypoints2[goodMatches[i].trainIdx].pt);
                swap(rgb[0],rgb[2]);
            } else if (frame1->image.type() == CV_8UC1) {
                cvtColor(frame1->image.at<uint8_t>(keypoints2[goodMatches[i].trainIdx].pt),
                         rgb,
                         COLOR_GRAY2RGB);
            }
            MapPoint::Ptr mapPoint(new MapPoint(Vector3d(x.at<double>(0, 0),
                                                         x.at<double>(1, 0),
                                                         x.at<double>(2, 0)),
                                                descriptor, rgb, frame1
            ));
            mapPoint->addFrame(frame2);
            map->insertMapPoint(mapPoint);
        }



        //可视化初始化点云
#ifdef DEBUG
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
/*        for (auto point:points3D) {
            pcl::PointXYZ pointXYZ(point.x, point.y, point.z);
            cloud->push_back(pointXYZ);
        }*/
        for (MapPoint::Ptr point:map->mapPoints) {
            pcl::PointXYZRGB pointXYZ(point->rgb[0],point->rgb[1],point->rgb[2]);
            pointXYZ.x=point->pos(0);
            pointXYZ.y=point->pos(1);
            pointXYZ.z=point->pos(2);
            cloud->push_back(pointXYZ);
        }
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(cloud);
        while (!viewer.wasStopped()) {
        }

#endif


        frame1 = frame2;



        //3D-2D

        for (; imageDirIt != imagesDir.end();
               ++imageDirIt) {
            Frame::Ptr frame2(new Frame(camera, imread(*imageDirIt)));

#ifdef DEBUG
            cout << "Adding image: " + *imageDirIt << endl;
#endif
            cvv::debugFilter(frame1->image, frame2->image, CVVISUAL_LOCATION, "Adding image: " + *imageDirIt, "");

            frame1 = frame2;
        }
    }


}
