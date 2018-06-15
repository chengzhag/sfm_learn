//
// Created by pidan1231239 on 18-6-13.
//

#include "SFM.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>

using namespace cv;

namespace sky {

    void SFM::addImages(const vector<string> &imagesDir, Camera::Ptr camera) {

        auto imageDirIt = imagesDir.begin();
        Frame::Ptr frame1(new Frame(camera, imread(*imageDirIt++)));
        Frame::Ptr frame2(new Frame(camera, imread(*imageDirIt++)));

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
        cvv::debugDMatch(frame1->image, keypoints1, frame2->image, keypoints2, goodMatches, CVVISUAL_LOCATION,
                         "2D-2D initialization points matching");

        //求解对极约束
        vector<Point2f> points1;
        vector<Point2f> points2;
        for (auto match:goodMatches) {
            points1.push_back(keypoints1[match.queryIdx].pt);
            points2.push_back(keypoints2[match.trainIdx].pt);
        }
        Mat fundamentalMatrix;
        fundamentalMatrix = findFundamentalMat(points1, points2, CV_FM_RANSAC);
        Mat essentialMatrix;
        essentialMatrix = findEssentialMat(points1, points2,
                                           camera->getFocalLength(),
                                           camera->getPrincipalPoint());
        //设置frame1的初始化se3
        Mat R1 = Mat::eye(3, 3, CV_64FC1), t1 = Mat::zeros(3, 1, CV_64FC1);
        frame1->T_c_w = SE3(
                SO3(R1.at<double>(0, 0), R1.at<double>(1, 0), R1.at<double>(2, 0)),
                Vector3d(t1.at<double>(0, 0), t1.at<double>(1, 0), t1.at<double>(2, 0))
        );
#ifdef DEBUG
        cout << "2D-2D initialization frame1 R: " << R1.size << endl << R1 << endl;
        cout << "2D-2D initialization frame1 t: " << t1.size << endl << t1 << endl;
        cout << "2D-2D initialization frame1 SE3: " << endl << frame1->T_c_w << endl;
#endif
        //解frame2的R、t并计算se3,三角化
        Mat R2, t2, points4d;
/*        recoverPose(essentialMatrix, points1, points2, R2, t2,
                    camera->getFocalLength(), camera->getPrincipalPoint());*/
        recoverPose(essentialMatrix, points1, points2,
                    camera->getIntrinsics(), R2, t2, 0, noArray(),
                    points4d);
        frame2->T_c_w = SE3(
                SO3(R2.at<double>(0, 0), R2.at<double>(1, 0), R2.at<double>(2, 0)),
                Vector3d(t2.at<double>(0, 0), t2.at<double>(1, 0), t2.at<double>(2, 0))
        );
#ifdef DEBUG
        cout << "2D-2D initialization frame2 R: " << R2.size << endl << R2 << endl;
        cout << "2D-2D initialization frame2 t: " << t2.size << endl << t2 << endl;
        cout << "2D-2D initialization frame2 SE3: " << endl << frame2->T_c_w << endl;
#endif

        //可视化初始化点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

        //三角化
        //vector<Point3d> points3d;
/*        Mat proj1(3, 4, CV_32FC1), proj2(3, 4, CV_32FC1);
        proj1(Range(0, 3), Range(0, 3)) = R1;
        proj1.col(3) = t1;
        proj2(Range(0, 3), Range(0, 3)) = R2;
        proj2.col(3) = t2;
        triangulatePoints(proj1,proj2,points1,points2,)*/

        //triangulation(keypoints1, keypoints2, goodMatches, R, t, *camera, points3d);


        frame1 = frame2;


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

    void SFM::triangulation(
            const vector<KeyPoint> &keypoint_1,
            const vector<KeyPoint> &keypoint_2,
            const std::vector<DMatch> &matches,
            const Mat &R, const Mat &t,
            const Camera &camera,
            vector<Point3d> &points) {
        Mat T1 = (Mat_<float>(3, 4) << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0);
        Mat T2 = (Mat_<float>(3, 4) << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0)
        );

        vector<Point2f> pts_1, pts_2;
        for (DMatch m:matches) {
            // 将像素坐标转换至相机坐标
            pts_1.push_back(camera.pixel2normal(keypoint_1[m.queryIdx].pt));
            pts_2.push_back(camera.pixel2normal(keypoint_2[m.trainIdx].pt));
        }

        Mat pts_4d;
        cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

        // 转换成非齐次坐标
        for (int i = 0; i < pts_4d.cols; i++) {
            Mat x = pts_4d.col(i);
            x /= x.at<float>(3, 0); // 归一化
            Point3d p(
                    x.at<float>(0, 0),
                    x.at<float>(1, 0),
                    x.at<float>(2, 0)
            );
            points.push_back(p);
        }
    }

}
