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
        //检测特征点并匹配
        feature2D->detect(frame1->image, keypoints1, cv::noArray());
        feature2D->detect(frame2->image, keypoints2, cv::noArray());
        feature2D->compute(frame1->image, keypoints1, descriptors1);
        feature2D->compute(frame2->image, keypoints2, descriptors2);
        matcher->match(descriptors1, descriptors2, matches, cv::noArray());
        cvv::debugDMatch(frame1->image, keypoints1, frame2->image, keypoints2, matches, CVVISUAL_LOCATION, "初始化特征点匹配");
        //TODO:筛选匹配点

        //求解对极约束
        vector<Point2f> points1;
        vector<Point2f> points2;
        for (int i = 0; i < (int) matches.size(); i++) {
            points1.push_back(keypoints1[matches[i].queryIdx].pt);
            points2.push_back(keypoints2[matches[i].trainIdx].pt);
        }
        Mat fundamentalMatrix;
        fundamentalMatrix = findFundamentalMat(points1, points2, CV_FM_RANSAC);
        Mat essentialMatrix;
        essentialMatrix = findEssentialMat(points1, points2,
                                           camera->getFocalLength(),
                                           camera->getPrincipalPoint());
        Mat R, t;
        recoverPose(essentialMatrix, points1, points2, R, t,
                    camera->getFocalLength(), camera->getPrincipalPoint());

        //三角化
        vector<Point3d> points3d;
        triangulation(keypoints1, keypoints2, matches, R, t, *camera, points3d);


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
