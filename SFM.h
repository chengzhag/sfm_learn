//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_SFM_H
#define SFM_LEARN_SFM_H

#include "common_include.h"
#include "Map.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>


namespace sky {

    using namespace cv;

    class SFM {

        Map::Ptr map;
        cv::Ptr<cv::Feature2D> feature2D;
        cv::Ptr<DescriptorMatcher> matcher;

    public:
        SFM(const cv::Ptr<cv::Feature2D> &feature2D,
            const cv::Ptr<DescriptorMatcher> &matcher
        ) :
                feature2D(feature2D),
                matcher(matcher),
                map(new Map)
        {}

        void addImages(const vector<string> &imagesDir, Camera::Ptr camera);

        void initialize(Mat &image1, Mat &image2, Camera::Ptr camera);
    };

}


#endif //SFM_LEARN_SFM_H
