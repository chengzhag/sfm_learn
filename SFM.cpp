//
// Created by pidan1231239 on 18-6-13.
//

#include "SFM.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>

using namespace cv;

namespace sky {

    void SFM::addImages(const vector<string> &imagesDir, Camera::Ptr camera) {
        Frame::Ptr frame1(new Frame(camera, imread(imagesDir.front())));

        for (auto imageDirIt = imagesDir.begin() + 1;
             imageDirIt != imagesDir.end();
             ++imageDirIt) {
            Frame::Ptr frame2(new Frame(camera, imread(*imageDirIt)));

#ifdef DEBUG
            cout<<"Adding image: "+*imageDirIt<<endl;
#endif
            cvv::debugFilter(frame1->image, frame2->image, CVVISUAL_LOCATION, "Adding image: "+*imageDirIt,"");

            frame1 = frame2;
        }
    }

}
