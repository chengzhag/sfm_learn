#include <iostream>
#include "SFM.h"
#include "common_include.h"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/xfeatures2d.hpp>

using namespace sky;
using namespace cv;

int main() {
    //SFM sfm(ORB::create(500,1.2,8));
    SFM sfm(xfeatures2d::SIFT::create(0, 3, 0.04, 10),
            DescriptorMatcher::create("BruteForce"));

    string imagesFolder("datasets/fountain_dense_images");
#ifdef DEBUG
    std::cout << "Adding images from: " + imagesFolder << std::endl;
#endif

    vector<string> imagesDir;

    if (!imagesFolder.empty()) {
        using namespace boost::filesystem;

        path dirPath(imagesFolder);
        if (not exists(dirPath) or not is_directory(dirPath)) {
            cerr << "Cannot open directory: " << imagesFolder << endl;
            return false;
        }

        for (directory_entry &x : directory_iterator(dirPath)) {
            string extension = x.path().extension().string();
            boost::algorithm::to_lower(extension);
            if (extension == ".jpg" or extension == ".png") {
                imagesDir.push_back(x.path().string());
            }
        }

        if (imagesDir.size() <= 0) {
            cerr << "Unable to find valid files in images directory (\"" << imagesFolder << "\")." << endl;
            return false;
        }

        sort(imagesDir.begin(), imagesDir.end());
    }

    sfm.addImages(imagesDir, Camera::Ptr(new Camera(
            2759.48, 2764.16, 1520.69, 1006.81
    )));

    cvv::finalShow();

    return 0;
}