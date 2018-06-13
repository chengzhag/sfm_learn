#include <iostream>
#include "SFM.h"
#include "common_include.h"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

using namespace sky;

int main() {
    SFM sfm;

    std::cout << "设置数据集路径：" << std::endl;
    string imagesFolder("datasets/fountain_dense_images");
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
            2759.48,2764.16,1520.69,1006.81
    )));

    return 0;
}