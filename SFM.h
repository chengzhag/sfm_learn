//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_SFM_H
#define SFM_LEARN_SFM_H

#include "common_include.h"
#include "Map.h"


namespace sky {

    class SFM {
        Map::Ptr map;

    public:
        void addImages(const vector<string> &imagesDir, Camera::Ptr camera);

    };

}


#endif //SFM_LEARN_SFM_H
