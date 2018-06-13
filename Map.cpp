//
// Created by pidan1231239 on 18-6-13.
//

#include "Map.h"

namespace sky {

    void Map::insertFrame(Frame::Ptr frame) {
        if (!frame)
            frames.push_back(frame);
    }

    void Map::insertMapPoint(MapPoint::Ptr mapPoint) {
        if (!mapPoint)
            mapPoints.push_back(mapPoint);
    }

}

