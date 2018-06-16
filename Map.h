//
// Created by pidan1231239 on 18-6-13.
//

#ifndef SFM_LEARN_MAP_H
#define SFM_LEARN_MAP_H

#include "MapPoint.h"
#include "Frame.h"

namespace sky {

    class Map {
    public:
        typedef shared_ptr<Map> Ptr;
        list<MapPoint::Ptr> mapPoints;        // all landmarks
        list<Frame::Ptr> frames;         // all key-frames

        Map() {}

        void insertFrame(Frame::Ptr frame);

        void insertMapPoint(MapPoint::Ptr mapPoint);

        void visInCloudViewer();
    };

}


#endif //SFM_LEARN_MAP_H
