//
// Created by pidan1231239 on 18-6-13.
//

#include "Map.h"
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>

namespace sky {

    void Map::addFrame(Frame::Ptr frame) {
        if (frame)
            frames.push_back(frame);
    }

    void Map::addMapPoint(MapPoint::Ptr mapPoint) {
        if (mapPoint)
            mapPoints.push_back(mapPoint);
    }

    void Map::visInCloudViewer() {
#ifdef CLOUDVIEWER_DEBUG
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (auto point:mapPoints) {
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
    }

}

