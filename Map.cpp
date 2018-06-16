//
// Created by pidan1231239 on 18-6-13.
//

#include "Map.h"
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

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

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (auto point:mapPoints) {
            pcl::PointXYZRGB pointXYZ(point->rgb[0], point->rgb[1], point->rgb[2]);
            pointXYZ.x = point->pos(0);
            pointXYZ.y = point->pos(1);
            pointXYZ.z = point->pos(2);
            cloud->push_back(pointXYZ);
        }
        pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
        viewer.showCloud(cloud);
        while (!viewer.wasStopped()) {
        }

/*        pcl::visualization::PCLVisualizer viewer("Viewer");
        viewer.setBackgroundColor(255, 255, 255);
        viewer.addPointCloud(cloud, "Triangulated Point Cloud");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                3,
                                                "Triangulated Point Cloud");
        viewer.addCoordinateSystem(1.0);*/

        /*int indexFrame=0;
        for (auto &frame:frames) {
            Affine3f campose(frame->T_c_w);
            viewer.addCoordinateSystem(1.0, campose, "cam" + to_string(indexFrame++));
        }*/

    }

}

