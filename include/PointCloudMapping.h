//
// Created by ichigoi7e on 12/07/2018.
//

#ifndef INDOOR_MAPPPING_POINTCLOUD_H
#define INDOOR_MAPPPING_POINTCLOUD_H

#include "System.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>

using namespace indoor_mapping;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping(double resolution_);

    void insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    void shutdown();
    void viewer();

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    PointCloud::Ptr globalMap;
    shared_ptr<thread>  viewerThread;

    bool    shutDownFlag    =false;
    mutex   shutDownMutex;

    condition_variable  keyFrameUpdated;
    mutex               keyFrameUpdateMutex;

    vector<KeyFrame*>       keyframes;
    vector<cv::Mat>         colorImgs;
    vector<cv::Mat>         depthImgs;
    mutex                   keyframeMutex;
    uint16_t                lastKeyframeSize =0;

    double resolution = 0.04;
    pcl::VoxelGrid<PointT>  voxel;
};

#endif //INDOOR_MAPPPING_POINTCLOUD_H