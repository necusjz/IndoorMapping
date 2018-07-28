#!/usr/bin/env bash

#
# Created by necusjz on 17/07/2018.
#

# Run test dataset
./bin/rgbd_tum Vocabulary/ORBvoc.bin test/TUM1.yaml Dataset/rgbd_dataset_freiburg1_desk test/associations/fr1_desk.txt

# Convert *.pcd to *.ot
./tools/pcd2octomap map.pcd map.ot

# Plot estimated difference
./tools/evaluate_ate.py Dataset/rgbd_dataset_freiburg1_desk/groundtruth.txt CameraTrajectory.txt --plot result.png

# Print all results
./tools/evaluate_ate.py Dataset/rgbd_dataset_freiburg1_desk/groundtruth.txt CameraTrajectory.txt --verbose
