#!/bin/bash

# Run test
./bin/rgbd_tum Vocabulary/ORBvoc.bin test/TUM1.yaml ~/Dataset/rgbd_dataset_freiburg1_room test/associations/fr1_room.txt
# Convert .pcd to .bt
./tools/pcd2octomap map.pcd map.bt
# Generate ATE evaluation
./tools/evaluate_ate.py ~/Dataset/rgbd_dataset_freiburg1_room/groundtruth.txt CameraTrajectory.txt --plot result.png