#!/bin/bash

# Run test
./bin/rgbd_tum Vocabulary/ORBvoc.bin test/TUM2.yaml ~/Dataset/rgbd_dataset_freiburg2_large_no_loop test/associations/fr2_large_no_loop.txt
# Convert .pcd to .bt
./tools/pcd2octomap map.pcd map.bt
# Generate ATE evaluation
./tools/evaluate_ate.py ~/Dataset/rgbd_dataset_freiburg2_large_no_loop/groundtruth.txt CameraTrajectory.txt --plot result.png