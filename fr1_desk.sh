#
# Created by ichigoi7e on 17/07/2018.
#

#!/bin/bash

# Run test
./bin/rgbd_tum Vocabulary/ORBvoc.bin test/TUM1.yaml ~/Dataset/rgbd_dataset_freiburg1_desk test/associations/fr1_desk.txt
# Convert .pcd to .ot
./tools/pcd2octomap map.pcd map.ot
# Generate ATE evaluation
./tools/evaluate_ate.py ~/Dataset/rgbd_dataset_freiburg1_desk/groundtruth.txt CameraTrajectory.txt --plot result.png