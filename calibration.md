# Calibration

Possible future options:
* Perform calibration from rgb_to_depth. The justification here is that if we calibrate to the depth camera's frame, then the error profile will be better aligned with our downstream output (good point clouds). However, it's a bit spotty visually, so apriltags may not be visualized properly.


https://calib.io/blogs/knowledge-base/camera-models


https://rosindustrial.org/3d-camera-survey

http://wiki.ros.org/pcl_ros/Tutorials/filters