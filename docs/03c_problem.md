What to expect if the fix is correct:

[DBG_FACE] map_pos=(X, Y, ~1.0) — map Z should now be ~1m (face height on wall) instead of ~2.7m
[DBG_FACE] map_n=(nx, ny, ~0) — map normal should be nearly horizontal (small Z component), pointing away from the wall
[DBG_CTRL] approach: -> goal=(...) — approach point should be ~0.8m from the face position in the horizontal plane, not behind the robot
What was wrong and why the fix works:

The /oakd/rgb/preview/depth topic's header.frame_id is oakd_rgb_camera_optical_frame, but in this Gazebo simulation the TF for that frame is stored as if it were the camera body frame (Z=up) rather than the optical frame (Z=forward). So the 2.46m depth (camera Z) was getting transformed to 2.7m world height instead of horizontal distance.

The PointCloud2 /oakd/rgb/preview/depth/points has its 3D points computed correctly by the simulator using the actual camera projection model — no pinhole math from our side needed. The TF for the PC2's frame works correctly (this is what the original dis_tutorial code relied on), so map_pos will now land on the wall in the correct X/Y position.