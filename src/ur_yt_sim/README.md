# ur_yt_sim

## Camera-guided docking workflow

1. Build the workspace after pulling these changes:
   ```bash
   colcon build --packages-select ur_yt_sim
   source install/setup.bash
   ```
2. Start Gazebo, MoveIt and the ArUco pipeline (RViz optional):
   ```bash
   ros2 launch ur_yt_sim spawn_ur10_camera_gripper_moveit2.launch.py \
     with_aruco:=true \
     with_aruco_docking:=true \
     aruco_docking_marker_id:=0 \
     tool_from_marker_translation:="[0.0, 0.0, 0.0]" \
     tool_from_marker_rpy:="[3.14159, 0.0, 0.0]"
   ```
3. Ensure the overhead camera publishes `/camera/image_raw` and that the `aruco_detector` node sees the desired tag (RViz topic `aruco/pose` helps debug). The docking node waits for the first pose before starting the motion sequence.
   - By default the detector now tracks *all* visible markers and publishes individual poses/TF frames. Pass `aruco_marker_id:=<id>` (e.g. `0`) when launching if the workflow should ignore other tags or if you only want TF for a single marker.
   - The docking node follows the transform broadcast for `aruco_docking_marker_id` (default `0`). Set it to the specific tag you want to approach whenever multiple tags are visible (e.g. keep `aruco_marker_id:=-1` for debugging but set `aruco_docking_marker_id:=2` to dock on tag 2).

### Parameters that matter

- `tool_from_marker_translation` – XYZ offset (metres) from the detected marker frame to the actual docking/contact point. Use this to shift from a corner tag to the docking port centre.
- `tool_from_marker_rpy` – Desired wrist orientation expressed as roll/pitch/yaw offsets relative to the marker frame (default flips the flange so its `-Z` axis aligns with the marker normal).
- `approach_distance` – How far (metres) the pre-dock pose sits along the tool's `-Z` axis before executing the straight Cartesian insertion.

Tuning these three values lets you reuse the same ArUco pose for different docking geometries or gripper orientations without touching the C++ node.
