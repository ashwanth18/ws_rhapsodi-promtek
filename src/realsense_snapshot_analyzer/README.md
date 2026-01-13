## realsense_snapshot_analyzer (ROS 2 Jazzy)

This package exposes a **service** that captures the **next** synced pair of:

- `sensor_msgs/Image` (color)
- `sensor_msgs/Image` (aligned depth)
- `sensor_msgs/PointCloud2` (aligned/organized pointcloud)

Then it saves snapshot artifacts (optional) and returns basic depth/3D statistics.

### Prereqs

- **RealSense driver**: `realsense2_camera` (recommended)
- **Python deps (ROS debs)**: `cv_bridge`, `message_filters`
- **System deps**: `numpy`, `opencv`

### 1) Start the RealSense camera

Recommended: enable pointcloud and align depth to color so pixel↔point mapping is consistent.

- Example (typical):
  - `ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true`

Verify topics:

- Color image: `/camera/camera/color/image_raw`
- Depth image: `/camera/camera/depth/image_rect_raw`
- Pointcloud (registered to color): `/camera/camera/depth/color/points`

### 2) Build

From your workspace root:

- `colcon build --packages-select realsense_snapshot_analyzer`
- `source install/setup.bash`

### 3) Run

- `ros2 launch realsense_snapshot_analyzer snapshot_analyzer.launch.py`

Service name:

- `/<node_namespace>/capture` (by default `/realsense_snapshot_analyzer/capture`)

Call it:

- `ros2 service call /realsense_snapshot_analyzer/capture std_srvs/srv/Trigger {}`

Response is JSON in `message`, e.g.:

- `depth_m.min/max/mean/median/p10/p90` are **meters** derived from pointcloud `z`
- `closest_range_m` is the minimum 3D range \(sqrt(x^2+y^2+z^2)\) in the ROI
- Saved artifacts (when enabled): `saved_image`, `saved_depth_raw`, `saved_depth_colormap`, `saved_cloud_ply`

### Visualize the saved 3D data

- `saved_depth_colormap`: open the PNG to see a depth heatmap.
- `saved_depth_raw`: 16-bit PNG in **millimeters** (good for quantitative inspection).
- `saved_cloud_ply`: open in RViz2 / CloudCompare / MeshLab to see the powder bed in 3D.

### Where files are stored

By default (via the launch file), snapshots are saved under this package’s share
directory:

- `.../share/realsense_snapshot_analyzer/snapshots/`

You can override it by setting the `save_dir` parameter.

Note: in your current topic list, the depth image is
`/camera/camera/depth/image_rect_raw` (not `/aligned_depth_to_color/image_raw`).
The node can still save/colormap it; the PLY is the best artifact for accurate 3D
visualization of the powder bed.

### 4) ROI (region of interest)

The node supports ROI parameters (pixels) when the pointcloud is **organized** (`height>1`).

Set:

- `roi_x`, `roi_y`, `roi_w`, `roi_h`

If `roi_w<=0` or `roi_h<=0`, the node uses the full frame.

### How to analyze depth/pointcloud data (patterns)

You typically do one (or more) of these:

- **Distance-to-object / nearest obstacle**:
  - Use `min(range)` or `min(z)` in an ROI (already implemented).
- **Surface estimation (plane fit)**:
  - Fit a plane to points in an ROI using RANSAC, then compute distance-to-plane.
- **Volume / fill level**:
  - Segment the container region, estimate surface height from `z` distribution.
- **Outlier rejection**:
  - Drop invalid points (`nan`, `z<=0`), then apply percentile-clipping (e.g., 5–95%).

Implementation entrypoint:

- `realsense_snapshot_analyzer/snapshot_analyzer_node.py`


