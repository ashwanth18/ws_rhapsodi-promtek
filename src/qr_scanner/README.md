# qr_scanner

ROS 2 (Humble) action server that scans incoming camera images for QR codes using OpenCV + ZBar, with optional preprocessing to handle white/high-key backgrounds.

## What this package provides
- A node `scan_qr_server_node` that starts an action server at `/scan_qr`.
- A live debug image stream `/scan_qr/debug_image` so you can see detections and tuning effects in RViz/rqt_image_view.
- Tunable preprocessing to improve robustness when codes sit on bright/white paper.

## How it works (high level)
1) Subscribes to the configured `image_topic` (BGR8 expected; converted to grayscale internally).
2) If `always_scan=true`, it continuously runs a lightweight decode pass in the image callback to draw green polygons around detected codes on the debug image.
3) On an action goal:
   - A worker thread waits for the latest frame, applies optional preprocessing (CLAHE + adaptive threshold + optional morphological close), and tries ZBar.
   - If ZBar fails and `try_opencv=true`, it falls back to OpenCV’s `QRCodeDetector`.
   - Feedback reports `detected_lot` and a heuristic `confidence` in [0..1]; result returns `match`, `lot`, and a `message` (Matched/Mismatched/Timeout/Canceled).

## Node & Interfaces
- Node executable: `scan_qr_server_node`
- Action type: `robot_common_msgs/action/ScanQr`
  - Goal: `string expected_lot`
  - Feedback: `string detected_lot`, `float32 confidence`
  - Result: `bool match`, `string lot`, `string message`

## Parameters
- `image_topic` (string, default `/camera/camera/color/image_raw`): input topic.
- `publish_debug_image` (bool, default `true`): publish `/scan_qr/debug_image`.
- `always_scan` (bool, default `false`): overlay detections even without an active goal.
- `enable_preproc` (bool, default `false`): enable preprocessing path for white backgrounds.
- `preproc_mode` (string, default `clahe_adapt`): current mode: CLAHE + adaptive threshold.
- `clahe_clip` (float, default `2.0`), `clahe_grid` (int, default `8`): CLAHE settings.
- `thr_block_size` (odd int, default `21`), `thr_C` (int, default `5`): adaptive threshold settings.
- `morph_close` (int, default `3`): morphological close kernel size; `0` disables.
- `try_opencv` (bool, default `true`): try OpenCV `QRCodeDetector` if ZBar fails.
- `min_confidence` (float [0..1], default `0.0`): filter out low-confidence results.

## Topics
- Subscribes: `image_topic` (`sensor_msgs/Image`)
- Publishes:
  - `/scan_qr/debug_image` (`sensor_msgs/Image`) — raw feed with green polygon overlays for detections.
  - `/scan_qr/camera_info` (`sensor_msgs/CameraInfo`) — minimal width/height to support RViz Image/Camera displays.

## Run & test
1) Start camera (example RealSense):
```
ros2 launch realsense2_camera rs_camera.launch.py
```
2) Start scanner (white background–tuned settings):
```
ros2 run qr_scanner scan_qr_server_node --ros-args \
  -p image_topic:=/camera/camera/color/image_raw \
  -p publish_debug_image:=true \
  -p always_scan:=true \
  -p enable_preproc:=true -p preproc_mode:=clahe_adapt \
  -p clahe_clip:=2.0 -p clahe_grid:=8 \
  -p thr_block_size:=21 -p thr_C:=5 \
  -p morph_close:=3 -p try_opencv:=true -p min_confidence:=0.2
```
3) Visualize:
- RViz: Image display → `/scan_qr/debug_image` (Transport: raw). Or use rqt_image_view.
4) Send an action goal:
```
ros2 action send_goal /scan_qr robot_common_msgs/action/ScanQr "{expected_lot: 'LOT123'}" --feedback
```

## Tuning for white backgrounds
- Camera: disable auto-exposure/auto-gain, reduce exposure/gain; disable AWB if needed. Avoid direct glare.
- Preproc: start with the defaults above; tweak:
  - `clahe_clip` (1.5–3.0), `clahe_grid` (4–8)
  - `thr_block_size` (17–31), `thr_C` (2–7)
  - `morph_close` (0–5)
- If ZBar still fails, keep `try_opencv=true` for a second-chance decode.

## Troubleshooting
- “No image in RViz”: ensure `/scan_qr/debug_image` is published (set `publish_debug_image=true`).
- RViz Camera display complains about CameraInfo: use the node’s `/scan_qr/camera_info`, or point to the upstream camera’s `camera_info`.
- Action times out: check the `image_topic` is correct and frames are arriving. Try `always_scan=true` to confirm live overlays.
- Weak detection: verify lighting/exposure, then enable preprocessing and tune params as above.

## Development
Build this package only:
```
colcon build --packages-select qr_scanner
source install/setup.bash
```
Record a short bag for offline tuning:
```
ros2 bag record -o qr_debug /camera/color/image_raw
```
Then replay while running the node with different params.




