# Lane Detection + Steering Pipeline

A three-stage perception and control pipeline for processing dashcam
footage. The script detects lane lines and computes a steering correction
that keeps the vehicle centered in the lane.

The project is organised to reflect the same capture, perception, and
control separation used in autonomous vehicle software stacks.

## Demo

![Lane detection and steering demo](docs/demo.gif)

## Pipeline

1. **Capture** — reads video frame by frame.
2. **Perception** — auto-thresholded Canny edge detection (thresholds
   derived from the frame's own median intensity) → region-of-interest
   mask → Hough transform → averages detected segments into a left and
   right lane line, with slope/position filtering to reject noise (bridges,
   guardrails, other vehicles). An exponential moving average smooths each
   lane line across frames, and briefly persists the last known line if a
   frame fails to detect one.
3. **Control** — a proportional controller computes the heading-correction
   angle, in real degrees, between the frame center and the midpoint of the
   two lane lines at a look-ahead point.

The pipeline exists in two forms: a standalone Python script
(`src/lane_detection.py`) and a ROS2 workspace (`ros2_ws/`) where each
stage runs as its own node communicating over topics:

```
[camera_node] --/camera/image_raw--> [perception_node] --/lane_offset--> [control_node] --/steering_cmd-->
```

## Usage (standalone script)

Store input dashcam clips in `data/input/` and generated videos in
`data/output/`.

```bash
pip install opencv-python numpy
python src/lane_detection.py data/input/your_video.mp4
```

CLI flags:

```bash
python src/lane_detection.py data/input/your_video.mp4 --no-preview --kp 1.5 --smoothing 0.15
python src/lane_detection.py --help    # full list of flags
```

| Flag | Default | Meaning |
| --- | --- | --- |
| `--output` | `data/output/output.mp4` | Output video path |
| `--no-preview` | off | Disables the live preview window |
| `--kp` | `1.0` | Steering gain applied to the heading angle |
| `--smoothing` | `0.2` | EMA smoothing factor for lane lines |
| `--max-missed-frames` | `5` | Frames to retain the last known lane before dropping it |

The default output is `data/output/output.mp4`. The rendered video includes
the lane overlay, center-offset markers, and the steering angle in degrees
on each frame. To convert the result to a GIF:

```bash
ffmpeg -i data/output/output.mp4 -vf "fps=10,scale=480:-1" data/output/output.gif
```

## Usage (ROS2)

The `ros2_ws/` workspace contains two packages:

- **`lane_interfaces`** — the custom `LaneOffset` message carrying the
  detected lane-center position from perception to control.
- **`lane_pipeline`** — the nodes. The algorithmic core lives in
  `lane_pipeline/lane_detection_core.py` with no ROS dependencies; the
  nodes are thin wrappers around it.

| Node | Subscribes | Publishes |
| --- | --- | --- |
| `camera_node` | — | `/camera/image_raw` (`sensor_msgs/Image`) |
| `perception_node` | `/camera/image_raw` | `/lane_offset` (`lane_interfaces/LaneOffset`), `/lane_overlay` (`sensor_msgs/Image`) |
| `control_node` | `/lane_offset` | `/steering_cmd` (`std_msgs/Float32`, degrees) |
| `visualizer_node` (optional) | `/lane_overlay`, `/steering_cmd` | preview window / recorded mp4 |

### Docker

No local ROS2 install is required — just Docker on the host. The workspace
builds and runs the same way inside the official `ros:jazzy-perception-noble`
image (the same one `.devcontainer/` uses). This example builds the workspace
and runs the full pipeline against the bundled
`data/input/minute-drive-curve.mp4` clip, recording the annotated output
instead of opening a preview window (a plain container has no display to
show one on):

```bash
# pull the image once (~2 GB, only needed the first time)
docker pull ros:jazzy-perception-noble

# from the repo root, on the host
docker run -it --rm -v "$(pwd):/workspace" -w /workspace ros:jazzy-perception-noble bash
```

Then, inside the container:

```bash
apt-get update && apt-get install -y python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y

cd ros2_ws
colcon build
source install/setup.bash

ros2 launch lane_pipeline pipeline.launch.py \
  video_path:=$(pwd)/../data/input/minute-drive-curve.mp4 \
  show_preview:=false \
  output_path:=$(pwd)/../data/output/ros2_output.mp4
```

The pipeline shuts down on its own once the clip ends (add `loop:=true` to
loop it instead, and Ctrl+C to stop). Because the repo root is bind-mounted,
`data/output/ros2_output.mp4` is immediately available back on the host —
open it directly, or convert it to a GIF the same way as the standalone
script's output:

```bash
ffmpeg -i data/output/ros2_output.mp4 -vf "fps=10,scale=480:-1" data/output/ros2_output.gif
```

Paths above use `$(pwd)` deliberately: the manual `docker run` step mounts
the repo at `/workspace`, but if you're working in VS Code,
`.devcontainer/devcontainer.json` wraps the same image with `rosdep`
already configured and "Reopen in Container" mounts the repo at
`/workspaces/<repo-name>` instead — a hardcoded `/workspace/...` path will
not resolve there. Use "Reopen in Container" instead of the manual `docker
run` step above, then run the `colcon build`/`ros2 launch` commands in its
integrated terminal; `$(pwd)`-relative paths work correctly under either
mount point.

### Native (Ubuntu/WSL2)

Build and run directly if ROS2 Humble or newer is already installed
(requires `ros-<distro>-cv-bridge`):

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 launch lane_pipeline pipeline.launch.py video_path:=$(pwd)/../data/input/your_video.mp4
```

Launch arguments mirror the script's CLI flags:

```bash
ros2 launch lane_pipeline pipeline.launch.py \
  video_path:=/abs/path/video.mp4 \
  kp:=1.5 smoothing:=0.15 max_missed_frames:=5 \
  loop:=true show_preview:=false output_path:=/abs/path/output.mp4
```

Individual nodes can also be run and inspected separately:

```bash
ros2 run lane_pipeline camera_node --ros-args -p video_path:=/abs/path/video.mp4
ros2 topic echo /steering_cmd
ros2 run rqt_image_view rqt_image_view /lane_overlay
```

`show_preview:=true` and `rqt_image_view` both need a display, so they work
natively or over WSL2/X11 forwarding but not in a plain `docker run` container
— use `output_path` there instead, as in the Docker example above.

## Known limitations / next steps

- Straight-line Hough detection is less effective on sharp curves. A
  polynomial or sliding-window approach would generalize better.
- The steering angle is a geometric estimate derived from pixel offset and
  an assumed look-ahead distance. It is not calibrated against camera
  intrinsics or vehicle geometry.
- The controller is proportional-only. Adding integral and derivative
  terms would reduce oscillation.
- The ROS2 control node publishes a bare `Float32` steering angle; on a
  real vehicle this would map onto `ackermann_msgs/AckermannDriveStamped`.

## License

[MIT](LICENSE)
