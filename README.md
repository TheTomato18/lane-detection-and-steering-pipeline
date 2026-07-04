# Lane Detection + Steering Pipeline

A three-stage perception and control pipeline for processing dashcam
footage. The script detects lane lines and computes a steering correction
that keeps the vehicle centered in the lane.

The project is organised to reflect the same capture, perception, and
control separation used in autonomous vehicle software stacks.

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

The stages are implemented as separate components so the structure maps
directly to a ROS2 node graph:

```
[Camera Node] --"frames"--> [Perception Node] --"lane_offset"--> [Control Node] --"steering_cmd"-->
```

## Usage

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

## Known limitations / next steps

- Straight-line Hough detection is less effective on sharp curves. A
  polynomial or sliding-window approach would generalize better.
- The steering angle is a geometric estimate derived from pixel offset and
  an assumed look-ahead distance. It is not calibrated against camera
  intrinsics or vehicle geometry.
- The controller is proportional-only. Adding integral and derivative
  terms would reduce oscillation.
- Future work includes porting each stage into a ROS2 node
  (`camera_node`, `perception_node`, `control_node`) communicating over
  topics.

## License

[MIT](LICENSE)
