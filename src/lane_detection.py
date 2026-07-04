"""
Lane Detection + Steering Pipeline
-----------------------------------
A three-stage perception and control pipeline for processing dashcam
footage. The script detects lane lines and computes a steering correction
that keeps the vehicle centered in the lane.

The code is organised to reflect the same capture, perception, and control
separation used in autonomous vehicle software stacks.

    STAGE 1 - capture_frame()      -> "camera node"
    STAGE 2 - LaneDetector.detect() -> "perception node"
    STAGE 3 - compute_steering()   -> "control node"

Usage:
    python src/lane_detection.py path/to/video.mp4
    python src/lane_detection.py path/to/video.mp4 --no-preview --kp 0.03 --smoothing 0.15
    python src/lane_detection.py --help    # full list of tunable flags

Output:
        - A window showing the live pipeline (frame + detected lanes + steering)
        - An output video saved as "data/output/output.mp4" with overlays
            suitable for conversion into a GIF or inclusion in documentation.

Dependencies:
    pip install opencv-python numpy
"""

import argparse
import os
import cv2
import numpy as np


# ---------------------------------------------------------------------------
# STAGE 1: CAPTURE  ("camera node" — publishes raw frames)
# ---------------------------------------------------------------------------
def get_video_capture(path: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise IOError(f"Could not open video file: {path}")
    return cap


# ---------------------------------------------------------------------------
# STAGE 2: PERCEPTION  ("lane detection node" — subscribes to frames,
#                        publishes lane_offset)
# ---------------------------------------------------------------------------
def auto_canny(gray: np.ndarray, sigma: float = 0.33) -> np.ndarray:
    """Pick Canny thresholds from the frame's own median intensity instead
    of fixed constants, so edge detection adapts to lighting/footage."""
    median = float(np.median(gray))
    lower = int(max(0, (1.0 - sigma) * median))
    upper = int(min(255, (1.0 + sigma) * median))
    return cv2.Canny(gray, lower, upper)


def region_of_interest(edges: np.ndarray) -> np.ndarray:
    """Mask out everything except a trapezoid covering the road ahead."""
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # Trapezoid roughly covering the lower half of the frame, narrowing
    # toward the horizon. These ratios are footage-dependent and may
    # require adjustment for different camera mounts or fields of view.
    polygon = np.array([[
        (int(0.05 * width), height),
        (int(0.40 * width), int(0.55 * height)),
        (int(0.60 * width), int(0.55 * height)),
        (int(0.95 * width), height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    return cv2.bitwise_and(edges, mask)


def average_slope_intercept(lines, width):
    """Split raw Hough line segments into a left-lane line and a
    right-lane line by slope sign, then average each group into one line."""
    left_fit, right_fit = [], []
    min_slope = 0.3   # discard near-horizontal noise (bridges, guardrails, cars)
    center_x = width / 2

    if lines is None:
        return None, None

    for line in lines:
        x1, y1, x2, y2 = np.asarray(line).reshape(-1)[:4]
        if x2 == x1:
            continue  # skip vertical lines, undefined slope
        slope, intercept = np.polyfit((x1, x2), (y1, y2), 1)
        if abs(slope) < min_slope:
            continue  # too flat to be a lane line, likely noise

        if slope < 0 and max(x1, x2) <= center_x:      # left lane: negative slope, left half
            left_fit.append((slope, intercept))
        elif slope > 0 and min(x1, x2) >= center_x:    # right lane: positive slope, right half
            right_fit.append((slope, intercept))

    left_avg = np.mean(left_fit, axis=0) if left_fit else None
    right_avg = np.mean(right_fit, axis=0) if right_fit else None
    return left_avg, right_avg


def make_line_points(y1, y2, line_params):
    if line_params is None:
        return None
    slope, intercept = line_params
    if slope == 0:
        slope = 0.1  # avoid div-by-zero
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return x1, y1, x2, y2


class LaneDetector:
    """Perception node with temporal smoothing across frames.

    Raw per-frame (slope, intercept) estimates are noisy; a single
    inaccurate Hough result can produce a large frame-to-frame deviation
    in a lane line. This class maintains an exponential moving average of
    each lane's parameters and briefly reuses the last known line if a
    frame fails to detect one, rather than dropping it entirely.
    """

    def __init__(self, smoothing: float = 0.2, max_missed_frames: int = 5):
        self.smoothing = smoothing            # 0 = no update, 1 = no smoothing
        self.max_missed_frames = max_missed_frames
        self.left_avg = None
        self.right_avg = None
        self.left_missed = 0
        self.right_missed = 0

    def _smooth(self, prev, new):
        if new is None:
            return prev
        if prev is None:
            return new
        return self.smoothing * new + (1 - self.smoothing) * prev

    def _update(self, prev, missed, new):
        if new is None:
            missed += 1
            if missed > self.max_missed_frames:
                prev = None
        else:
            missed = 0
        return self._smooth(prev, new), missed

    def detect(self, frame: np.ndarray):
        """
        Returns:
            overlay        - frame with detected lane lines drawn on it
            lane_center_x  - x-pixel of the midpoint between the two lanes
                              (None if a lane wasn't found)
        """
        height, width = frame.shape[:2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = auto_canny(blur)
        cropped_edges = region_of_interest(edges)

        lines = cv2.HoughLinesP(
            cropped_edges, rho=2, theta=np.pi / 180, threshold=50,
            minLineLength=40, maxLineGap=100,
        )

        left_new, right_new = average_slope_intercept(lines, width)
        self.left_avg, self.left_missed = self._update(self.left_avg, self.left_missed, left_new)
        self.right_avg, self.right_missed = self._update(self.right_avg, self.right_missed, right_new)

        y1 = height
        y2 = int(height * 0.6)
        left_line = make_line_points(y1, y2, self.left_avg)
        right_line = make_line_points(y1, y2, self.right_avg)

        overlay = frame.copy()
        lane_center_x = None

        if left_line is not None and right_line is not None:
            # Translucent fill between the two lane lines, easier to read
            # at a glance than two thin lines.
            lane_area = np.array([[
                (left_line[0], left_line[1]), (left_line[2], left_line[3]),
                (right_line[2], right_line[3]), (right_line[0], right_line[1]),
            ]], np.int32)
            fill = overlay.copy()
            cv2.fillPoly(fill, lane_area, (0, 200, 0))
            overlay = cv2.addWeighted(fill, 0.3, overlay, 0.7, 0)

        if left_line is not None:
            cv2.line(overlay, (left_line[0], left_line[1]),
                      (left_line[2], left_line[3]), (0, 255, 0), 5)
        if right_line is not None:
            cv2.line(overlay, (right_line[0], right_line[1]),
                      (right_line[2], right_line[3]), (0, 255, 0), 5)

        if left_line is not None and right_line is not None:
            # Midpoint of the two lane bases (at the bottom of the frame)
            lane_center_x = (left_line[0] + right_line[0]) / 2

        # Offset markers: the car's centerline (blue) vs. the detected
        # lane's centerline (yellow). The distance between them is the
        # input the steering controller acts on.
        marker_y = height - 10
        cv2.drawMarker(overlay, (int(width / 2), marker_y), (255, 0, 0),
                        markerType=cv2.MARKER_TRIANGLE_UP, markerSize=20, thickness=3)
        if lane_center_x is not None:
            cv2.drawMarker(overlay, (int(lane_center_x), marker_y), (0, 255, 255),
                            markerType=cv2.MARKER_TRIANGLE_DOWN, markerSize=20, thickness=3)

        return overlay, lane_center_x


# ---------------------------------------------------------------------------
# STAGE 3: CONTROL  ("steering node" — subscribes to lane_offset,
#                     publishes steering_cmd)
# ---------------------------------------------------------------------------
class SteeringController:
    """A minimal proportional (P) controller. Production controllers
    typically add integral and derivative terms (full PID) to reduce
    steady-state error and oscillation; this is a candidate extension."""

    def __init__(self, kp: float = 1.0):
        self.kp = kp

    def compute_steering(self, frame_width: int, frame_height: int, lane_center_x):
        """Returns the heading-correction angle in degrees: the angle,
        relative to straight-ahead, between the car's centerline and the
        detected lane's centerline at a look-ahead point half a frame-height
        up the road. This value is a geometric estimate rather than a
        physically measured angle, since no camera calibration is applied;
        it is nonetheless expressed in true degrees rather than an
        arbitrary pixel-scaled unit."""
        if lane_center_x is None:
            return 0.0  # no lane detected this frame -> go straight / hold
        image_center_x = frame_width / 2
        x_offset = lane_center_x - image_center_x   # +ve = drift right
        y_offset = frame_height / 2                  # look-ahead distance
        angle_deg = np.degrees(np.arctan2(x_offset, y_offset))
        steering_angle = -self.kp * angle_deg         # steer opposite to drift
        return steering_angle


# ---------------------------------------------------------------------------
# MAIN LOOP — wires the three stages together
# ---------------------------------------------------------------------------
def main(
    video_path: str,
    output_path: str = "data/output/output.mp4",
    show_preview: bool = True,
    kp: float = 1.0,
    smoothing: float = 0.2,
    max_missed_frames: int = 5,
):
    cap = get_video_capture(video_path)
    controller = SteeringController(kp=kp)
    detector = LaneDetector(smoothing=smoothing, max_missed_frames=max_missed_frames)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))  # 0 if unknown (e.g. a live stream)

    output_dir = os.path.dirname(output_path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    frame_count = 0
    try:
        while True:
            ret, frame = cap.read()          # STAGE 1
            if not ret:
                break

            overlay, lane_center_x = detector.detect(frame)       # STAGE 2
            steering_angle = controller.compute_steering(width, height, lane_center_x)  # STAGE 3

            # --- visualize ---
            direction = "LEFT" if steering_angle > 0.5 else "RIGHT" if steering_angle < -0.5 else "STRAIGHT"
            cv2.putText(
                overlay, f"Steering: {steering_angle:+.2f} deg ({direction})",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2,
            )

            writer.write(overlay)

            # Live preview window. Disable with --no-preview when running
            # headless (e.g. WSL without an X server, or a remote server).
            if show_preview:
                cv2.imshow("Lane Detection Pipeline", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            frame_count += 1
            if frame_count % 30 == 0:
                if total_frames:
                    pct = 100 * frame_count / total_frames
                    print(f"\rProcessed {frame_count}/{total_frames} frames ({pct:.0f}%)", end="", flush=True)
                else:
                    print(f"\rProcessed {frame_count} frames", end="", flush=True)
    finally:
        cap.release()
        writer.release()
        cv2.destroyAllWindows()

    print(f"\nProcessed {frame_count} frames. Output saved to {output_path}")


def parse_args():
    parser = argparse.ArgumentParser(description="Lane detection + steering pipeline")
    parser.add_argument("video_path", help="Path to input video file")
    parser.add_argument("--output", default="data/output/output.mp4", help="Path to output video (default: data/output/output.mp4)")
    parser.add_argument("--no-preview", action="store_true", help="Disable the live preview window")
    parser.add_argument("--kp", type=float, default=1.0,
                         help="Steering gain applied to the computed heading angle in degrees (default: 1.0)")
    parser.add_argument("--smoothing", type=float, default=0.2,
                         help="Lane EMA smoothing factor, 0-1; lower = smoother but laggier (default: 0.2)")
    parser.add_argument("--max-missed-frames", type=int, default=5,
                         help="Frames to keep reusing the last known lane before giving up (default: 5)")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    main(
        args.video_path,
        output_path=args.output,
        show_preview=not args.no_preview,
        kp=args.kp,
        smoothing=args.smoothing,
        max_missed_frames=args.max_missed_frames,
    )
