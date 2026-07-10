"""ROS-free perception and control logic for the lane pipeline.

This module is the algorithmic core lifted from the standalone script
(src/lane_detection.py). It has no ROS dependencies so it can be unit
tested directly; the nodes in this package are thin wrappers that move
data between topics and these classes.
"""

import cv2
import numpy as np


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
    """Perception stage with temporal smoothing across frames.

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
