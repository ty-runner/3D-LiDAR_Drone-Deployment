#!/usr/bin/env python3
"""
profile_classifier_node.py
==========================
Classifies a detected object by matching its vertical profile — the
per-tilt-angle masked point count produced by scan_sequence_node's
PROFILING mode — against a small set of reference templates.

Subscriptions
-------------
/profile_complete  (Float32MultiArray)
    Format: [n_steps, tilt_0, count_0, tilt_1, count_1, ..., tilt_n, count_n]
    Published by scan_sequence_node when a tilt sweep finishes.

/target_bearing_deg (Float32)  — for RViz label placement
/target_range_m     (Float32)  — for RViz label placement

Publications
------------
/profile_label  (String)
    Human-readable classification result, e.g. "person", "box", "pole".

/profile_label_marker  (Marker)
    Text marker displayed in RViz above the detected target.
"""

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


# ── Template definitions ──────────────────────────────────────────────────────
# Each template is a 1-D array of 20 values representing the NORMALISED
# vertical profile shape sampled uniformly across the tilt sweep range.
# Values are in [0, 1]; 1 = widest (most beams) at that height fraction.
#
# These are intentionally simple starting points.  Replace them with profiles
# collected from real objects once you have calibration data.
#
# Height fraction 0 = bottom of sweep, 1 = top of sweep.

_N = 20
_T = np.linspace(0.0, 1.0, _N)  # normalised height axis


def _gaussian(t, mu, sigma):
    return np.exp(-0.5 * ((t - mu) / sigma) ** 2)


TEMPLATES = {
    # Person: broadest around shoulder / torso height (≈40 % from bottom),
    # narrowing toward head and feet.
    "person": _gaussian(_T, mu=0.40, sigma=0.22),

    # Upright box / cylinder: roughly uniform width throughout its height.
    "box": np.clip(np.ones(_N) * 0.88, 0.0, 1.0),

    # Pole or thin vertical object: consistently sparse (few beams) at all
    # heights; the shape is flat but at a low absolute count.
    "pole": np.ones(_N) * 0.20,

    # Cone / pyramid: wide at the base, tapering to the top.
    "cone": np.linspace(1.0, 0.05, _N),
}

# Normalise all templates so classification is purely shape-based, not
# size-based.  A person close-up and a person far away should match the
# same template.
for _k in TEMPLATES:
    _mx = TEMPLATES[_k].max()
    if _mx > 0:
        TEMPLATES[_k] = TEMPLATES[_k] / _mx


class ProfileClassifierNode(Node):

    def __init__(self):
        super().__init__('profile_classifier_node')

        # ── Parameters ────────────────────────────────────────────────────────
        # Minimum total point count across the entire sweep.  Profiles with
        # fewer total points are considered too sparse to classify reliably.
        self.declare_parameter('min_total_points', 15)
        # Minimum normalised similarity score (0–1) to accept a match.
        # Below this the node publishes "unknown".
        self.declare_parameter('min_similarity', 0.70)
        # Marker frame used for the RViz label.
        self.declare_parameter('marker_frame', 'laser_frame')
        # How long the RViz text marker stays visible (seconds).
        self.declare_parameter('marker_lifetime_sec', 3.0)

        self.min_total_pts  = self.get_parameter('min_total_points').value
        self.min_similarity = self.get_parameter('min_similarity').value
        self.marker_frame   = self.get_parameter('marker_frame').value
        self.marker_life    = self.get_parameter('marker_lifetime_sec').value

        # ── State ─────────────────────────────────────────────────────────────
        self.last_bearing_rad = 0.0
        self.last_range_m     = 1.0

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray,
            '/profile_complete',
            self.profile_callback,
            10)

        self.create_subscription(
            Float32,
            '/target_bearing_deg',
            lambda msg: setattr(self, 'last_bearing_rad',
                                float(msg.data) * np.pi / 180.0),
            10)

        self.create_subscription(
            Float32,
            '/target_range_m',
            lambda msg: setattr(self, 'last_range_m', float(msg.data)),
            10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.label_pub  = self.create_publisher(String, '/profile_label', 10)
        self.marker_pub = self.create_publisher(Marker, '/profile_label_marker', 10)

        self.get_logger().info(
            f'ProfileClassifier started | min_sim={self.min_similarity:.2f} '
            f'min_pts={self.min_total_pts}')
        self.get_logger().info(f'Templates loaded: {list(TEMPLATES.keys())}')

    # ── Profile ingestion ─────────────────────────────────────────────────────

    def profile_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) < 3:
            self.get_logger().warn('Received empty or malformed profile, skipping.')
            return

        n_steps = int(data[0])
        expected_len = 1 + n_steps * 2

        if len(data) < expected_len:
            self.get_logger().warn(
                f'Profile length mismatch: expected {expected_len}, got {len(data)}')
            return

        tilt_angles = []
        counts      = []
        for i in range(n_steps):
            tilt_angles.append(data[1 + i * 2])
            counts.append(data[2 + i * 2])

        tilt_angles = np.array(tilt_angles, dtype=np.float32)
        counts      = np.array(counts,      dtype=np.float32)

        self.get_logger().info(
            f'Profile received: {n_steps} steps | '
            f'tilt=[{tilt_angles.min():.0f}°..{tilt_angles.max():.0f}°] | '
            f'total_pts={int(counts.sum())} | per-step={counts.tolist()}')

        label, score = self.classify(tilt_angles, counts)

        self.get_logger().info(f'Classification → "{label}" (score={score:.3f})')
        self._publish_label(label, score)
        self._publish_marker(label, score)

    # ── Classification ────────────────────────────────────────────────────────

    def classify(self, tilt_angles: np.ndarray, counts: np.ndarray):
        """
        Normalise the incoming profile and compute normalised cross-correlation
        against each template after resampling both to a common 20-bin grid.
        Returns (best_label, best_score) where score ∈ [0, 1].
        """
        total_pts = counts.sum()

        if total_pts < self.min_total_pts:
            self.get_logger().warn(
                f'Too few points ({int(total_pts)} < {self.min_total_pts}) to classify.')
            return "unknown", 0.0

        # Resample to the fixed N-bin template grid using linear interpolation.
        t_norm = np.linspace(0.0, 1.0, len(tilt_angles))
        t_grid = np.linspace(0.0, 1.0, _N)
        profile_resampled = np.interp(t_grid, t_norm, counts)

        # Normalise to [0, 1] — shape only, not magnitude.
        p_max = profile_resampled.max()
        if p_max == 0:
            return "unknown", 0.0
        profile_norm = profile_resampled / p_max

        # Score each template via normalised cross-correlation.
        # NCC gives 1.0 for a perfect shape match regardless of scale.
        best_label = "unknown"
        best_score = 0.0

        for label, template in TEMPLATES.items():
            score = self._ncc(profile_norm, template)
            self.get_logger().debug(f'  {label}: {score:.3f}')
            if score > best_score:
                best_score = score
                best_label = label

        if best_score < self.min_similarity:
            return "unknown", best_score

        return best_label, best_score

    @staticmethod
    def _ncc(a: np.ndarray, b: np.ndarray) -> float:
        """Normalised cross-correlation between two equal-length vectors."""
        a = a - a.mean()
        b = b - b.mean()
        denom = np.sqrt((a ** 2).sum() * (b ** 2).sum())
        if denom < 1e-9:
            return 0.0
        return float(np.dot(a, b) / denom)

    # ── Publishing helpers ────────────────────────────────────────────────────

    def _publish_label(self, label: str, score: float):
        msg = String()
        msg.data = f'{label} ({score:.2f})'
        self.label_pub.publish(msg)

    def _publish_marker(self, label: str, score: float):
        """
        Floating text marker in RViz positioned in front of the gimbal at the
        target's last known bearing and range, offset upward by 0.5 m.
        """
        marker = Marker()
        marker.header.frame_id = self.marker_frame
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns              = 'profile_label'
        marker.id              = 0
        marker.type            = Marker.TEXT_VIEW_FACING
        marker.action          = Marker.ADD

        # Position at the target (polar → Cartesian, flat z then offset up).
        marker.pose.position.x = float(self.last_range_m * np.cos(self.last_bearing_rad))
        marker.pose.position.y = float(self.last_range_m * np.sin(self.last_bearing_rad))
        marker.pose.position.z = 0.5          # float above the target sphere
        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.25                  # text height in metres

        # Colour by classification result.
        colour = {
            "person":  (0.2, 1.0, 0.2),        # green
            "box":     (0.2, 0.6, 1.0),         # blue
            "pole":    (1.0, 0.8, 0.0),         # yellow
            "cone":    (1.0, 0.4, 0.0),         # orange
            "unknown": (0.7, 0.7, 0.7),         # grey
        }.get(label, (0.7, 0.7, 0.7))

        marker.color.r = colour[0]
        marker.color.g = colour[1]
        marker.color.b = colour[2]
        marker.color.a = 0.95

        marker.text = f'{label}\n{score:.2f}'

        sec  = int(self.marker_life)
        nsec = int((self.marker_life - sec) * 1e9)
        marker.lifetime = Duration(sec=sec, nanosec=nsec)

        self.marker_pub.publish(marker)

    # ── Calibration helper ───────────────────────────────────────────────────
    # Call this from a ROS service or a separate script once you have real
    # measurements.  Pass in the raw per-tilt counts and the desired label,
    # and it will print the normalised template array you can paste into
    # TEMPLATES above.

    @staticmethod
    def build_template_from_data(tilt_angles: list, counts: list, label: str):
        counts = np.array(counts, dtype=np.float32)
        t_norm = np.linspace(0.0, 1.0, len(tilt_angles))
        t_grid = np.linspace(0.0, 1.0, _N)
        resampled = np.interp(t_grid, t_norm, counts)
        mx = resampled.max()
        if mx > 0:
            resampled /= mx
        print(f'# Template for "{label}":')
        print(f'"{label}": np.array({resampled.tolist()}),')


def main(args=None):
    rclpy.init(args=args)
    node = ProfileClassifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    last_detect_(false)
