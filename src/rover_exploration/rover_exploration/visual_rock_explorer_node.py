import math
import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class VisualRockExplorerNode(Node):
    """RGB-D autonomous visual rock explorer.

    This version does NOT use Nav2 for exploration. It avoids the earlier issue where
    the 2-D costmap did not represent hills/mounds as high cost. Instead, it uses the
    live depth image to choose a low-risk valley direction.

    Behavior:
      1. Segment dark gray rocks from RGB.
      2. If a stable rock is visible, visual-servo toward it.
      3. If close enough, stop and save an image.
      4. If no stable rock is visible, choose left/center/right using depth slope,
         roughness, and closeness. Turn toward the safer/flatter sector.
    """

    def __init__(self):
        super().__init__("visual_rock_explorer_node")

        # Topics.
        self.declare_parameter("rgb_topic", "/camera/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("segmentation_topic", "/rock_segmentation/debug_image")
        self.declare_parameter("output_dir", "captured_images")

        # Capture / mission.
        self.declare_parameter("max_goals", 5)
        self.declare_parameter("capture_distance", 1.25)
        self.declare_parameter("post_capture_pause_sec", 1.0)
        self.declare_parameter("post_capture_ignore_sec", 8.0)
        self.declare_parameter("post_capture_escape_sec", 5.0)
        self.declare_parameter("escape_linear_speed", 0.06)
        self.declare_parameter("escape_angular_speed", 0.38)

        # Rock segmentation parameters.
        self.declare_parameter("dark_value_threshold", 130)
        self.declare_parameter("rock_saturation_max", 150)
        self.declare_parameter("rock_red_dominance_max", 35)
        self.declare_parameter("min_blob_area", 1000)
        self.declare_parameter("max_blob_area", 90000)
        self.declare_parameter("relative_blob_area", 0.15)
        self.declare_parameter("max_blobs", 2)
        self.declare_parameter("min_rock_depth", 0.6)
        self.declare_parameter("max_rock_depth", 6.5)
        self.declare_parameter("min_detection_hits", 2)
        self.declare_parameter("target_pixel_tolerance", 0.12)
        self.declare_parameter("target_depth_tolerance", 1.0)

        # Rock approach controller.
        self.declare_parameter("approach_linear_speed", 0.07)
        self.declare_parameter("approach_slow_linear_speed", 0.04)
        self.declare_parameter("approach_angular_gain", 0.9)
        self.declare_parameter("max_approach_angular", 0.45)
        # If robot turns away from rocks, flip this to +1.0.
        self.declare_parameter("approach_angular_sign", -1.0)

        # Valley / terrain exploration controller.
        self.declare_parameter("explore_linear_speed", 0.055)
        self.declare_parameter("explore_turn_linear_speed", 0.055)
        self.declare_parameter("explore_angular_speed", 0.42)
        self.declare_parameter("front_blocked_depth", 2.6)
        self.declare_parameter("sector_safe_depth", 3.2)
        self.declare_parameter("max_depth_for_analysis", 8.0)
        self.declare_parameter("close_fraction_blocked", 0.18)
        self.declare_parameter("roughness_weight", 1.0)
        self.declare_parameter("slope_weight", 2.0)
        self.declare_parameter("close_penalty_weight", 4.0)
        self.declare_parameter("center_bias", 0.15)
        self.declare_parameter("turn_hysteresis", 0.25)
        # In ROS, positive angular.z usually turns left. If your rover is reversed, set -1.0.
        self.declare_parameter("left_turn_sign", 1.0)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.segmentation_topic = self.get_parameter("segmentation_topic").value
        self.output_dir = self.get_parameter("output_dir").value

        self.max_goals = int(self.get_parameter("max_goals").value)
        self.capture_distance = float(self.get_parameter("capture_distance").value)
        self.post_capture_pause_sec = float(self.get_parameter("post_capture_pause_sec").value)
        self.post_capture_ignore_sec = float(self.get_parameter("post_capture_ignore_sec").value)
        self.post_capture_escape_sec = float(self.get_parameter("post_capture_escape_sec").value)
        self.escape_linear_speed = float(self.get_parameter("escape_linear_speed").value)
        self.escape_angular_speed = float(self.get_parameter("escape_angular_speed").value)

        self.dark_value_threshold = int(self.get_parameter("dark_value_threshold").value)
        self.rock_saturation_max = int(self.get_parameter("rock_saturation_max").value)
        self.rock_red_dominance_max = int(self.get_parameter("rock_red_dominance_max").value)
        self.min_blob_area = int(self.get_parameter("min_blob_area").value)
        self.max_blob_area = int(self.get_parameter("max_blob_area").value)
        self.relative_blob_area = float(self.get_parameter("relative_blob_area").value)
        self.max_blobs = int(self.get_parameter("max_blobs").value)
        self.min_rock_depth = float(self.get_parameter("min_rock_depth").value)
        self.max_rock_depth = float(self.get_parameter("max_rock_depth").value)
        self.min_detection_hits = int(self.get_parameter("min_detection_hits").value)
        self.target_pixel_tolerance = float(self.get_parameter("target_pixel_tolerance").value)
        self.target_depth_tolerance = float(self.get_parameter("target_depth_tolerance").value)

        self.approach_linear_speed = float(self.get_parameter("approach_linear_speed").value)
        self.approach_slow_linear_speed = float(self.get_parameter("approach_slow_linear_speed").value)
        self.approach_angular_gain = float(self.get_parameter("approach_angular_gain").value)
        self.max_approach_angular = float(self.get_parameter("max_approach_angular").value)
        self.approach_angular_sign = float(self.get_parameter("approach_angular_sign").value)

        self.explore_linear_speed = float(self.get_parameter("explore_linear_speed").value)
        self.explore_turn_linear_speed = float(self.get_parameter("explore_turn_linear_speed").value)
        self.explore_angular_speed = float(self.get_parameter("explore_angular_speed").value)
        self.front_blocked_depth = float(self.get_parameter("front_blocked_depth").value)
        self.sector_safe_depth = float(self.get_parameter("sector_safe_depth").value)
        self.max_depth_for_analysis = float(self.get_parameter("max_depth_for_analysis").value)
        self.close_fraction_blocked = float(self.get_parameter("close_fraction_blocked").value)
        self.roughness_weight = float(self.get_parameter("roughness_weight").value)
        self.slope_weight = float(self.get_parameter("slope_weight").value)
        self.close_penalty_weight = float(self.get_parameter("close_penalty_weight").value)
        self.center_bias = float(self.get_parameter("center_bias").value)
        self.turn_hysteresis = float(self.get_parameter("turn_hysteresis").value)
        self.left_turn_sign = float(self.get_parameter("left_turn_sign").value)

        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.latest_debug = None
        self.latest_blobs = []
        self.latest_sector_info = {}

        self.current_target = None
        self.target_hits = 0
        self.successful_captures = 0
        self.last_capture_time = None
        self.last_captured_target = None
        self.paused_until = None
        self.ignore_rocks_until = None
        self.escape_until = None
        self.escape_turn_sign = 1.0
        self.last_direction = "center"

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_pub = self.create_publisher(Image, self.segmentation_topic, 10)

        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data)

        self.timer = self.create_timer(0.12, self.control_loop)

        self.get_logger().info("Slope-aware RGB-D rock explorer started.")
        self.get_logger().info("No Nav2 exploration goals are used in this version.")
        self.get_logger().info(f"RGB: {self.rgb_topic}")
        self.get_logger().info(f"Depth: {self.depth_topic}")
        self.get_logger().info(f"Debug segmentation: {self.segmentation_topic}")

    # ----------------------------- ROS callbacks -----------------------------

    def rgb_callback(self, msg):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")
            return
        self.update_segmentation_debug()

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = depth.astype(np.float32)
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")
            return
        self.update_segmentation_debug()

    # ----------------------------- Main control ------------------------------

    def control_loop(self):
        if self.latest_rgb is None or self.latest_depth is None:
            self.stop_robot()
            return

        now = self.get_clock().now()

        # Short stop after taking an image so the saved frame is stable.
        if self.paused_until is not None and now < self.paused_until:
            self.stop_robot()
            return
        self.paused_until = None

        if self.max_goals > 0 and self.successful_captures >= self.max_goals:
            self.stop_robot()
            return

        # After capturing a rock, actively leave the same view instead of
        # repeatedly saving the same rock until max_goals is reached.
        if self.escape_until is not None and now < self.escape_until:
            twist = Twist()
            twist.linear.x = self.escape_linear_speed
            twist.angular.z = self.escape_turn_sign * self.escape_angular_speed
            self.cmd_pub.publish(twist)
            return
        self.escape_until = None

        # During the ignore window, keep exploring terrain but ignore rock blobs.
        # This prevents immediate recapture of the same object.
        if self.ignore_rocks_until is not None and now < self.ignore_rocks_until:
            self.current_target = None
            self.target_hits = 0
            self.explore_by_depth_valley()
            return
        self.ignore_rocks_until = None

        target = self.get_stable_rock_target()

        if target is not None:
            self.visual_servo_to_target(target)
        else:
            self.explore_by_depth_valley()

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    # --------------------------- Rock segmentation ---------------------------

    def segment_rocks(self, rgb):
        h_img, w_img, _ = rgb.shape

        # Ignore sky and extreme bottom band where rover body/shadows often appear.
        y_min = int(0.18 * h_img)
        y_max = int(0.88 * h_img)
        crop = rgb[y_min:y_max, :, :]

        hsv = cv2.cvtColor(crop, cv2.COLOR_RGB2HSV)
        _, sat, val = cv2.split(hsv)
        red = crop[:, :, 0].astype(np.int16)
        green = crop[:, :, 1].astype(np.int16)
        blue = crop[:, :, 2].astype(np.int16)

        red_dominance = red - np.maximum(green, blue)

        # Rock-like in this Mars scene: dark gray/black, not strongly red/orange.
        mask = (
            (val < self.dark_value_threshold)
            & (sat < self.rock_saturation_max)
            & (red_dominance < self.rock_red_dominance_max)
        ).astype(np.uint8) * 255

        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        blobs = []
        center_x = w_img / 2.0

        for label in range(1, num_labels):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < self.min_blob_area or area > self.max_blob_area:
                continue

            x = int(stats[label, cv2.CC_STAT_LEFT])
            y = int(stats[label, cv2.CC_STAT_TOP]) + y_min
            bw = int(stats[label, cv2.CC_STAT_WIDTH])
            bh = int(stats[label, cv2.CC_STAT_HEIGHT])
            cx_crop, cy_crop = centroids[label]
            cx = int(cx_crop)
            cy = int(cy_crop + y_min)

            # Ignore thin terrain edge lines; rocks tend to have some vertical extent.
            if bh < 12 or bw < 12:
                continue

            centrality = 1.0 - min(abs(cx - center_x) / center_x, 1.0)
            score = float(area) * (0.55 + 0.45 * centrality)
            blobs.append({"x": x, "y": y, "w": bw, "h": bh, "cx": cx, "cy": cy, "area": area, "score": score})

        if blobs:
            largest_area = max(b["area"] for b in blobs)
            blobs = [b for b in blobs if b["area"] >= self.relative_blob_area * largest_area]
            blobs.sort(key=lambda b: b["score"], reverse=True)
            blobs = blobs[: self.max_blobs]

        return blobs, mask, y_min, y_max

    def robust_depth_at(self, depth, cx, cy, radius=6):
        h, w = depth.shape[:2]
        x0 = max(0, cx - radius)
        x1 = min(w, cx + radius + 1)
        y0 = max(0, cy - radius)
        y1 = min(h, cy + radius + 1)
        patch = depth[y0:y1, x0:x1]
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > 0.05) & (valid < self.max_depth_for_analysis)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def get_stable_rock_target(self):
        if not self.latest_blobs or self.latest_depth is None:
            self.current_target = None
            self.target_hits = 0
            return None

        candidates = []
        width = self.latest_rgb.shape[1]
        for blob in self.latest_blobs:
            d = self.robust_depth_at(self.latest_depth, blob["cx"], blob["cy"])
            if d is None:
                continue
            if d < self.min_rock_depth or d > self.max_rock_depth:
                continue
            norm_x = (blob["cx"] - width / 2.0) / (width / 2.0)
            candidates.append({**blob, "depth": d, "norm_x": float(norm_x)})

        if not candidates:
            self.current_target = None
            self.target_hits = 0
            return None

        # Prefer large, visible, closer rock-like objects.
        candidates.sort(key=lambda c: c["area"] / max(c["depth"], 0.1), reverse=True)
        target = candidates[0]

        if self.current_target is None:
            self.current_target = target
            self.target_hits = 1
            return None

        same_pixel = abs(target["norm_x"] - self.current_target.get("norm_x", 0.0)) < self.target_pixel_tolerance
        same_depth = abs(target["depth"] - self.current_target.get("depth", target["depth"])) < self.target_depth_tolerance

        if same_pixel and same_depth:
            # Smooth target estimate.
            alpha = 0.65
            target["norm_x"] = alpha * self.current_target["norm_x"] + (1 - alpha) * target["norm_x"]
            target["depth"] = alpha * self.current_target["depth"] + (1 - alpha) * target["depth"]
            self.current_target = target
            self.target_hits += 1
        else:
            self.current_target = target
            self.target_hits = 1
            return None

        if self.target_hits >= self.min_detection_hits:
            return self.current_target

        return None

    def visual_servo_to_target(self, target):
        depth = float(target["depth"])
        norm_x = float(target["norm_x"])

        if depth <= self.capture_distance:
            self.stop_robot()
            self.capture_image(target)
            self.current_target = None
            self.target_hits = 0
            return

        twist = Twist()
        twist.linear.x = self.approach_slow_linear_speed if depth < self.capture_distance + 0.8 else self.approach_linear_speed
        angular = self.approach_angular_sign * self.approach_angular_gain * norm_x
        twist.angular.z = float(np.clip(angular, -self.max_approach_angular, self.max_approach_angular))
        self.cmd_pub.publish(twist)

    def capture_image(self, target=None):
        if self.latest_rgb is None:
            return

        self.successful_captures += 1
        bgr = cv2.cvtColor(self.latest_rgb, cv2.COLOR_RGB2BGR)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.output_dir, f"rock_{self.successful_captures:03d}_{stamp}.png")
        cv2.imwrite(path, bgr)
        self.get_logger().info(f"Captured rock image {self.successful_captures}: {path}")

        now = self.get_clock().now()
        pause_ns = int(self.post_capture_pause_sec * 1e9)
        ignore_ns = int(self.post_capture_ignore_sec * 1e9)
        escape_ns = int((self.post_capture_pause_sec + self.post_capture_escape_sec) * 1e9)

        self.paused_until = now + Duration(nanoseconds=pause_ns)
        self.ignore_rocks_until = now + Duration(nanoseconds=ignore_ns)
        self.escape_until = now + Duration(nanoseconds=escape_ns)

        if target is not None:
            self.last_captured_target = {
                "norm_x": float(target.get("norm_x", 0.0)),
                "depth": float(target.get("depth", 0.0)),
                "time": now,
            }
            # Turn away from the captured rock so the next frames see a new area.
            # norm_x < 0 means rock is left of image center, so turn right.
            if float(target.get("norm_x", 0.0)) < 0.0:
                self.escape_turn_sign = -self.left_turn_sign
            else:
                self.escape_turn_sign = self.left_turn_sign
        else:
            self.escape_turn_sign = self.left_turn_sign

        self.get_logger().info(
            f"Ignoring rock detections for {self.post_capture_ignore_sec:.1f}s and escaping for {self.post_capture_escape_sec:.1f}s."
        )

    # -------------------------- Slope-aware exploration -----------------------

    def valid_depth_values(self, patch):
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > 0.05) & (valid < self.max_depth_for_analysis)]
        return valid

    def sector_metrics(self, depth, name, x0, x1):
        h, _ = depth.shape[:2]

        # Mid-lower camera view: where future ground/terrain appears.
        y0 = int(0.45 * h)
        y1 = int(0.82 * h)
        patch = depth[y0:y1, x0:x1].astype(np.float32)
        valid = self.valid_depth_values(patch)

        if valid.size < 50:
            return {"name": name, "score": -999.0, "median": 0.0, "rough": 999.0, "slope": 999.0, "close_frac": 1.0, "blocked": True}

        median = float(np.median(valid))
        p20 = float(np.percentile(valid, 20))
        mad = float(np.median(np.abs(valid - median)))
        close_frac = float(np.mean(valid < self.front_blocked_depth))

        # Vertical depth gradient. Mounds/slopes tend to create strong depth change patterns.
        # Work on a downsampled finite-filled patch for robustness.
        small = cv2.resize(patch, (max(8, (x1 - x0) // 10), 24), interpolation=cv2.INTER_AREA)
        finite = np.isfinite(small) & (small > 0.05) & (small < self.max_depth_for_analysis)
        if np.any(finite):
            fill_val = float(np.median(small[finite]))
            small = np.where(finite, small, fill_val)
            grad_y = np.abs(np.diff(small, axis=0))
            slope = float(np.median(grad_y))
        else:
            slope = 999.0

        # Compare lower vs upper band. If both are close or lower is very close,
        # the region is risky for this rover even if not a vertical obstacle.
        upper = depth[int(0.42 * h): int(0.58 * h), x0:x1].astype(np.float32)
        lower = depth[int(0.66 * h): int(0.84 * h), x0:x1].astype(np.float32)
        upper_v = self.valid_depth_values(upper)
        lower_v = self.valid_depth_values(lower)
        if upper_v.size > 20 and lower_v.size > 20:
            upper_med = float(np.median(upper_v))
            lower_med = float(np.median(lower_v))
            # Large difference is normal on flat ground, but if lower band is very near
            # while upper is also not far, treat as rising/mound risk.
            rise_risk = max(0.0, self.sector_safe_depth - min(upper_med, lower_med))
        else:
            rise_risk = 1.0

        near_penalty = max(0.0, self.sector_safe_depth - median)
        blocked = (p20 < self.front_blocked_depth) or (close_frac > self.close_fraction_blocked)

        score = median
        score -= self.close_penalty_weight * near_penalty
        score -= self.roughness_weight * mad
        score -= self.slope_weight * slope
        score -= 0.8 * rise_risk
        if name == "center":
            score += self.center_bias
        if blocked:
            score -= 4.0

        return {
            "name": name,
            "score": float(score),
            "median": median,
            "p20": p20,
            "rough": mad,
            "slope": slope,
            "close_frac": close_frac,
            "blocked": bool(blocked),
        }

    def compute_sector_info(self):
        depth = self.latest_depth
        if depth is None:
            return {}
        h, w = depth.shape[:2]
        # Three wide sectors; ignore extreme image edges.
        x_margin = int(0.08 * w)
        usable_w = w - 2 * x_margin
        third = usable_w // 3
        sectors = {
            "left": (x_margin, x_margin + third),
            "center": (x_margin + third, x_margin + 2 * third),
            "right": (x_margin + 2 * third, w - x_margin),
        }
        return {name: self.sector_metrics(depth, name, x0, x1) for name, (x0, x1) in sectors.items()}

    def explore_by_depth_valley(self):
        sector_info = self.compute_sector_info()
        self.latest_sector_info = sector_info
        if not sector_info:
            self.stop_robot()
            return

        left = sector_info["left"]
        center = sector_info["center"]
        right = sector_info["right"]

        # Choose direction with highest slope-aware score.
        best = max(sector_info.values(), key=lambda s: s["score"])

        # Hysteresis: avoid jitter if current direction is still nearly as good.
        if self.last_direction in sector_info:
            current = sector_info[self.last_direction]
            if current["score"] + self.turn_hysteresis >= best["score"] and not current["blocked"]:
                best = current

        self.last_direction = best["name"]

        twist = Twist()
        left_sign = self.left_turn_sign

        if best["name"] == "center" and not center["blocked"]:
            twist.linear.x = self.explore_linear_speed
            twist.angular.z = 0.0
        elif best["name"] == "left":
            twist.linear.x = self.explore_turn_linear_speed
            twist.angular.z = left_sign * self.explore_angular_speed
        elif best["name"] == "right":
            twist.linear.x = self.explore_turn_linear_speed
            twist.angular.z = -left_sign * self.explore_angular_speed
        else:
            # If center is blocked but still scored best due to bad side data, force turn to safer side.
            side = left if left["score"] >= right["score"] else right
            twist.linear.x = self.explore_turn_linear_speed
            twist.angular.z = left_sign * self.explore_angular_speed if side["name"] == "left" else -left_sign * self.explore_angular_speed
            self.last_direction = side["name"]

        self.cmd_pub.publish(twist)

    # ------------------------------ Debug image ------------------------------

    def update_segmentation_debug(self):
        if self.latest_rgb is None:
            return

        rgb = self.latest_rgb.copy()
        blobs, mask, y_min, y_max = self.segment_rocks(rgb)
        self.latest_blobs = blobs

        debug = rgb.copy()
        overlay = np.zeros_like(debug)
        crop_overlay = overlay[y_min:y_max, :, :]
        crop_overlay[mask > 0] = np.array([255, 0, 0], dtype=np.uint8)  # red rock mask
        debug = cv2.addWeighted(debug, 0.70, overlay, 0.30, 0)

        cv2.line(debug, (0, y_min), (debug.shape[1] - 1, y_min), (0, 255, 255), 2)
        cv2.line(debug, (0, y_max), (debug.shape[1] - 1, y_max), (0, 255, 255), 2)

        for blob in blobs:
            x, y, w, h = blob["x"], blob["y"], blob["w"], blob["h"]
            cx, cy = blob["cx"], blob["cy"]
            d = self.robust_depth_at(self.latest_depth, cx, cy) if self.latest_depth is not None else None
            cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(debug, (cx, cy), 4, (0, 0, 255), -1)
            label = f"area={blob['area']}"
            if d is not None:
                label += f" d={d:.2f}"
            cv2.putText(debug, label, (x, max(18, y - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)

        # Draw sector metrics if depth exists.
        if self.latest_depth is not None:
            sector_info = self.compute_sector_info()
            self.latest_sector_info = sector_info
            h_img, w_img = debug.shape[:2]
            y0 = int(0.45 * h_img)
            y1 = int(0.82 * h_img)
            x_margin = int(0.08 * w_img)
            usable_w = w_img - 2 * x_margin
            third = usable_w // 3
            ranges = {
                "left": (x_margin, x_margin + third),
                "center": (x_margin + third, x_margin + 2 * third),
                "right": (x_margin + 2 * third, w_img - x_margin),
            }
            best_name = max(sector_info.values(), key=lambda s: s["score"])["name"] if sector_info else "none"
            for name, (x0, x1) in ranges.items():
                info = sector_info.get(name, {})
                color = (0, 255, 0) if name == best_name else (255, 255, 0)
                if info.get("blocked", False):
                    color = (255, 0, 0)
                cv2.rectangle(debug, (x0, y0), (x1, y1), color, 2)
                text = f"{name[0]} s={info.get('score', 0):.1f} m={info.get('median', 0):.1f} sl={info.get('slope', 0):.2f}"
                cv2.putText(debug, text, (x0 + 3, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.40, color, 1, cv2.LINE_AA)

        cv2.putText(
            debug,
            f"rock mask: V<{self.dark_value_threshold}, S<{self.rock_saturation_max}, Rdom<{self.rock_red_dominance_max}; blobs={len(blobs)}; hits={self.target_hits}; captures={self.successful_captures}",
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, encoding="rgb8")
            self.debug_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Debug image publish failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = VisualRockExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
