#!/usr/bin/env python3
"""Track a color target and report its image-plane position.

This module deliberately stops at perception. It does not command the robot;
pipe its JSON output into a separately reviewed control policy after camera and
robot coordinate frames have been calibrated.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

try:
    import cv2
    import numpy as np
except ModuleNotFoundError as error:
    raise SystemExit(
        "Vision dependencies are missing. Install them with:\n"
        "  python -m pip install -r software/vision/requirements.txt"
    ) from error


@dataclass(frozen=True)
class Detection:
    """Largest valid target found in one frame."""

    frame: int
    timestamp_s: float
    detected: bool
    center_x_px: int | None = None
    center_y_px: int | None = None
    offset_x_norm: float | None = None
    offset_y_norm: float | None = None
    area_px: float | None = None
    radius_px: float | None = None
    confidence: float = 0.0


def bounded_int(value: str) -> int:
    parsed = int(value)
    if not 0 <= parsed <= 255:
        raise argparse.ArgumentTypeError("HSV values must be between 0 and 255")
    return parsed


def positive_odd(value: str) -> int:
    parsed = int(value)
    if parsed < 1 or parsed % 2 == 0:
        raise argparse.ArgumentTypeError("value must be a positive odd integer")
    return parsed


def parse_source(value: str) -> int | str:
    """Interpret an integer as a camera index and other values as video paths."""
    try:
        return int(value)
    except ValueError:
        return value


def validate_hsv(lower: Sequence[int], upper: Sequence[int]) -> None:
    if lower[0] > 179 or upper[0] > 179:
        raise ValueError("OpenCV hue values must be between 0 and 179")
    if any(low > high for low, high in zip(lower, upper, strict=True)):
        raise ValueError("each --lower HSV value must be <= its --upper value")


class ColorTracker:
    """Segment an HSV color range and select its largest valid contour."""

    def __init__(
        self,
        lower_hsv: Sequence[int],
        upper_hsv: Sequence[int],
        min_area_px: float,
        blur_size: int,
        morph_size: int,
    ) -> None:
        validate_hsv(lower_hsv, upper_hsv)
        self._lower = np.array(lower_hsv, dtype=np.uint8)
        self._upper = np.array(upper_hsv, dtype=np.uint8)
        self._min_area = min_area_px
        self._blur_size = blur_size
        self._kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (morph_size, morph_size)
        )

    def detect(
        self, frame: np.ndarray, frame_number: int
    ) -> tuple[Detection, np.ndarray]:
        blurred = cv2.GaussianBlur(
            frame, (self._blur_size, self._blur_size), sigmaX=0
        )
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lower, self._upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return self._missing(frame_number), mask

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self._min_area:
            return self._missing(frame_number), mask

        moments = cv2.moments(contour)
        if moments["m00"] == 0.0:
            return self._missing(frame_number), mask

        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        (_, _), radius = cv2.minEnclosingCircle(contour)
        height, width = frame.shape[:2]
        offset_x = (center_x - width / 2.0) / (width / 2.0)
        # Positive Y is up, matching the robot's conventional image-plane view.
        offset_y = (height / 2.0 - center_y) / (height / 2.0)

        # Confidence is a stable size heuristic, not a calibrated probability.
        frame_area = float(width * height)
        confidence = min(1.0, area / max(self._min_area * 4.0, frame_area * 0.02))
        return (
            Detection(
                frame=frame_number,
                timestamp_s=time.time(),
                detected=True,
                center_x_px=center_x,
                center_y_px=center_y,
                offset_x_norm=round(offset_x, 5),
                offset_y_norm=round(offset_y, 5),
                area_px=round(area, 2),
                radius_px=round(float(radius), 2),
                confidence=round(confidence, 4),
            ),
            mask,
        )

    @staticmethod
    def _missing(frame_number: int) -> Detection:
        return Detection(
            frame=frame_number,
            timestamp_s=time.time(),
            detected=False,
        )


def draw_overlay(frame: np.ndarray, detection: Detection, fps: float) -> None:
    height, width = frame.shape[:2]
    center = (width // 2, height // 2)
    cv2.drawMarker(
        frame,
        center,
        (180, 180, 180),
        markerType=cv2.MARKER_CROSS,
        markerSize=24,
        thickness=1,
    )

    status = "TARGET LOST"
    color = (0, 0, 255)
    if detection.detected:
        assert detection.center_x_px is not None
        assert detection.center_y_px is not None
        assert detection.radius_px is not None
        target = (detection.center_x_px, detection.center_y_px)
        color = (0, 220, 0)
        status = (
            f"TARGET dx={detection.offset_x_norm:+.3f} "
            f"dy={detection.offset_y_norm:+.3f}"
        )
        cv2.circle(frame, target, max(2, int(detection.radius_px)), color, 2)
        cv2.circle(frame, target, 4, color, -1)
        cv2.line(frame, center, target, color, 1)

    cv2.putText(
        frame,
        status,
        (16, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        color,
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"{fps:5.1f} FPS | q/esc: quit",
        (16, height - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (235, 235, 235),
        1,
        cv2.LINE_AA,
    )


def emit_detection(detection: Detection, json_lines: bool) -> None:
    if json_lines:
        print(json.dumps(asdict(detection), separators=(",", ":")), flush=True)
        return

    if detection.detected:
        print(
            "target "
            f"x={detection.center_x_px:4d}px y={detection.center_y_px:4d}px "
            f"dx={detection.offset_x_norm:+.3f} dy={detection.offset_y_norm:+.3f} "
            f"area={detection.area_px:.0f}px²",
            flush=True,
        )
    else:
        print("target lost", flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Track the largest object within an HSV color range."
    )
    parser.add_argument(
        "--source",
        default="0",
        help="Camera index or video-file path (default: 0)",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument(
        "--lower",
        nargs=3,
        type=bounded_int,
        metavar=("H", "S", "V"),
        default=(35, 70, 60),
        help="Lower HSV bound; default tracks saturated green",
    )
    parser.add_argument(
        "--upper",
        nargs=3,
        type=bounded_int,
        metavar=("H", "S", "V"),
        default=(90, 255, 255),
        help="Upper HSV bound; OpenCV hue is 0..179",
    )
    parser.add_argument("--min-area", type=float, default=600.0)
    parser.add_argument("--blur", type=positive_odd, default=7)
    parser.add_argument("--morph", type=positive_odd, default=5)
    parser.add_argument("--mirror", action="store_true", help="Mirror the input image")
    parser.add_argument("--show-mask", action="store_true")
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit one machine-readable JSON object per reported frame",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=5,
        help="Report every N frames (default: 5)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable preview windows for pipelines and remote systems",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Stop after N frames; zero runs until interrupted",
    )
    return parser


def run(args: argparse.Namespace) -> int:
    if args.width < 1 or args.height < 1:
        raise ValueError("frame width and height must be positive")
    if args.min_area <= 0:
        raise ValueError("--min-area must be positive")
    if args.print_every < 1:
        raise ValueError("--print-every must be at least 1")
    if args.max_frames < 0:
        raise ValueError("--max-frames cannot be negative")

    source = parse_source(args.source)
    if isinstance(source, str) and not Path(source).is_file():
        raise ValueError(f"video source does not exist: {source}")

    tracker = ColorTracker(
        lower_hsv=args.lower,
        upper_hsv=args.upper,
        min_area_px=args.min_area,
        blur_size=args.blur,
        morph_size=args.morph,
    )
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise RuntimeError(f"could not open video source {args.source!r}")

    if isinstance(source, int):
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    frame_number = 0
    fps = 0.0
    previous_tick = time.perf_counter()

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                if isinstance(source, str):
                    break
                raise RuntimeError("camera stopped returning frames")

            frame_number += 1
            if args.mirror:
                frame = cv2.flip(frame, 1)

            detection, mask = tracker.detect(frame, frame_number)
            now = time.perf_counter()
            instantaneous_fps = 1.0 / max(now - previous_tick, 1.0e-6)
            fps = instantaneous_fps if fps == 0.0 else fps * 0.9 + instantaneous_fps * 0.1
            previous_tick = now

            if frame_number % args.print_every == 0:
                emit_detection(detection, args.json)

            if not args.headless:
                draw_overlay(frame, detection, fps)
                cv2.imshow("mini-dummy tracker", frame)
                if args.show_mask:
                    cv2.imshow("mini-dummy mask", mask)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

            if args.max_frames and frame_number >= args.max_frames:
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()

    return 0


def main() -> int:
    args = build_parser().parse_args()
    try:
        return run(args)
    except (RuntimeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
