"""Black-tape offset detection adapted from the supplied line_offset.py."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Result:
    annotated: np.ndarray
    edges: np.ndarray
    mask: np.ndarray
    visible: bool
    offset_px: float
    offset_normalized: float
    center_px: float | None


def detect(frame: np.ndarray, previous_center: float | None = None) -> Result:
    """Find tape at 75% image height; positive offset means tape is right."""
    height, width = frame.shape[:2]
    camera_center = width // 2
    scan_y = int(height * 0.75)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    sobel_x = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(blurred, cv2.CV_32F, 0, 1, ksize=3)
    edges = cv2.convertScaleAbs(cv2.magnitude(sobel_x, sobel_y))

    _, mask = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    band = mask[max(0, scan_y - 4):min(height, scan_y + 5)]
    tape_pixels = (np.mean(band > 0, axis=0) > 0.6).astype(np.int16)
    changes = np.diff(np.pad(tape_pixels, (1, 1)))
    left_edges = np.where(changes == 1)[0]
    right_edges = np.where(changes == -1)[0] - 1
    candidates = []
    for left, right in zip(left_edges, right_edges):
        tape_width = right - left + 1
        if 10 <= tape_width <= width * 0.6:
            candidates.append((left, right, (left + right) / 2, tape_width))

    annotated = frame.copy()
    cv2.line(annotated, (camera_center, 0), (camera_center, height), (0, 255, 0), 2)
    cv2.line(annotated, (0, scan_y), (width, scan_y), (255, 255, 0), 2)

    if candidates:
        target = previous_center if previous_center is not None else camera_center
        left, right, center, tape_width = min(candidates, key=lambda c: abs(c[2] - target))
        offset = center - camera_center
        normalized = offset / (width / 2)
        direction = 'TAPE RIGHT' if offset > 15 else 'TAPE LEFT' if offset < -15 else 'CENTERED'
        color = (0, 255, 0) if direction == 'CENTERED' else (0, 0, 255)
        cv2.circle(annotated, (left, scan_y), 7, (0, 0, 255), -1)
        cv2.circle(annotated, (right, scan_y), 7, (0, 0, 255), -1)
        cv2.circle(annotated, (int(center), scan_y), 8, (255, 0, 0), -1)
        cv2.line(annotated, (camera_center, scan_y), (int(center), scan_y), (0, 255, 255), 3)
        error_text = f'LATERAL ERROR: {offset:+.1f} px'
        details = f'Width: {tape_width}px  Normalized: {normalized:+.3f}'
    else:
        center = None
        offset = float('nan')
        normalized = float('nan')
        direction = 'LINE LOST'
        color = (0, 0, 255)
        error_text = 'NO TAPE DETECTED'
        details = 'Width: --'

    cv2.rectangle(annotated, (0, height - 110), (width, height), (0, 0, 0), -1)
    cv2.putText(annotated, error_text, (15, height - 75), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
    cv2.putText(annotated, direction, (15, height - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    cv2.putText(annotated, details, (15, height - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1)
    return Result(annotated, edges, mask, bool(candidates), offset, normalized, center)
