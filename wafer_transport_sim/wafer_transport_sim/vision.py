"""Image processing without ROS dependencies."""
import cv2
import numpy as np
from .core import clamp


def line_command(image, speed, kp, max_angular, threshold, crop_ratio):
    height, width = image.shape[:2]
    crop = image[int(height * crop_ratio):, :]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, int(threshold))
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
    candidates = [i for i in range(1, count)
                  if stats[i, cv2.CC_STAT_AREA] >= 15 and
                  stats[i, cv2.CC_STAT_HEIGHT] >= crop.shape[0] * 0.35 and
                  stats[i, cv2.CC_STAT_WIDTH] < width * 0.5]
    if not candidates:
        return None
    # Prefer a continuous floor stripe, avoiding isolated dark shadows.
    label = max(candidates, key=lambda i: stats[i, cv2.CC_STAT_HEIGHT])
    error = float(centroids[label][0]) - width / 2
    return speed, clamp(-kp * error, -max_angular, max_angular)


def station_codes(detector, image, minimum_side=70.0):
    ok, decoded, points, _ = detector.detectAndDecodeMulti(image)
    results = [code for code, corners in zip(decoded, points) if code and
            min(np.linalg.norm(corners[i] - corners[(i+1) % 4])
                for i in range(4)) >= minimum_side] if ok else []
    if not results:
        code, corners, _ = detector.detectAndDecode(image)
        if code and corners is not None:
            corners = corners.reshape(4, 2)
            if min(np.linalg.norm(corners[i]-corners[(i+1) % 4]) for i in range(4)) >= minimum_side:
                results.append(code)
    return results
