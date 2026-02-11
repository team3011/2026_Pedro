import cv2
import numpy as np
from apriltag import apriltag

# Initialize detector once (outside runPipeline for performance)
detector = apriltag("tag36h11")

# =============================================================================
# CONFIGURATION - ADJUST THESE VALUES FOR YOUR SETUP
# =============================================================================
# Real-world tag size in your preferred units (e.g., inches, meters, mm)
# FRC 2024 tags are 6.5 inches (165.1mm), FTC tags are typically 2 inches
TAG_SIZE_REAL = 2  # inches (change units as needed, output will match)

# Approximate focal length in pixels for Limelight
# For 640x480: ~550-600 pixels, For 1280x960: ~1100-1200 pixels
# You can calibrate this by measuring a known distance
FOCAL_LENGTH_PIXELS = 580.0

# Image center (will be updated based on actual image size)
IMAGE_CENTER_X = 320  # half of 640
IMAGE_CENTER_Y = 240  # half of 480


def calculate_tag_pixel_size(corners):
    """
    Calculate the apparent size of the tag in pixels.
    Uses the average of the four edge lengths for robustness.
    """
    # corners format: [lb, rb, rt, lt] (left-bottom, right-bottom, right-top, left-top)
    edge_lengths = []
    for i in range(4):
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]
        length = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
        edge_lengths.append(length)

    # Return average edge length
    return sum(edge_lengths) / 4.0


def calculate_distance(tag_pixel_size):
    """
    Calculate distance using similar triangles:
    distance = (real_size * focal_length) / pixel_size
    """
    if tag_pixel_size <= 0:
        return -1.0  # Invalid

    distance = (TAG_SIZE_REAL * FOCAL_LENGTH_PIXELS) / tag_pixel_size
    return distance


def calculate_relative_position(corners, image_width, image_height):
    """
    Calculate the tag center position relative to image center.
    Returns (x_offset, y_offset) where:
    - Positive X = tag is to the RIGHT of center
    - Positive Y = tag is BELOW center (standard image coordinates)
    """
    # Calculate tag center from corners
    center_x = sum(c[0] for c in corners) / 4.0
    center_y = sum(c[1] for c in corners) / 4.0

    # Calculate offset from image center
    img_center_x = image_width / 2.0
    img_center_y = image_height / 2.0

    x_offset = center_x - img_center_x
    y_offset = center_y - img_center_y

    return center_x, center_y, x_offset, y_offset


def runPipeline(image, llrobot):
    # =========================================================================
    # INITIALIZE ALL VARIABLES FIRST (prevents UnboundLocalError)
    # =========================================================================
    largestContour = np.array([[]])
    llpython = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # llpython format:
    # [0] = valid tag found (1.0 = yes, 0.0 = no)
    # [1] = tag ID
    # [2] = distance to tag (in same units as TAG_SIZE_REAL)
    # [3] = tag center X (pixels)
    # [4] = tag center Y (pixels)
    # [5] = X offset from image center (pixels, positive = right)
    # [6] = Y offset from image center (pixels, positive = down)
    # [7] = tag pixel size (for debugging/calibration)

    # Get image dimensions
    image_height, image_width = image.shape[:2]

    # =========================================================================
    # CONVERT TO GRAYSCALE FOR DETECTION (required by apriltag library)
    # =========================================================================
    img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Optional: Apply slight blur to reduce noise (improves detection stability)
    img_gray = cv2.GaussianBlur(img_gray, (3, 3), 0)

    # =========================================================================
    # DETECT APRILTAGS
    # =========================================================================
    detections = detector.detect(img_gray)

    # =========================================================================
    # PROCESS DETECTIONS
    # =========================================================================
    best_detection = None
    best_tag_size = 0

    for detection in detections:
        corners = detection['lb-rb-rt-lt']
        tag_id = detection['id']

        # Calculate tag size in pixels
        tag_pixel_size = calculate_tag_pixel_size(corners)

        # Track the largest (closest) tag
        if tag_pixel_size > best_tag_size:
            best_tag_size = tag_pixel_size
            best_detection = detection

        # ----- DRAW ALL DETECTED TAGS -----
        # Draw tag outline
        corners_int = [(int(c[0]), int(c[1])) for c in corners]
        for i in range(4):
            cv2.line(image, corners_int[i], corners_int[(i + 1) % 4], (0, 255, 0), 2)

        # Calculate and display info for this tag
        distance = calculate_distance(tag_pixel_size)
        center_x, center_y, x_off, y_off = calculate_relative_position(
            corners, image_width, image_height
        )

        # Draw tag center
        cv2.circle(image, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

        # Draw text info
        info_text = f"ID:{tag_id} D:{distance:.1f}"
        cv2.putText(image, info_text,
                    (int(center_x) - 40, int(center_y) - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        pos_text = f"X:{x_off:.0f} Y:{y_off:.0f}"
        cv2.putText(image, pos_text,
                    (int(center_x) - 40, int(center_y) + 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 200