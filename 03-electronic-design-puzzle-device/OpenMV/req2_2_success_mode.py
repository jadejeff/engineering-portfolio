"""Requirement 2(2) successful detector/solver for the multimode launcher.

The vision and puzzle algorithms in this module are copied from
position_detector(3).py.  Only the module name, calibration storage and the
shared STM32 UART/run wrapper differ from the standalone successful program.
"""

import sensor
import time
import math
import gc


# ---------------- Configuration ----------------
LANDSCAPE = True
PIECES_ON_RIGHT = True
# Competition mode: do not use printed-card patterns as a matching signal.
CARD_PATTERN_MODE = False
# Printed marks may still punch small holes in the white mask. Repair only
# those local contour gaps; this does not identify or compare card patterns.
PATTERN_CONTOUR_REPAIR = True
# The supplied fragments have no inward edge notches. Treat every detected
# concavity as printing/glare damage and fit only the convex outer envelope.
ASSUME_CONVEX_PIECES = True

# The source area is a dark mat. Detect by LAB lightness only so colour cast
# and red/black printing cannot exclude the white card substrate.
WHITE_THRESH = [(50, 100, -128, 127, -128, 127)]
WHITE_THRESH_RELAXED = [(38, 100, -128, 127, -128, 127)]
RECT_THRESH_CALIB = 500
RECT_THRESH_RUN = 200
CALIB_STABLE_FRAMES = 5
CALIB_CORNER_MOVE_TOL_PX = 3.0
CALIBRATION_VERSION = "stable-v2"

MIN_PIECE_PX = 120
RELAXED_MIN_PX = 35
MIN_COMPONENT_PX = 12
MAX_PIECE_PX = 12000
MAX_BBOX_AREA = 25000
MIN_DENSITY = 0.04
MAX_ELONG = 10.0
MAX_CANDIDATES = 6
ROI_BORDER_GUARD_PX = 6
MAX_PIECE_WIDTH_RATIO = 0.96
MAX_PIECE_HEIGHT_RATIO = 0.90
MIN_REQUIRED_PIECES = 2
MAX_REQUIRED_PIECES = 4
MERGE = False
MARGIN = 3
CENTER_DIST_THRESH = 14

CONTOUR_ROI_PAD_PX = 4
CONTOUR_SAMPLE_MAX = 240
CARD_GAP_BRIDGE_PX = 3
CARD_PATTERN_SAMPLES = 12
CARD_PATTERN_DEPTHS_MM = (2.5, 5.0)
CARD_PATTERN_MIN_INFORMATIVE = 3
RDP_START_MM = 0.8
RDP_MAX_MM = 8.0
RDP_STEP_MM = 0.4
MAX_OUTPUT_VERTICES = 5
MIN_VERTEX_TURN_DEG = 12.0
COLLINEAR_DISTANCE_MM = 1.6
MIN_DETECTED_EDGE_MM = 12.0

SKIP_FRAMES = 1
STATUS_INTERVAL_FRAMES = 30
REPORT_INTERVAL_FRAMES = 12
STALE_A4_FRAMES = 30
LOCK_EXPOSURE_ON_STABLE_COUNT = False
ANTI_GLARE_EXPOSURE_MAX_US = 20000
ANTI_GLARE_SETTLE_FRAMES = 6
PIECE_COUNT_SETTLE_FRAMES = 4
SOLVE_STABLE_FRAMES = 4
SOLVE_MOVE_TOL_MM = 3.0
SOLVE_VERTEX_TOL_MM = 3.5
SOLVE_EDGE_ABS_TOL_MM = 3.0
SOLVE_EDGE_TOL_RATIO = 0.12
SOLVE_AREA_TOL_RATIO = 0.12
SOLVE_RETRY_FRAMES = 30

RED = (255, 0, 0)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)
CYAN = (0, 255, 255)
WHITE = (255, 255, 255)

if LANDSCAPE:
    A4_W = 297.0
    A4_H = 210.0
    TARGET_CENTER = (A4_W * 0.25, A4_H * 0.5)
else:
    A4_W = 210.0
    A4_H = 297.0
    TARGET_CENTER = (A4_W * 0.5, A4_H * 0.25)

TARGET_RECT_LONG_MM = 87.0
TARGET_RECT_SHORT_MM = 56.0
TARGET_RECT_RATIO = TARGET_RECT_LONG_MM / TARGET_RECT_SHORT_MM
CARD_TOTAL_AREA_MM2 = TARGET_RECT_LONG_MM * TARGET_RECT_SHORT_MM
COUNT_AREA_MIN_RATIO = 0.86
COUNT_AREA_MAX_RATIO = 1.08
COUNT_AREA_FRAME_TOL_RATIO = 0.04
TARGET_HARD_LONG_MIN = 79.0
TARGET_HARD_LONG_MAX = 95.0
TARGET_HARD_SHORT_MIN = 49.0
TARGET_HARD_SHORT_MAX = 63.0
TARGET_RATIO_MIN = 1.38
TARGET_RATIO_MAX = 1.72

SOLVER_CONFIG = {
    "target_center": TARGET_CENTER,
    "target_vertical": True,
    "edge_abs_tol": 4.0,
    "edge_rel_tol": 0.15,
    "min_fill": 0.86,
    "outer_tol": 4.5,
    "boundary_weight": 4.0,
    "side_weight": 3.0,
    "seam_weight": 10.0,
    "seam_tol": 3.0,
    "missing_outer_piece_penalty": 60.0,
    "min_boundary_coverage": 0.65,
    "min_outer_side_coverage": 0.20,
    "require_all_outer_corners": True,
    "corner_tol": 10.0,
    "minimum_physical_edge": MIN_DETECTED_EDGE_MM,
    "long_min": TARGET_HARD_LONG_MIN,
    "long_max": TARGET_HARD_LONG_MAX,
    "short_min": TARGET_HARD_SHORT_MIN,
    "short_max": TARGET_HARD_SHORT_MAX,
    "hard_long_min": TARGET_HARD_LONG_MIN,
    "hard_long_max": TARGET_HARD_LONG_MAX,
    "hard_short_min": TARGET_HARD_SHORT_MIN,
    "hard_short_max": TARGET_HARD_SHORT_MAX,
    "aspect_ratio_min": TARGET_RATIO_MIN,
    "aspect_ratio_max": TARGET_RATIO_MAX,
    "target_long": TARGET_RECT_LONG_MM,
    "target_short": TARGET_RECT_SHORT_MM,
    "target_dimension_weight": 0.75,
    "target_aspect_weight": 16.0,
    "convex_pieces_only": ASSUME_CONVEX_PIECES,
    "partial_bound_slack": 3.0,
    "low_memory": True,
    "low_memory_max_nodes": 4000,
    "low_memory_stop_score": 12.0,
    "low_memory_branch_width": 12,
    "state_dedup_limit": 5000,
    "state_angle_quant_deg": 0.25,
    "state_position_quant_mm": 0.25,
}

DETECTION_STATS = {}


def progress(step, message):
    print("[STEP %s] %s" % (step, message))


try:
    from geometry_solver_req2_2 import solve_puzzle
    SOLVER_AVAILABLE = True
    SOLVER_IMPORT_ERROR = None
except Exception as error:
    solve_puzzle = None
    SOLVER_AVAILABLE = False
    SOLVER_IMPORT_ERROR = error


# Camera initialization is intentionally performed only inside run().


# ---------------- A4 homography ----------------
def gauss_solve_8(matrix, vector):
    count = 8
    work = [matrix[row][:] + [vector[row]] for row in range(count)]
    for column in range(count):
        pivot = max(range(column, count),
                    key=lambda row: abs(work[row][column]))
        if abs(work[pivot][column]) < 1e-12:
            return None
        if pivot != column:
            work[column], work[pivot] = work[pivot], work[column]
        divisor = work[column][column]
        for index in range(column, count + 1):
            work[column][index] /= divisor
        for row in range(count):
            if row == column:
                continue
            factor = work[row][column]
            if abs(factor) > 0.0:
                for index in range(column, count + 1):
                    work[row][index] -= factor * work[column][index]
    return [work[index][count] for index in range(count)]


def solve_homography(source, destination):
    matrix = [[0.0] * 8 for _ in range(8)]
    vector = [0.0] * 8
    for index in range(4):
        x, y = source[index]
        target_x, target_y = destination[index]
        matrix[index * 2] = [
            x, y, 1, 0, 0, 0, -target_x * x, -target_x * y]
        matrix[index * 2 + 1] = [
            0, 0, 0, x, y, 1, -target_y * x, -target_y * y]
        vector[index * 2] = target_x
        vector[index * 2 + 1] = target_y
    values = gauss_solve_8(matrix, vector)
    return values + [1.0] if values is not None else None


def invert_homography(matrix):
    a, b, c, d, e, f, g, h, i = matrix
    ca = e * i - f * h
    cb = f * g - d * i
    cc = d * h - e * g
    determinant = a * ca + b * cb + c * cc
    if abs(determinant) < 1e-12:
        return None
    scale = 1.0 / determinant
    return [
        ca * scale, (c * h - b * i) * scale, (b * f - c * e) * scale,
        cb * scale, (a * i - c * g) * scale, (c * d - a * f) * scale,
        cc * scale, (b * g - a * h) * scale, (a * e - b * d) * scale,
    ]


def project(matrix, x, y):
    scale = matrix[6] * x + matrix[7] * y + matrix[8]
    if abs(scale) < 1e-9:
        scale = 1e-9
    return ((matrix[0] * x + matrix[1] * y + matrix[2]) / scale,
            (matrix[3] * x + matrix[4] * y + matrix[5]) / scale)


CALIBRATION_PATH = "/flash/calib_req2_2.txt"


def save_calibration(matrix):
    try:
        with open(CALIBRATION_PATH, "w") as output:
            output.write(CALIBRATION_VERSION + "\n")
            output.write("".join("%f\n" % value for value in matrix))
        progress("03", "calibration saved to " + CALIBRATION_PATH)
        return True
    except Exception as error:
        progress("99", "calibration save failed: %s" % error)
        return False


def load_calibration():
    try:
        with open(CALIBRATION_PATH, "r") as source:
            values = [value for value in source.read().strip().split("\n")
                      if value.strip()]
        if not values or values[0] != CALIBRATION_VERSION:
            progress("03", "old calibration ignored; 5-frame recalibration required")
            return None
        matrix = [float(value) for value in values[1:]]
        return matrix if len(matrix) == 9 else None
    except Exception:
        return None


def select_a4_rect(rectangles, frame):
    frame_area = frame.width() * frame.height()
    candidates = []
    for rectangle in rectangles:
        area = rectangle.w() * rectangle.h()
        longest = max(rectangle.w(), rectangle.h())
        shortest = min(rectangle.w(), rectangle.h())
        aspect = float(shortest) / longest if longest else 0.0
        if 0.03 * frame_area < area < 0.95 * frame_area and 0.3 < aspect < 1.0:
            candidates.append((area, rectangle))
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


def detect_a4_corners(frame, threshold):
    try:
        rectangles = frame.find_rects(threshold=threshold)
        rectangle = select_a4_rect(rectangles, frame) if rectangles else None
        if rectangle is None:
            return None
        corners = rectangle.corners()
        if isinstance(corners, (list, tuple)) and len(corners) == 4:
            return corners
    except Exception as error:
        progress("99", "A4 detection failed: %s" % error)
    return None


def classify_corners(corners):
    y_middle = (min(point[1] for point in corners) +
                max(point[1] for point in corners)) * 0.5
    image_top = [point for point in corners if point[1] <= y_middle]
    image_bottom = [point for point in corners if point[1] > y_middle]
    image_top.sort(key=lambda point: point[0])
    image_bottom.sort(key=lambda point: point[0])
    return {
        "BL": image_top[0], "BR": image_top[-1],
        "TL": image_bottom[0], "TR": image_bottom[-1],
    }


def half_roi(labels, frame):
    xs = [labels[name][0] for name in ("TL", "TR", "BL", "BR")]
    ys = [labels[name][1] for name in ("TL", "TR", "BL", "BR")]
    minimum_x = max(0, int(min(xs)))
    maximum_x = min(frame.width() - 1, int(max(xs)))
    minimum_y = max(0, int(min(ys)))
    maximum_y = min(frame.height() - 1, int(max(ys)))
    if LANDSCAPE:
        middle_x = max(minimum_x, min(maximum_x - 1,
                                      int(sum(xs) / 4)))
        if PIECES_ON_RIGHT:
            minimum_x = middle_x
        else:
            maximum_x = middle_x
    else:
        middle_y = max(minimum_y, min(maximum_y - 1,
                                      int(sum(ys) / 4)))
        if PIECES_ON_RIGHT:
            minimum_y = middle_y
        else:
            maximum_y = middle_y
    width = max(1, min(frame.width() - minimum_x,
                       maximum_x - minimum_x))
    height = max(1, min(frame.height() - minimum_y,
                        maximum_y - minimum_y))
    return (minimum_x, minimum_y, width, height)


def clip_roi(roi, frame):
    """Clip a cached-corner ROI so OpenMV never receives an invalid box."""
    x, y, width, height = roi
    image_width = frame.width()
    image_height = frame.height()
    x = max(0, min(image_width - 1, int(x)))
    y = max(0, min(image_height - 1, int(y)))
    right = max(x + 1, min(image_width, int(x + width)))
    bottom = max(y + 1, min(image_height, int(y + height)))
    return (x, y, right - x, bottom - y)


# ---------------- White-piece detection from 1success ----------------
def rect_distance(first, second):
    x1, y1, w1, h1 = first
    x2, y2, w2, h2 = second
    if x1 + w1 < x2:
        dx = x2 - x1 - w1
    elif x2 + w2 < x1:
        dx = x1 - x2 - w2
    else:
        dx = 0
    if y1 + h1 < y2:
        dy = y2 - y1 - h1
    elif y2 + h2 < y1:
        dy = y1 - y2 - h2
    else:
        dy = 0
    return math.sqrt(dx * dx + dy * dy)


def cluster_blobs(blobs):
    items = [(blob.cx(), blob.cy(), blob.rect(), blob.pixels(), blob)
             for blob in blobs]
    used = [False] * len(items)
    groups = []
    for first_index in range(len(items)):
        if used[first_index]:
            continue
        cx, cy, rectangle, area, blob = items[first_index]
        xs = [rectangle[0], rectangle[0] + rectangle[2]]
        ys = [rectangle[1], rectangle[1] + rectangle[3]]
        group = [blob]
        used[first_index] = True
        changed = True
        while changed:
            changed = False
            current = (min(xs), min(ys), max(xs) - min(xs), max(ys) - min(ys))
            for index in range(len(items)):
                if used[index]:
                    continue
                cx2, cy2, rectangle2, area2, blob2 = items[index]
                center_distance = math.sqrt((cx2 - cx) ** 2 + (cy2 - cy) ** 2)
                cluster_gap = (CARD_GAP_BRIDGE_PX
                               if PATTERN_CONTOUR_REPAIR else 1.0)
                if (rect_distance(current, rectangle2) <= cluster_gap and
                        center_distance < CENTER_DIST_THRESH):
                    used[index] = True
                    group.append(blob2)
                    xs.extend([rectangle2[0], rectangle2[0] + rectangle2[2]])
                    ys.extend([rectangle2[1], rectangle2[1] + rectangle2[3]])
                    total = area + area2
                    cx = (cx * area + cx2 * area2) / total
                    cy = (cy * area + cy2 * area2) / total
                    area = total
                    changed = True
        groups.append(((min(xs), min(ys), max(xs) - min(xs),
                        max(ys) - min(ys)), group))
    return groups


def filter_pieces(groups, roi, minimum_pixels):
    kept = []
    rejected = [0, 0, 0, 0, 0]
    for rectangle, blob_list in groups:
        x, y, width, height = rectangle
        roi_x, roi_y, roi_width, roi_height = roi
        touches_left = x - roi_x <= ROI_BORDER_GUARD_PX
        touches_right = (
            roi_x + roi_width - (x + width) <= ROI_BORDER_GUARD_PX)
        touches_top = y - roi_y <= ROI_BORDER_GUARD_PX
        touches_bottom = (
            roi_y + roi_height - (y + height) <= ROI_BORDER_GUARD_PX)
        if ((touches_left or touches_right) and
                (touches_top or touches_bottom)):
            rejected[0] += 1
            continue
        width, height = rectangle[2], rectangle[3]
        pixels = sum(blob.pixels() for blob in blob_list)
        box_area = width * height
        density = float(pixels) / box_area if box_area else 0.0
        if (pixels < minimum_pixels or pixels > MAX_PIECE_PX or
                box_area > MAX_BBOX_AREA):
            rejected[1] += 1
            continue
        if density < MIN_DENSITY or min(width, height) <= 0:
            rejected[2] += 1
            continue
        if float(max(width, height)) / min(width, height) > MAX_ELONG:
            rejected[3] += 1
            continue
        if (width > roi[2] * MAX_PIECE_WIDTH_RATIO or
                height > roi[3] * MAX_PIECE_HEIGHT_RATIO):
            rejected[4] += 1
            continue
        kept.append((rectangle, blob_list))
    return kept, tuple(rejected)


def detect_once(frame, roi, thresholds, minimum_pixels, label):
    global DETECTION_STATS
    roi = clip_roi(roi, frame)
    try:
        blobs = frame.find_blobs(
            thresholds, roi=roi, x_stride=1, y_stride=1,
            pixels_threshold=MIN_COMPONENT_PX,
            area_threshold=MIN_COMPONENT_PX,
            merge=MERGE,
            margin=max(MARGIN, CARD_GAP_BRIDGE_PX))
    except Exception as error:
        progress("99", "find_blobs failed: %s" % error)
        DETECTION_STATS[label] = (0, 0, (0, 0, 0, 0, 0))
        return []
    # Printed red/black areas may split one card fragment into several small
    # white blobs. Merge first, then apply the whole-piece area threshold.
    groups = cluster_blobs(blobs)
    kept, rejected = filter_pieces(groups, roi, minimum_pixels)
    DETECTION_STATS[label] = (len(blobs), len(groups), rejected)
    return kept


def piece_pixels(piece):
    return sum(blob.pixels() for blob in piece[1])


def detect_pieces(frame, roi, force_relaxed=False):
    primary = detect_once(
        frame, roi, WHITE_THRESH, MIN_PIECE_PX, "primary")
    if not force_relaxed and len(primary) >= MAX_REQUIRED_PIECES:
        primary.sort(key=piece_pixels, reverse=True)
        return primary[:MAX_CANDIDATES], "primary", len(primary), -1
    relaxed = detect_once(
        frame, roi, WHITE_THRESH_RELAXED, RELAXED_MIN_PX, "relaxed")
    if force_relaxed:
        pieces = relaxed
    else:
        pieces = relaxed if len(relaxed) > len(primary) else primary
    pieces.sort(key=piece_pixels, reverse=True)
    mode = ("relaxed-forced" if force_relaxed else
            ("relaxed" if pieces is relaxed else "primary"))
    return pieces[:MAX_CANDIDATES], mode, len(primary), len(relaxed)


def report_detection_stats():
    for label in ("primary", "relaxed"):
        stats = DETECTION_STATS.get(label)
        if stats is None:
            continue
        raw_count, group_count, rejected = stats
        progress(
            "05",
            "%s raw=%d groups=%d reject corner=%d size=%d density=%d shape=%d bounds=%d" %
            (label, raw_count, group_count, rejected[0], rejected[1],
             rejected[2], rejected[3], rejected[4]))


def foreground_centroid(blob_list, rectangle):
    pixels = sum(blob.pixels() for blob in blob_list)
    if pixels:
        return (sum(blob.cx() * blob.pixels() for blob in blob_list) / pixels,
                sum(blob.cy() * blob.pixels() for blob in blob_list) / pixels)
    x, y, width, height = rectangle
    return x + width * 0.5, y + height * 0.5


# ---------------- Firmware-compatible boundary and vertices ----------------
def pixel_brightness(value):
    if isinstance(value, (tuple, list)):
        if len(value) >= 3:
            return int((value[0] + value[1] + value[2]) // 3)
        return int(value[0]) if value else 0
    if value is None:
        return 0
    value = int(value)
    if value <= 255:
        return value
    red = ((value >> 11) & 0x1f) * 255 // 31
    green = ((value >> 5) & 0x3f) * 255 // 63
    blue = (value & 0x1f) * 255 // 31
    return (red + green + blue) // 3


def otsu_threshold(histogram, total):
    weighted_total = 0
    for index in range(256):
        weighted_total += index * histogram[index]
    background_weight = 0
    background_sum = 0
    best_threshold = 127
    best_variance = -1.0
    for threshold in range(256):
        count = histogram[threshold]
        background_weight += count
        background_sum += threshold * count
        if background_weight <= 0:
            continue
        foreground_weight = total - background_weight
        if foreground_weight <= 0:
            break
        background_mean = background_sum / background_weight
        foreground_mean = (weighted_total - background_sum) / foreground_weight
        difference = background_mean - foreground_mean
        variance = background_weight * foreground_weight * difference * difference
        if variance > best_variance:
            best_variance = variance
            best_threshold = threshold
    return best_threshold


def trace_boundary(mask, seed_x, seed_y):
    height = len(mask)
    width = len(mask[0]) if height else 0
    if width <= 0:
        return []
    seed_x = max(0, min(width - 1, int(seed_x)))
    seed_y = max(0, min(height - 1, int(seed_y)))
    nearest = (seed_x, seed_y) if mask[seed_y][seed_x] else None
    if nearest is None:
        nearest_distance = 1e30
        for y in range(height):
            row = mask[y]
            for x in range(width):
                if row[x]:
                    distance = (x - seed_x) ** 2 + (y - seed_y) ** 2
                    if distance < nearest_distance:
                        nearest = (x, y)
                        nearest_distance = distance
    if nearest is None:
        return []
    x, y = nearest
    while x > 0 and mask[y][x - 1]:
        x -= 1
    directions = [(1, 0), (1, 1), (0, 1), (-1, 1),
                  (-1, 0), (-1, -1), (0, -1), (1, -1)]
    start = (x, y)
    current = start
    backtrack = (x - 1, y)
    first_next = None
    points = [start]
    for _ in range(width * height * 4):
        back_direction = (backtrack[0] - current[0],
                          backtrack[1] - current[1])
        try:
            back_index = directions.index(back_direction)
        except ValueError:
            back_index = 4
        next_point = None
        for step in range(1, 9):
            direction_index = (back_index + step) % 8
            dx, dy = directions[direction_index]
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < width and 0 <= ny < height and mask[ny][nx]:
                previous_index = (direction_index - 1) % 8
                pdx, pdy = directions[previous_index]
                backtrack = (current[0] + pdx, current[1] + pdy)
                next_point = (nx, ny)
                break
        if next_point is None:
            return points
        if first_next is None:
            first_next = next_point
        elif current == start and next_point == first_next:
            if points and points[-1] == start:
                points.pop()
            break
        points.append(next_point)
        current = next_point
    return points


def outer_mask_seed(mask, preferred_x, preferred_y, candidate_bounds):
    """Choose the card component at the candidate box edge, not its print."""
    height = len(mask)
    width = len(mask[0]) if height else 0
    if width <= 0:
        return None
    preferred_x = max(0, min(width - 1, int(preferred_x)))
    preferred_y = max(0, min(height - 1, int(preferred_y)))
    left, top, right, bottom = candidate_bounds
    left = max(0, min(width - 1, int(left)))
    top = max(0, min(height - 1, int(top)))
    right = max(left, min(width - 1, int(right)))
    bottom = max(top, min(height - 1, int(bottom)))
    best = None
    best_edge_distance = width + height
    best_neighbours = -1
    best_center_distance = 1 << 30
    # Requiring two neighbours avoids selecting an isolated glare pixel at a
    # bounding-box extremum. The second pass keeps very sharp valid corners.
    for minimum_neighbours in (2, 0):
        for y in range(top, bottom + 1):
            row = mask[y]
            for x in range(left, right + 1):
                if not row[x]:
                    continue
                neighbours = 0
                for offset_y in (-1, 0, 1):
                    neighbour_y = y + offset_y
                    if neighbour_y < 0 or neighbour_y >= height:
                        continue
                    neighbour_row = mask[neighbour_y]
                    for offset_x in (-1, 0, 1):
                        if offset_x == 0 and offset_y == 0:
                            continue
                        neighbour_x = x + offset_x
                        if (0 <= neighbour_x < width and
                                neighbour_row[neighbour_x]):
                            neighbours += 1
                if neighbours < minimum_neighbours:
                    continue
                edge_distance = min(x - left, right - x,
                                    y - top, bottom - y)
                center_distance = ((x - preferred_x) ** 2 +
                                   (y - preferred_y) ** 2)
                better = (
                    edge_distance < best_edge_distance or
                    (edge_distance == best_edge_distance and
                     neighbours > best_neighbours) or
                    (edge_distance == best_edge_distance and
                     neighbours == best_neighbours and
                     center_distance < best_center_distance))
                if better:
                    best = (x, y)
                    best_edge_distance = edge_distance
                    best_neighbours = neighbours
                    best_center_distance = center_distance
        if best is not None:
            return best
    return None


def adaptive_boundary(frame, rectangle, seed):
    x, y, width, height = rectangle
    x0 = max(0, x - CONTOUR_ROI_PAD_PX)
    y0 = max(0, y - CONTOUR_ROI_PAD_PX)
    x1 = min(frame.width(), x + width + CONTOUR_ROI_PAD_PX)
    y1 = min(frame.height(), y + height + CONTOUR_ROI_PAD_PX)
    width, height = max(1, x1 - x0), max(1, y1 - y0)
    brightness = [bytearray(width) for _ in range(height)]
    histogram = [0] * 256
    minimum, maximum = 255, 0
    for local_y in range(height):
        row = brightness[local_y]
        for local_x in range(width):
            value = pixel_brightness(frame.get_pixel(x0 + local_x,
                                                      y0 + local_y))
            row[local_x] = value
            histogram[value] += 1
            if value < minimum:
                minimum = value
            if value > maximum:
                maximum = value
    threshold = otsu_threshold(histogram, width * height)
    mask = [bytearray(width) for _ in range(height)]
    foreground_count = 0
    for local_y in range(height):
        source_row = brightness[local_y]
        mask_row = mask[local_y]
        for local_x in range(width):
            if source_row[local_x] > threshold:
                mask_row[local_x] = 1
                foreground_count += 1
    if PATTERN_CONTOUR_REPAIR:
        bridge_short_mask_gaps(mask, CARD_GAP_BRIDGE_PX)
    del brightness
    del histogram
    local_seed_x = int(round(seed[0] - x0))
    local_seed_y = int(round(seed[1] - y0))
    outer_seed = outer_mask_seed(
        mask, local_seed_x, local_seed_y,
        (x - x0, y - y0, x + rectangle[2] - 1 - x0,
         y + rectangle[3] - 1 - y0))
    if outer_seed is None:
        outer_seed = (local_seed_x, local_seed_y)
    seed_shift = math.sqrt((outer_seed[0] - local_seed_x) ** 2 +
                           (outer_seed[1] - local_seed_y) ** 2)
    points = trace_boundary(mask, outer_seed[0], outer_seed[1])
    del mask
    return ([(point[0] + x0, point[1] + y0) for point in points],
            threshold, minimum, maximum, foreground_count, seed_shift)


def bridge_short_mask_gaps(mask, maximum_gap):
    """Bridge tiny ink gaps in-place without allocating a second mask."""
    height = len(mask)
    width = len(mask[0]) if height else 0
    maximum_gap = max(1, int(maximum_gap))
    for y in range(height):
        row = mask[y]
        x = 0
        while x < width:
            if row[x]:
                x += 1
                continue
            start = x
            while x < width and not row[x]:
                x += 1
            if start > 0 and x < width and x - start <= maximum_gap:
                for fill_x in range(start, x):
                    row[fill_x] = 1
    for x in range(width):
        y = 0
        while y < height:
            if mask[y][x]:
                y += 1
                continue
            start = y
            while y < height and not mask[y][x]:
                y += 1
            if start > 0 and y < height and y - start <= maximum_gap:
                for fill_y in range(start, y):
                    mask[fill_y][x] = 1


def point_line_distance(point, start, end):
    vx = end[0] - start[0]
    vy = end[1] - start[1]
    denominator = vx * vx + vy * vy
    if denominator <= 1e-12:
        dx, dy = point[0] - start[0], point[1] - start[1]
        return math.sqrt(dx * dx + dy * dy)
    numerator = abs(vy * point[0] - vx * point[1] +
                    end[0] * start[1] - end[1] * start[0])
    return numerator / math.sqrt(denominator)


def rdp_open(points, epsilon):
    if len(points) <= 2:
        return points[:]
    maximum, split = -1.0, 0
    for index in range(1, len(points) - 1):
        distance = point_line_distance(points[index], points[0], points[-1])
        if distance > maximum:
            maximum, split = distance, index
    if maximum > epsilon:
        left = rdp_open(points[:split + 1], epsilon)
        right = rdp_open(points[split:], epsilon)
        return left[:-1] + right
    return [points[0], points[-1]]


def vertex_turn_degrees(previous, current, following):
    first_x = current[0] - previous[0]
    first_y = current[1] - previous[1]
    second_x = following[0] - current[0]
    second_y = following[1] - current[1]
    first_length = math.sqrt(first_x * first_x + first_y * first_y)
    second_length = math.sqrt(second_x * second_x + second_y * second_y)
    if first_length <= 1e-9 or second_length <= 1e-9:
        return 0.0
    cosine = ((first_x * second_x + first_y * second_y) /
              (first_length * second_length))
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def simplify_closed(points, epsilon):
    if len(points) < 3:
        return []
    first = points[0]
    farthest, maximum = 1, 0.0
    for index in range(1, len(points)):
        dx = points[index][0] - first[0]
        dy = points[index][1] - first[1]
        distance = dx * dx + dy * dy
        if distance > maximum:
            maximum, farthest = distance, index
    first_chain = rdp_open(points[:farthest + 1], epsilon)
    second_chain = rdp_open(points[farthest:] + [points[0]], epsilon)
    polygon = first_chain[:-1] + second_chain[:-1]
    while len(polygon) > 3:
        weakest_index = None
        weakest_rank = None
        count = len(polygon)
        for index in range(count):
            previous = polygon[(index - 1) % count]
            current = polygon[index]
            following = polygon[(index + 1) % count]
            edge1 = math.sqrt((current[0] - previous[0]) ** 2 +
                              (current[1] - previous[1]) ** 2)
            edge2 = math.sqrt((following[0] - current[0]) ** 2 +
                              (following[1] - current[1]) ** 2)
            line_distance = point_line_distance(
                current, previous, following)
            turn_degrees = vertex_turn_degrees(
                previous, current, following)
            removable = (
                min(edge1, edge2) < MIN_DETECTED_EDGE_MM or
                line_distance < max(COLLINEAR_DISTANCE_MM,
                                    epsilon * 0.55) or
                turn_degrees < MIN_VERTEX_TURN_DEG)
            if removable:
                rank = (turn_degrees, line_distance, min(edge1, edge2))
                if weakest_rank is None or rank < weakest_rank:
                    weakest_rank = rank
                    weakest_index = index
        if weakest_index is None:
            break
        del polygon[weakest_index]
    return polygon


def convex_hull(points):
    """Return a low-memory convex envelope and remove inward print notches."""
    if len(points) <= 2:
        return points[:]
    ordered = sorted(points)
    unique = []
    for point in ordered:
        if not unique or point != unique[-1]:
            unique.append(point)
    if len(unique) <= 2:
        return unique

    def turn(first, second, third):
        return ((second[0] - first[0]) * (third[1] - first[1]) -
                (second[1] - first[1]) * (third[0] - first[0]))

    lower = []
    for point in unique:
        while len(lower) >= 2 and turn(
                lower[-2], lower[-1], point) <= 1e-9:
            lower.pop()
        lower.append(point)
    upper = []
    for index in range(len(unique) - 1, -1, -1):
        point = unique[index]
        while len(upper) >= 2 and turn(
                upper[-2], upper[-1], point) <= 1e-9:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def polygon_vertices(contour_pixels, pixel_to_mm):
    if len(contour_pixels) < 6:
        return [], 0.0
    step = max(1, len(contour_pixels) // CONTOUR_SAMPLE_MAX)
    points_mm = [project(pixel_to_mm, contour_pixels[index][0],
                         contour_pixels[index][1])
                 for index in range(0, len(contour_pixels), step)]
    if ASSUME_CONVEX_PIECES:
        points_mm = convex_hull(points_mm)
        if len(points_mm) < 3:
            return [], 0.0
    epsilon = RDP_START_MM
    best = []
    best_epsilon = epsilon
    while epsilon <= RDP_MAX_MM + 1e-6:
        polygon = simplify_closed(points_mm, epsilon)
        if len(polygon) >= 3:
            best, best_epsilon = polygon, epsilon
        if 3 <= len(polygon) <= MAX_OUTPUT_VERTICES:
            return polygon, epsilon
        epsilon += RDP_STEP_MM
    return best, best_epsilon


def polygon_area(points):
    area = 0.0
    for index in range(len(points)):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % len(points)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


def observations_total_area(observations):
    return sum(polygon_area(item["vertices_mm"])
               for item in observations)


def select_area_confirmed_observations(observations):
    """Choose 2-4 candidates whose total area matches one complete card."""
    candidate_count = len(observations)
    raw_total = observations_total_area(observations)
    raw_ratio = raw_total / CARD_TOTAL_AREA_MM2

    if candidate_count < MIN_REQUIRED_PIECES:
        return None, raw_total, raw_ratio, raw_total, "COUNT_LOW"

    if (candidate_count <= MAX_REQUIRED_PIECES and
            COUNT_AREA_MIN_RATIO <= raw_ratio <= COUNT_AREA_MAX_RATIO):
        return (list(observations), raw_total, raw_ratio,
                raw_total, "ALL")

    best = None
    limit = 1 << candidate_count

    for mask in range(1, limit):
        subset_count = 0
        total_area = 0.0

        for index in range(candidate_count):
            if mask & (1 << index):
                subset_count += 1
                total_area += polygon_area(
                    observations[index]["vertices_mm"])

        if not (MIN_REQUIRED_PIECES <= subset_count <=
                MAX_REQUIRED_PIECES):
            continue

        ratio = total_area / CARD_TOTAL_AREA_MM2
        if not (COUNT_AREA_MIN_RATIO <= ratio <= COUNT_AREA_MAX_RATIO):
            continue

        score = (abs(1.0 - ratio), -subset_count, mask)
        if best is None or score < best[0]:
            selected = [observations[index]
                        for index in range(candidate_count)
                        if mask & (1 << index)]
            best = (score, selected, total_area, ratio)

    if best is not None:
        return best[1], best[2], best[3], raw_total, "SUBSET"

    if raw_ratio < COUNT_AREA_MIN_RATIO:
        reason = "AREA_LOW"
    elif raw_ratio > COUNT_AREA_MAX_RATIO:
        reason = "AREA_HIGH"
    else:
        reason = "NO_SUBSET"

    return None, raw_total, raw_ratio, raw_total, reason


def polygon_edge_lengths(points):
    lengths = []
    for index in range(len(points)):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % len(points)]
        dx, dy = x2 - x1, y2 - y1
        lengths.append(math.sqrt(dx * dx + dy * dy))
    return lengths


def polygon_signed_area(points):
    area = 0.0
    for index in range(len(points)):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % len(points)]
        area += x1 * y2 - x2 * y1
    return area * 0.5


def polygon_centroid(points):
    """Return the uniform-area centroid, ignoring printed interior holes."""
    if len(points) < 3:
        return None
    twice_area = 0.0
    center_x = 0.0
    center_y = 0.0
    for index in range(len(points)):
        x0, y0 = points[index]
        x1, y1 = points[(index + 1) % len(points)]
        cross = x0 * y1 - x1 * y0
        twice_area += cross
        center_x += (x0 + x1) * cross
        center_y += (y0 + y1) * cross
    if abs(twice_area) < 1e-9:
        return (sum(point[0] for point in points) / len(points),
                sum(point[1] for point in points) / len(points))
    return (center_x / (3.0 * twice_area),
            center_y / (3.0 * twice_area))


def pixel_rgb(value):
    if isinstance(value, (tuple, list)):
        if len(value) >= 3:
            return int(value[0]), int(value[1]), int(value[2])
        gray = int(value[0]) if value else 0
        return gray, gray, gray
    if value is None:
        return 0, 0, 0
    value = int(value)
    if value <= 255:
        return value, value, value
    return (((value >> 11) & 0x1f) * 255 // 31,
            ((value >> 5) & 0x3f) * 255 // 63,
            (value & 0x1f) * 255 // 31)


def classify_pattern_pixel(value):
    red, green, blue = pixel_rgb(value)
    brightness = (red + green + blue) // 3
    redness = red - (green + blue) // 2
    if brightness < 85:
        return 2
    if redness > 35 and red > green + 15:
        return 1
    if brightness > 165 and max(red, green, blue) - min(red, green, blue) < 70:
        return 0
    return 3


def sample_edge_pattern(frame, polygon, edge_index, mm_to_pixel):
    """Return compact white/red/dark labels for one inward edge strip."""
    count = int(CARD_PATTERN_SAMPLES)
    labels = bytearray(count)
    if len(polygon) < 3:
        return labels, 0
    a = polygon[edge_index]
    b = polygon[(edge_index + 1) % len(polygon)]
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.sqrt(dx * dx + dy * dy)
    if length < MIN_DETECTED_EDGE_MM:
        return labels, 0
    sign = 1.0 if polygon_signed_area(polygon) >= 0.0 else -1.0
    nx, ny = -dy / length * sign, dx / length * sign
    informative = 0
    for index in range(count):
        fraction = 0.08 + 0.84 * (index + 0.5) / float(count)
        px = a[0] + dx * fraction
        py = a[1] + dy * fraction
        red_count, dark_count, white_count = 0, 0, 0
        for depth in CARD_PATTERN_DEPTHS_MM:
            sample_x, sample_y = project(
                mm_to_pixel, px + nx * depth, py + ny * depth)
            ix, iy = int(round(sample_x)), int(round(sample_y))
            if (ix < 0 or iy < 0 or ix >= frame.width() or
                    iy >= frame.height()):
                continue
            label = classify_pattern_pixel(frame.get_pixel(ix, iy))
            if label == 1:
                red_count += 1
            elif label == 2:
                dark_count += 1
            elif label == 0:
                white_count += 1
        if red_count:
            labels[index] = 1
            informative += 1
        elif dark_count:
            labels[index] = 2
            informative += 1
        elif white_count:
            labels[index] = 0
        else:
            labels[index] = 3
    return labels, informative


def attach_edge_patterns(frame, observations, mm_to_pixel):
    total_informative = 0
    for item in observations:
        edge_patterns = []
        piece_informative = 0
        polygon = item["vertices_mm"]
        for edge_index in range(len(polygon)):
            labels, informative = sample_edge_pattern(
                frame, polygon, edge_index, mm_to_pixel)
            edge_patterns.append(labels)
            piece_informative += informative
        item["edge_pattern"] = edge_patterns
        total_informative += piece_informative
        progress("08", "piece[%d] card pattern samples=%d" %
                 (item["id"], piece_informative))
    progress("08", "card pattern sampling complete informative=%d" %
             total_informative)


def align_polygon_vertices(reference, current):
    """Match cyclic vertex order, allowing a reversed contour direction."""
    count = len(reference)
    if count != len(current) or count == 0:
        return None
    best = None
    best_score = None
    for reverse in (False, True):
        sequence = current[::-1] if reverse else current
        for shift in range(count):
            aligned = [sequence[(index + shift) % count]
                       for index in range(count)]
            score = 0.0
            for old, new in zip(reference, aligned):
                dx, dy = old[0] - new[0], old[1] - new[1]
                score += dx * dx + dy * dy
            if best_score is None or score < best_score:
                best_score = score
                best = aligned
    return best


def copy_observations(observations):
    copied = []
    for item in observations:
        clone = dict(item)
        clone["center_mm"] = tuple(item["center_mm"])
        clone["vertices_mm"] = [tuple(point)
                                for point in item["vertices_mm"]]
        clone["vertices_px"] = [tuple(point)
                                for point in item.get("vertices_px", ())]
        copied.append(clone)
    return copied


def average_observations(previous, current, sample_count, mm_to_pixel):
    """Update the stable geometry with one aligned-frame running average."""
    alpha = 1.0 / max(2, int(sample_count))
    averaged = []
    for old, new in zip(previous, current):
        aligned = align_polygon_vertices(
            old["vertices_mm"], new["vertices_mm"])
        if aligned is None:
            return copy_observations(current)
        item = dict(new)
        item["center_mm"] = (
            old["center_mm"][0] * (1.0 - alpha) +
            new["center_mm"][0] * alpha,
            old["center_mm"][1] * (1.0 - alpha) +
            new["center_mm"][1] * alpha,
        )
        item["vertices_mm"] = [
            (old_point[0] * (1.0 - alpha) + new_point[0] * alpha,
             old_point[1] * (1.0 - alpha) + new_point[1] * alpha)
            for old_point, new_point in zip(old["vertices_mm"], aligned)
        ]
        cx, cy = project(mm_to_pixel, item["center_mm"][0],
                         item["center_mm"][1])
        item["center_px"] = (cx, cy)
        item["vertices_px"] = []
        for vertex in item["vertices_mm"]:
            px, py = project(mm_to_pixel, vertex[0], vertex[1])
            item["vertices_px"].append(
                (int(round(px)), int(round(py))))
        averaged.append(item)
    return averaged


def draw_and_collect(frame, pieces, pixel_to_mm, mm_to_pixel):
    observations = []
    for rectangle, blob_list in pieces:
        seed_x, seed_y = foreground_centroid(blob_list, rectangle)
        (contour, threshold, minimum, maximum, foreground,
         seed_shift) = adaptive_boundary(
             frame, rectangle, (seed_x, seed_y))
        polygon_mm, epsilon = polygon_vertices(contour, pixel_to_mm)
        center_mm = polygon_centroid(polygon_mm)
        if center_mm is None:
            center_mm = project(pixel_to_mm, seed_x, seed_y)
        cx, cy = project(mm_to_pixel, center_mm[0], center_mm[1])
        observations.append({
            "rect": rectangle,
            "center_px": (cx, cy),
            "center_mm": center_mm,
            "vertices_mm": polygon_mm,
            "epsilon": epsilon,
            "contour_points": len(contour),
            "threshold": threshold,
            "range": (minimum, maximum),
            "foreground": foreground,
            "seed_shift": seed_shift,
            "pixels": piece_pixels((rectangle, blob_list)),
        })
    observations.sort(key=lambda item: (item["center_mm"][0],
                                         item["center_mm"][1]))
    for piece_id, item in enumerate(observations):
        item["id"] = piece_id
        frame.draw_rectangle(item["rect"], color=RED, thickness=1)
        polygon_px = []
        for vertex in item["vertices_mm"]:
            px, py = project(mm_to_pixel, vertex[0], vertex[1])
            pixel = (int(round(px)), int(round(py)))
            polygon_px.append(pixel)
            frame.draw_cross(pixel[0], pixel[1], color=YELLOW, size=5)
        item["vertices_px"] = polygon_px
        if len(polygon_px) >= 3:
            for index in range(len(polygon_px)):
                x0, y0 = polygon_px[index]
                x1, y1 = polygon_px[(index + 1) % len(polygon_px)]
                frame.draw_line(x0, y0, x1, y1, color=RED, thickness=2)
        cx, cy = item["center_px"]
        frame.draw_cross(int(round(cx)), int(round(cy)), color=YELLOW, size=8)
        frame.draw_string(int(round(cx)) + 5, int(round(cy)) - 10,
                          "P%d" % piece_id, color=YELLOW)
    return observations


def report_observations(observations, mode, primary_count, relaxed_count):
    progress("05", "threshold=%s primary=%d relaxed=%d selected=%d" %
             (mode, primary_count, relaxed_count, len(observations)))
    progress("06", "coordinate report begin")
    for item in observations:
        x, y, width, height = item["rect"]
        cx, cy = item["center_px"]
        mx, my = item["center_mm"]
        progress("06", "piece[%d] bbox_px=(%d,%d,%d,%d) pixels=%d" %
                 (item["id"], x, y, width, height, item["pixels"]))
        progress("06", "piece[%d] centroid px=(%.1f,%.1f) mm=(%.1f,%.1f)" %
                 (item["id"], cx, cy, mx, my))
        progress("07", "piece[%d] boundary=%d threshold=%d range=%d..%d foreground=%d outer-seed=%.1fpx vertices=%d epsilon=%.1fmm" %
                 (item["id"], item["contour_points"], item["threshold"],
                   item["range"][0], item["range"][1], item["foreground"],
                   item["seed_shift"], len(item["vertices_mm"]),
                   item["epsilon"]))
        for vertex_id, vertex in enumerate(item["vertices_mm"]):
            vertex_px = item["vertices_px"][vertex_id]
            progress("07", "piece[%d] vertex[%d] px=(%d,%d) mm=(%.1f,%.1f)" %
                     (item["id"], vertex_id, vertex_px[0], vertex_px[1],
                      vertex[0], vertex[1]))
    progress("08", "red=piece border; yellow=centroid and vertices")


def solver_input_issue(observations):
    if not MIN_REQUIRED_PIECES <= len(observations) <= MAX_REQUIRED_PIECES:
        return "need 2-4 pieces"
    for item in observations:
        vertices = item["vertices_mm"]
        if not 3 <= len(vertices) <= 5:
            return "piece[%d] has %d vertices" % (item["id"], len(vertices))
        minimum_boundary = int(1.5 * max(item["rect"][2], item["rect"][3]))
        if item["contour_points"] < minimum_boundary:
            return "piece[%d] contour %d<%d (internal region)" % (
                item["id"], item["contour_points"], minimum_boundary)
        lengths = polygon_edge_lengths(vertices)
        if lengths and min(lengths) < MIN_DETECTED_EDGE_MM:
            return "piece[%d] edge %.1fmm<%.1fmm" % (
                item["id"], min(lengths), MIN_DETECTED_EDGE_MM)
    return None


def solver_input_valid(observations):
    return solver_input_issue(observations) is None


def observations_compatible(previous, current):
    if previous is None or len(previous) != len(current):
        return False
    for old, new in zip(previous, current):
        if len(old["vertices_mm"]) != len(new["vertices_mm"]):
            return False
        aligned = align_polygon_vertices(
            old["vertices_mm"], new["vertices_mm"])
        if aligned is None:
            return False
        dx = old["center_mm"][0] - new["center_mm"][0]
        dy = old["center_mm"][1] - new["center_mm"][1]
        if math.sqrt(dx * dx + dy * dy) > SOLVE_MOVE_TOL_MM:
            return False
        for old_point, new_point in zip(old["vertices_mm"], aligned):
            vx = old_point[0] - new_point[0]
            vy = old_point[1] - new_point[1]
            if math.sqrt(vx * vx + vy * vy) > SOLVE_VERTEX_TOL_MM:
                return False
        old_lengths = polygon_edge_lengths(old["vertices_mm"])
        new_lengths = polygon_edge_lengths(aligned)
        for old_length, new_length in zip(old_lengths, new_lengths):
            tolerance = max(
                SOLVE_EDGE_ABS_TOL_MM,
                SOLVE_EDGE_TOL_RATIO * max(old_length, new_length))
            if abs(old_length - new_length) > tolerance:
                return False
        old_area = polygon_area(old["vertices_mm"])
        new_area = polygon_area(aligned)
        if max(old_area, new_area) <= 1e-6:
            return False
        if abs(old_area - new_area) / max(old_area, new_area) > SOLVE_AREA_TOL_RATIO:
            return False
    return True


def report_stable_coordinates(observations, sample_count):
    progress("08", "stable coordinate average from %d frames" % sample_count)
    for item in observations:
        cx, cy = item["center_mm"]
        progress("08", "stable piece[%d] centroid mm=(%.1f,%.1f)" %
                 (item["id"], cx, cy))
        for vertex_id, vertex in enumerate(item["vertices_mm"]):
            progress("08", "stable piece[%d] vertex[%d] mm=(%.1f,%.1f)" %
                     (item["id"], vertex_id, vertex[0], vertex[1]))
        lengths = polygon_edge_lengths(item["vertices_mm"])
        progress("08", "stable piece[%d] edges_mm=%s" %
                 (item["id"], ",".join("%.1f" % value
                                        for value in lengths)))


def draw_solution(frame, solution, mm_to_pixel):
    all_target_points = []
    for placement in solution["placements"]:
        all_target_points.extend(placement["polygon"])

    if all_target_points:
        minimum_x = min(point[0] for point in all_target_points)
        maximum_x = max(point[0] for point in all_target_points)
        minimum_y = min(point[1] for point in all_target_points)
        maximum_y = max(point[1] for point in all_target_points)
        rectangle_mm = [
            (minimum_x, minimum_y), (maximum_x, minimum_y),
            (maximum_x, maximum_y), (minimum_x, maximum_y),
        ]
        rectangle_px = []
        for point in rectangle_mm:
            px, py = project(mm_to_pixel, point[0], point[1])
            rectangle_px.append((int(round(px)), int(round(py))))
        for index in range(4):
            x0, y0 = rectangle_px[index]
            x1, y1 = rectangle_px[(index + 1) % 4]
            frame.draw_line(x0, y0, x1, y1, color=CYAN, thickness=1)

    for placement in solution["placements"]:
        polygon_px = []
        for point in placement["polygon"]:
            px, py = project(mm_to_pixel, point[0], point[1])
            pixel = (int(round(px)), int(round(py)))
            polygon_px.append(pixel)
            frame.draw_cross(pixel[0], pixel[1], color=CYAN, size=3)
        if len(polygon_px) >= 3:
            for index in range(len(polygon_px)):
                x0, y0 = polygon_px[index]
                x1, y1 = polygon_px[(index + 1) % len(polygon_px)]
                frame.draw_line(x0, y0, x1, y1, color=BLUE, thickness=3)
        tx, ty = project(mm_to_pixel, placement["target"][0],
                         placement["target"][1])
        frame.draw_cross(int(round(tx)), int(round(ty)), color=CYAN, size=7)
        frame.draw_string(int(round(tx)) + 4, int(round(ty)) - 10,
                          "S%d %.0f" % (placement["slot"],
                                         placement["angle"]), color=BLUE)


def run_solver(observations):
    solver_input = []
    for item in observations:
        solver_input.append({
            "id": item["id"],
            "polygon": item["vertices_mm"],
            "pick": item["center_mm"],
        })
    attempts = [
        ("F", 16, 1200, 12.0, 0.86, 4.5, True, False, 0.34, 4000,
         83.0, 91.0, 52.0, 60.0),
        ("A", 12, 600, 12.0, 0.86, 4.5, True, False, 0.34, 4000,
         83.0, 91.0, 52.0, 60.0),
        ("B", 24, 2500, 18.0, 0.78, 7.0, True, True, 0.30, 8000,
         81.0, 93.0, 50.0, 62.0),
        ("C", 64, 9000, None, 0.65, 10.0, True, True, 0.12, 14000,
         79.0, 95.0, 49.0, 63.0),
        ("D", 96, 20000, None, 0.60, 12.0, True, True, 0.12, 15000,
         79.0, 95.0, 49.0, 63.0),
    ]
    best_solution = None
    best_rank = None
    total_start_ms = time.ticks_ms()

    total_limit_ms = 45000
    progress("09", "rule pruning: target=87x56mm hard=79..95x49..63 ratio=1.38..1.72, each piece outer")
    for (name, branch, nodes, stop_score, minimum_fill, outer_tol,
         require_each_outer, allow_partial, partial_ratio, pass_limit_ms,
         long_min, long_max,
         short_min, short_max) in attempts:
        total_elapsed_ms = time.ticks_diff(time.ticks_ms(), total_start_ms)
        remaining_ms = total_limit_ms - total_elapsed_ms
        if remaining_ms <= 0:
            progress("10", "45-second search budget reached")
            break
        config = dict(SOLVER_CONFIG)
        config["low_memory_branch_width"] = branch
        config["low_memory_max_nodes"] = nodes
        config["low_memory_stop_score"] = stop_score
        config["min_fill"] = minimum_fill
        config["outer_tol"] = outer_tol
        config["require_each_outer"] = require_each_outer
        config["allow_partial_edges"] = allow_partial
        config["partial_edge_min_ratio"] = partial_ratio
        config["long_min"] = long_min
        config["long_max"] = long_max
        config["short_min"] = short_min
        config["short_max"] = short_max
        config["time_limit_ms"] = min(pass_limit_ms, remaining_ms)
        config["cache_geometry"] = (name in ("F", "A", "B"))
        config["partial_event_fractions"] = (name == "D")
        config["partial_event_limit"] = 11
        config["frame_hypothesis_search"] = name in ("F", "B", "C")
        # Exact-edge stage A is already tiny. The outer-frame feasibility
        # check pays off only once relaxed matching creates many branches.
        config["prune_outer_axis"] = (name != "A")
        # The physical target is fixed, so even the final fallback cannot grow
        # beyond its hard dimensions. Best-effort remains available inside it.
        config["prune_target_bounds"] = True
        if name in ("F", "A"):
            config["partial_slide_fractions"] = (0.0, 1.0)
            config["corner_tol"] = 10.0
            config["outer_axis_tol_deg"] = 12.0
            config["minimum_physical_edge"] = 16.0
            config["min_seam_coverage"] = 0.75
        elif name == "B":
            config["partial_slide_fractions"] = (0.0, 0.5, 1.0)
            config["corner_tol"] = 12.0
            config["edge_abs_tol"] = 6.0
            config["edge_rel_tol"] = 0.20
            config["outer_axis_tol_deg"] = 16.0
            config["minimum_physical_edge"] = 14.0
            config["min_seam_coverage"] = 0.55
        elif name == "C":
            config["partial_slide_fractions"] = (
                0.0, 0.25, 0.5, 0.75, 1.0)
            config["corner_tol"] = 18.0
            config["edge_abs_tol"] = 7.0
            config["edge_rel_tol"] = 0.22
            config["outer_axis_tol_deg"] = 22.0
            config["minimum_physical_edge"] = 12.0
            config["min_seam_coverage"] = 0.35
        else:
            config["partial_slide_fractions"] = (
                0.0, 0.25, 0.5, 0.75, 1.0)
            config["corner_tol"] = 18.0
            config["outer_axis_tol_deg"] = 30.0
            config["minimum_physical_edge"] = 12.0
            config["min_seam_coverage"] = 0.0
        config["state_dedup_limit"] = min(nodes, 5000)
        gc.collect()
        try:
            progress("09-%s" % name,
                     "search branch=%d nodes=%d frame=%s partial=%s ratio=%.2f edge=%.1f/%.0f%% bounds=%s limit=%dms free=%d bytes" %
                     (branch, nodes,
                      "on" if config["frame_hypothesis_search"] else "off",
                      "on" if allow_partial else "off",
                      partial_ratio, config["edge_abs_tol"],
                      config["edge_rel_tol"] * 100.0,
                      "on" if config["prune_target_bounds"] else "off",
                      config["time_limit_ms"], gc.mem_free()))
        except Exception:
            progress("09-%s" % name,
                     "search branch=%d nodes=%d frame=%s partial=%s ratio=%.2f edge=%.1f/%.0f%% bounds=%s limit=%dms" %
                     (branch, nodes,
                      "on" if config["frame_hypothesis_search"] else "off",
                      "on" if allow_partial else "off",
                      partial_ratio, config["edge_abs_tol"],
                      config["edge_rel_tol"] * 100.0,
                      "on" if config["prune_target_bounds"] else "off",
                      config["time_limit_ms"]))

        attempt_start_ms = time.ticks_ms()
        try:
            candidate = solve_puzzle(solver_input, config)
        except MemoryError as error:
            attempt_ms = time.ticks_diff(
                time.ticks_ms(), attempt_start_ms)
            progress("10-%s" % name,
                     "memory limited after %d ms: %s; previous best kept" %
                     (attempt_ms, str(error)))
            gc.collect()
            continue
        except Exception as error:
            attempt_ms = time.ticks_diff(
                time.ticks_ms(), attempt_start_ms)
            progress("10-%s" % name,
                     "stage failed after %d ms: %s; previous best kept" %
                     (attempt_ms, str(error)))
            gc.collect()
            continue
        attempt_ms = time.ticks_diff(time.ticks_ms(), attempt_start_ms)
        if candidate is None:
            progress("10-%s" % name, "no candidate (%d ms)" % attempt_ms)
            continue

        boundary = candidate.get("boundary_coverage", 0.0)
        minimum_side = candidate.get("minimum_side_coverage", 0.0)
        corners = candidate.get("corner_coverage", 0.0)
        valid = candidate.get("valid", True)
        edge_score = candidate.get("edge_score", candidate["score"])
        outer_count = candidate.get("outer_piece_count", 0)
        seam = candidate.get("seam_coverage", 0.0)
        target_dimension_error = candidate.get(
            "target_dimension_error",
            abs(candidate["width"] - TARGET_RECT_LONG_MM) +
            abs(candidate["height"] - TARGET_RECT_SHORT_MM))
        aspect_error = abs(candidate.get(
            "aspect_ratio", candidate["width"] / candidate["height"]) -
            TARGET_RECT_RATIO)
        # Requirement priority is explicit: valid layout, official size,
        # every piece on the outer frame, four corners, then geometric fit.
        rank = (
            0 if valid else 1,
            target_dimension_error,
            aspect_error,
            max(0, len(solver_input) - outer_count),
            max(0.0, 1.0 - corners),
            abs(1.0 - candidate["fill"]),
            max(0.0, 1.0 - seam),
            max(0.0, 1.0 - boundary),
            candidate["score"],
        )
        progress("10-%s" % name,
                 "%s %dx%d ratio=%.3f dimerr=%.1f nodes=%d fill=%.3f boundary=%.3f minside=%.3f corners=%d/4 outer=%d/%d seam=%.3f edge=%.2f score=%.2f (%d ms)" %
                 ("valid" if valid else "best-effort",
                  int(round(candidate["width"])),
                  int(round(candidate["height"])),
                  candidate.get("aspect_ratio", 0.0),
                  target_dimension_error,
                  candidate.get("search_nodes", 0), candidate["fill"],
                  boundary, minimum_side, int(round(corners * 4.0)),
                  outer_count, len(solver_input), seam, edge_score,
                  candidate["score"], attempt_ms))
        stats = candidate.get("search_stats", {})
        progress("10-%s" % name,
                 "frames=%d generated=%d overlap-cut=%d outer-cut=%d bounds-cut=%d duplicate-cut=%d score-cut=%d complete=%d SAT=%d general=%d" %
                 (stats.get("frame_hypotheses", 0),
                  stats.get("generated", 0),
                  stats.get("overlap_pruned", 0),
                  stats.get("outer_axis_pruned", 0),
                  stats.get("bounds_pruned", 0),
                  stats.get("duplicate_pruned", 0),
                  stats.get("score_pruned", 0),
                  stats.get("complete", 0),
                  stats.get("sat_checks", 0),
                  stats.get("general_overlap_checks", 0)))
        if best_rank is None or rank < best_rank:
            best_solution = candidate
            best_rank = rank

        reliable = (valid and candidate["fill"] >= 0.90 and
                    boundary >= 0.68 and
                    minimum_side >= 0.25 and
                    corners >= 0.999 and
                    outer_count == len(solver_input) and
                    seam >= 0.50 and
                    edge_score <= 4.0 and
                    candidate["score"] <= 12.0)
        if reliable:
            progress("10-%s" % name,
                     "quality accepted; wider search skipped")
            break
        progress("10-%s" % name,
                 "quality below target; expanding search")

    elapsed_ms = time.ticks_diff(time.ticks_ms(), total_start_ms)
    solution = best_solution
    if solution is None:
        progress("10", "no usable arrangement found (%d ms)" % elapsed_ms)
        return None
    progress("10", "%s selected in %d ms: source=%.1fx%.1f target=%.1fx%.1f mm vertical=%d fill=%.3f boundary=%.3f minside=%.3f corners=%d/4 nodes=%d" %
             ("valid arrangement" if solution.get("valid", True) else
              "best-effort arrangement",
              elapsed_ms, solution["width"], solution["height"],
               solution.get("target_width", solution["width"]),
               solution.get("target_height", solution["height"]),
               1 if solution.get("target_vertical", False) else 0,
               solution["fill"], solution.get("boundary_coverage", 0.0),
               solution.get("minimum_side_coverage", 0.0),
               int(round(solution.get("corner_coverage", 0.0) * 4.0)),
              solution.get("search_nodes", 0)))
    for placement in solution["placements"]:
        progress("11", "slot[%d] piece[%d] pick=(%.1f,%.1f) target=(%.1f,%.1f) angle=%.1f" %
                 (placement["slot"], placement["id"],
                  placement["pick"][0], placement["pick"][1],
                  placement["target"][0], placement["target"][1],
                  placement["angle"]))
    return solution


def hold_solution_preview(solution, mm_to_pixel):
    preview = sensor.snapshot()
    valid = solution.get("valid", True)
    progress("12", "%s preview active; reset camera to scan again" %
             ("valid blue target" if valid else "best-effort blue target"))
    for placement in solution["placements"]:
        px, py = project(mm_to_pixel, placement["target"][0],
                         placement["target"][1])
        visible = (0 <= px < preview.width() and 0 <= py < preview.height())
        progress("12", "slot[%d] target_px=(%.1f,%.1f) visible=%d" %
                 (placement["slot"], px, py, 1 if visible else 0))
    while True:
        draw_solution(preview, solution, mm_to_pixel)
        preview.draw_string(8, 8, "RESULT LOCKED", color=BLUE)
        preview.draw_string(8, 20,
                            "RECT SOLVED" if valid else "BEST EFFORT",
                            color=BLUE)
        time.sleep_ms(80)
        preview = sensor.snapshot()


# ---------------- Shared UART protocol ----------------
UART_RX_BUFFER_MAX = 256
uart_rx_buffer = ""


def poll_mcu_frames(uart_obj):
    """Non-blocking $...# frame reader shared with the multimode launcher."""
    global uart_rx_buffer

    if uart_obj is None:
        return []

    try:
        count = uart_obj.any()
    except Exception:
        count = 0

    if not count:
        return []

    try:
        raw = uart_obj.read(count)
    except Exception:
        return []

    if raw is None:
        return []

    try:
        if isinstance(raw, (bytes, bytearray)):
            try:
                chunk = raw.decode()
            except Exception:
                chunk = "".join(chr(value) for value in raw)
        else:
            chunk = str(raw)
    except Exception:
        chunk = ""

    if not chunk:
        return []

    uart_rx_buffer += chunk

    if len(uart_rx_buffer) > UART_RX_BUFFER_MAX:
        start = uart_rx_buffer.rfind("$")
        uart_rx_buffer = uart_rx_buffer[start:] if start >= 0 else ""

    frames = []

    while True:
        start = uart_rx_buffer.find("$")
        if start < 0:
            uart_rx_buffer = ""
            break

        if start > 0:
            uart_rx_buffer = uart_rx_buffer[start:]

        end = uart_rx_buffer.find("#", 1)
        if end < 0:
            break

        payload = uart_rx_buffer[1:end].strip()
        uart_rx_buffer = uart_rx_buffer[end + 1:]

        if payload:
            frames.append(payload)

    return frames


def send_uart_frame(uart_obj, payload):
    if uart_obj is None:
        return False

    frame = "$" + payload + "#\n"

    try:
        uart_obj.write(frame)
        progress("13", "UART TX " + frame.strip())
        return True
    except Exception as error:
        progress("99", "UART send failed: %s" % error)
        return False


def send_solution_to_mcu(uart_obj, solution):
    """Adapt solver placements to the existing STM32 TASK,n frame."""
    if solution is None:
        return False

    placements = list(solution.get("placements", []))

    if not (1 <= len(placements) <= 4):
        return False

    placements.sort(key=lambda item: item["slot"])
    groups = []

    for placement in placements:
        pick_x, pick_y = placement["pick"]
        target_x, target_y = placement["target"]
        angle = placement["angle"]

        groups.append(
            "%d,%.1f,%.1f,%.1f,%.1f,%.1f" %
            (
                placement["slot"],
                pick_x,
                pick_y,
                target_x,
                target_y,
                angle
            )
        )

    return send_uart_frame(
        uart_obj,
        "TASK,%d;%s" %
        (
            len(groups),
            ";".join(groups)
        )
    )


def run(uart_obj):
    """Run Requirement 2(2) until STM32 selects the next mode."""
    global uart_rx_buffer

    uart_rx_buffer = ""
    uart = uart_obj

    progress("00", "MODE 3: successful Requirement 2(2) solver starting")
    progress("01", "camera initialization started")
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(True)
    progress("01", "camera ready; shared UART enabled")
    progress("02", "ROI corner-noise guard active: %d px" %
             ROI_BORDER_GUARD_PX)
    progress("02", "large-piece limits: width<=%.0f%% height<=%.0f%% of ROI" %
             (MAX_PIECE_WIDTH_RATIO * 100.0,
              MAX_PIECE_HEIGHT_RATIO * 100.0))
    progress("02", "anti-glare exposure max=%dus; contour gap repair=%dpx" %
             (ANTI_GLARE_EXPOSURE_MAX_US, CARD_GAP_BRIDGE_PX))
    progress("02", "detect-first brightness threshold L>=50; fallback L>=38")
    if SOLVER_AVAILABLE:
        progress("02", "geometry_solver_req2_2.py loaded; shared UART enabled")
        progress("02", "target is vertical %.0fx%.0fmm on the left half" %
                 (TARGET_RECT_SHORT_MM, TARGET_RECT_LONG_MM))
        progress("02", "card-pattern matching=%s; contour repair=%s" %
                 ("on" if CARD_PATTERN_MODE else "off",
                   "on" if PATTERN_CONTOUR_REPAIR else "off"))
        progress("02", "convex piece outline=on; black inward notches ignored")
    else:
        progress("99", "geometry_solver_req2_2.py unavailable: %s" %
                 SOLVER_IMPORT_ERROR)

    calibration = load_calibration()
    stage = "DETECT" if calibration is not None else "CALIB"
    mm_to_pixel = invert_homography(calibration) if calibration is not None else None
    progress("03", "loaded calibration" if calibration is not None else
             "no calibration; show an empty A4 for calibration")

    clock = time.clock()
    frame_count = 0
    calib_stable = 0
    calib_samples = []
    last_corners = None
    last_corner_frame = -9999
    last_raw_piece_count = -1
    last_piece_count = -1
    piece_count_stable = 0
    last_count_area = None
    force_relaxed_detection = False
    last_report_frame = -9999
    exposure_locked = False
    a4_exposure_stable = 0
    solve_previous = None
    solve_stable = 0
    last_solve_frame = -9999
    current_solution = None
    last_stable_report = -1

    comm_state = "ARMED"
    last_sent_solution = None

    while True:
        try:
            for mcu_msg in poll_mcu_frames(uart):
                progress("13", "MCU RX " + mcu_msg)

                if mcu_msg.startswith("MODE,"):
                    try:
                        next_mode = int(mcu_msg.split(",", 1)[1])
                    except Exception:
                        next_mode = 0

                    if next_mode in (1, 2, 3):
                        progress(
                            "13",
                            "next mode %d selected; return to dispatcher" %
                            next_mode
                        )
                        return next_mode

                    continue

                if mcu_msg == "ACK":
                    comm_state = "WAIT_START"

                elif mcu_msg == "STARTED" or mcu_msg.startswith("RUN,"):
                    comm_state = "RUNNING"

                elif mcu_msg == "ALL_DONE":
                    comm_state = "WAIT_NEXT"
                    progress("13", "Requirement 2(2) motion complete")

                elif mcu_msg.startswith("ERR"):
                    progress("99", "STM32 error; unlock solver output")
                    comm_state = "ARMED"
                    last_sent_solution = None
                    current_solution = None
                    solve_previous = None
                    solve_stable = 0
                    last_stable_report = -1
                    last_raw_piece_count = -1
                    last_piece_count = -1
                    piece_count_stable = 0
                    last_count_area = None
                    force_relaxed_detection = False

            if comm_state != "ARMED":
                frame = sensor.snapshot()

                if last_sent_solution is not None and mm_to_pixel is not None:
                    draw_solution(frame, last_sent_solution, mm_to_pixel)

                if comm_state == "WAIT_ACK":
                    status_text = "TASK SENT"
                elif comm_state in ("WAIT_START", "RUNNING"):
                    status_text = "MCU RUNNING"
                else:
                    status_text = "WAIT NEXT MODE"

                frame.draw_string(8, 8, status_text, color=BLUE)
                time.sleep_ms(10)
                continue

            clock.tick()
            frame = sensor.snapshot()
            frame_count += 1
            if frame_count % (SKIP_FRAMES + 1) != 0:
                continue

            if stage == "CALIB":
                corners = detect_a4_corners(frame, RECT_THRESH_CALIB)
                if corners is None:
                    if calib_stable:
                        progress("03", "A4 lost; calibration counter reset")
                    calib_stable = 0
                    calib_samples = []
                    frame.draw_string(8, 8, "SHOW EMPTY A4", color=YELLOW)
                    continue
                labels = classify_corners(corners)
                source = [labels["BL"], labels["BR"],
                          labels["TL"], labels["TR"]]
                if calib_samples:
                    movement = 0.0
                    for old_point, new_point in zip(calib_samples[-1], source):
                        dx = old_point[0] - new_point[0]
                        dy = old_point[1] - new_point[1]
                        movement = max(movement, math.sqrt(dx * dx + dy * dy))
                    if movement > CALIB_CORNER_MOVE_TOL_PX:
                        progress("03", "A4 corners moved %.1fpx; calibration restarted" %
                                 movement)
                        calib_samples = []
                calib_samples.append(source)
                if len(calib_samples) > CALIB_STABLE_FRAMES:
                    calib_samples.pop(0)
                calib_stable = len(calib_samples)
                progress("03", "calibration stable %d/%d" %
                         (calib_stable, CALIB_STABLE_FRAMES))
                for name in labels:
                    frame.draw_cross(labels[name][0], labels[name][1],
                                     color=YELLOW, size=7)
                if calib_stable >= CALIB_STABLE_FRAMES:
                    source = []
                    for corner_index in range(4):
                        source.append((
                            sum(sample[corner_index][0]
                                for sample in calib_samples) /
                            float(len(calib_samples)),
                            sum(sample[corner_index][1]
                                for sample in calib_samples) /
                            float(len(calib_samples)),
                        ))
                    destination = [(0, 0), (A4_W, 0), (0, A4_H), (A4_W, A4_H)]
                    candidate = solve_homography(source, destination)
                    if candidate is not None and save_calibration(candidate):
                        calibration = candidate
                        mm_to_pixel = invert_homography(calibration)
                        stage = "DETECT"
                        progress("03", "calibration complete from 5-frame average; place 2-4 pieces on right")
                continue

            fresh_corners = detect_a4_corners(frame, RECT_THRESH_RUN)
            if fresh_corners is not None:
                corners = fresh_corners
                last_corners = fresh_corners
                last_corner_frame = frame_count
                a4_exposure_stable = min(
                    ANTI_GLARE_SETTLE_FRAMES, a4_exposure_stable + 1)
            elif (last_corners is not None and
                  frame_count - last_corner_frame <= STALE_A4_FRAMES):
                corners = last_corners
                a4_exposure_stable = 0
            else:
                corners = None
                a4_exposure_stable = 0
            if corners is None:
                frame.draw_string(8, 8, "WAIT A4", color=YELLOW)
                if frame_count % STATUS_INTERVAL_FRAMES == 0:
                    progress("04", "A4 not found; FPS=%.1f" % clock.fps())
                continue

            if (not exposure_locked and
                    a4_exposure_stable >= ANTI_GLARE_SETTLE_FRAMES):
                try:
                    measured_us = sensor.get_exposure_us()
                    exposure_us = min(
                        int(measured_us), int(ANTI_GLARE_EXPOSURE_MAX_US))
                    sensor.set_auto_exposure(False, exposure_us=exposure_us)
                    exposure_locked = True
                    progress(
                        "04",
                        "anti-glare exposure locked at %dus (measured=%dus max=%dus)" %
                        (exposure_us, measured_us, ANTI_GLARE_EXPOSURE_MAX_US))
                except Exception as error:
                    exposure_locked = True
                    progress("99", "anti-glare exposure lock unavailable: %s" %
                             error)

            labels = classify_corners(corners)
            roi = half_roi(labels, frame)
            frame.draw_rectangle(roi, color=CYAN, thickness=1)
            pieces, mode, primary_count, relaxed_count = detect_pieces(
                frame, roi, force_relaxed_detection)
            force_relaxed_detection = False

            if len(pieces) != last_raw_piece_count:
                progress("05", "raw piece candidates changed: %d" %
                         len(pieces))
                last_raw_piece_count = len(pieces)

            if len(pieces) < MIN_REQUIRED_PIECES:
                if solve_stable or current_solution is not None:
                    progress("08", "2-4-piece detection lost; solver stability reset")
                last_piece_count = -1
                piece_count_stable = 0
                last_count_area = None
                solve_previous = None
                solve_stable = 0
                current_solution = None
                last_stable_report = -1
                frame.draw_string(8, 8, "WAIT 2-4 PIECES", color=YELLOW)
                if frame_count % STATUS_INTERVAL_FRAMES == 0:
                    progress("05", "waiting for 2-4 pieces: primary=%d relaxed=%d FPS=%.1f" %
                             (primary_count, relaxed_count, clock.fps()))
                    report_detection_stats()
                continue

            all_observations = draw_and_collect(
                frame, pieces, calibration, mm_to_pixel)
            report_now = (last_report_frame < 0 or
                          frame_count - last_report_frame >=
                          REPORT_INTERVAL_FRAMES)
            if report_now:
                report_observations(
                    all_observations, mode, primary_count, relaxed_count)
                last_report_frame = frame_count

            (observations, total_area, area_ratio,
             raw_total_area, area_path) = select_area_confirmed_observations(
                 all_observations)

            if observations is None:
                last_piece_count = -1
                piece_count_stable = 0
                last_count_area = None
                solve_previous = None
                solve_stable = 0
                current_solution = None
                last_stable_report = -1

                if area_path == "AREA_LOW":
                    force_relaxed_detection = True
                    status_text = "CARD AREA LOW"
                elif area_path == "AREA_HIGH":
                    status_text = "CARD AREA HIGH"
                else:
                    status_text = "COUNT NOT CONFIRMED"

                frame.draw_string(8, 8, status_text, color=YELLOW)
                if report_now:
                    progress(
                        "08",
                        "COUNT_CHECK raw=%d selected=0 area=%.0fmm2 ratio=%.1f%% path=%s" %
                        (len(all_observations), raw_total_area,
                         area_ratio * 100.0, area_path))
                    report_detection_stats()
                continue

            selected_count = len(observations)
            area_stable = False
            if last_count_area is not None:
                area_scale = max(total_area, last_count_area)
                if area_scale > 1e-6:
                    area_stable = (
                        abs(total_area - last_count_area) / area_scale <=
                        COUNT_AREA_FRAME_TOL_RATIO)

            if selected_count == last_piece_count and area_stable:
                piece_count_stable = min(
                    PIECE_COUNT_SETTLE_FRAMES, piece_count_stable + 1)
            else:
                piece_count_stable = 1

            last_piece_count = selected_count
            last_count_area = total_area

            frame.draw_string(
                8, 8, "%d PIECES AREA %.0f%%" %
                (selected_count, area_ratio * 100.0), color=YELLOW)

            if (piece_count_stable < PIECE_COUNT_SETTLE_FRAMES or
                    report_now):
                progress(
                    "08",
                    "COUNT_CHECK raw=%d selected=%d area=%.0fmm2 ratio=%.1f%% stable=%d/%d path=%s" %
                    (len(all_observations), selected_count, total_area,
                     area_ratio * 100.0, piece_count_stable,
                     PIECE_COUNT_SETTLE_FRAMES, area_path))

            if frame_count % STATUS_INTERVAL_FRAMES == 0:
                progress("08", "%d confirmed pieces drawn; FPS=%.1f" %
                         (selected_count, clock.fps()))

            if piece_count_stable < PIECE_COUNT_SETTLE_FRAMES:
                solve_previous = None
                solve_stable = 0
                current_solution = None
                last_stable_report = -1
                frame.draw_string(
                    8, 20, "COUNT %d/%d" %
                    (piece_count_stable, PIECE_COUNT_SETTLE_FRAMES),
                    color=YELLOW)
                if piece_count_stable in (1, PIECE_COUNT_SETTLE_FRAMES // 2):
                    progress("08", "piece count+area %d settling %d/%d" %
                             (selected_count, piece_count_stable,
                              PIECE_COUNT_SETTLE_FRAMES))
                continue

            if LOCK_EXPOSURE_ON_STABLE_COUNT and not exposure_locked:
                try:
                    exposure_us = sensor.get_exposure_us()
                    sensor.set_auto_exposure(False, exposure_us=exposure_us)
                    exposure_locked = True
                    progress("05", "%d pieces stable; exposure locked at %dus" %
                             (selected_count, exposure_us))
                except Exception as error:
                    exposure_locked = True
                    progress("99", "exposure lock unavailable: %s" % error)

            if not SOLVER_AVAILABLE:
                frame.draw_string(8, 20, "NO SOLVER FILE", color=RED)
                continue

            input_issue = solver_input_issue(observations)
            if input_issue is not None:
                if (solve_stable or current_solution is not None or
                        frame_count % REPORT_INTERVAL_FRAMES == 0):
                    progress("08", "solver input rejected: %s; retracing" %
                             input_issue)
                solve_previous = None
                solve_stable = 0
                current_solution = None
                last_stable_report = -1
                frame.draw_string(8, 20, "RETRACE OUTLINE", color=YELLOW)
                continue

            if observations_compatible(solve_previous, observations):
                solve_stable = min(SOLVE_STABLE_FRAMES, solve_stable + 1)
                solve_previous = average_observations(
                    solve_previous, observations, solve_stable, mm_to_pixel)
            else:
                if solve_previous is not None:
                    progress("08", "piece geometry changed; stability restarted")
                solve_stable = 1
                current_solution = None
                solve_previous = copy_observations(observations)

            if solve_stable != last_stable_report:
                progress("08", "solver input stable %d/%d" %
                         (solve_stable, SOLVE_STABLE_FRAMES))
                last_stable_report = solve_stable
                if solve_stable >= SOLVE_STABLE_FRAMES:
                    report_stable_coordinates(solve_previous, solve_stable)

            if solve_stable < SOLVE_STABLE_FRAMES:
                frame.draw_string(8, 20, "HOLD %d/%d" %
                                  (solve_stable, SOLVE_STABLE_FRAMES),
                                  color=YELLOW)
                continue

            if (current_solution is None and
                    frame_count - last_solve_frame >= SOLVE_RETRY_FRAMES):
                last_solve_frame = frame_count
                current_solution = run_solver(solve_previous)
                if current_solution is not None:
                    draw_solution(frame, current_solution, mm_to_pixel)
                    frame.draw_string(
                        8, 20,
                        "RECT SOLVED" if current_solution.get("valid", True)
                        else "BEST EFFORT", color=BLUE)
                    if send_solution_to_mcu(uart, current_solution):
                        last_sent_solution = current_solution
                        comm_state = "WAIT_ACK"
                        progress(
                            "13",
                            "Requirement 2(2) task sent; STM32 will start immediately"
                        )
                    else:
                        progress("99", "solution UART transmission failed")
                        current_solution = None

            if current_solution is not None:
                draw_solution(frame, current_solution, mm_to_pixel)
                frame.draw_string(8, 20, "RECT SOLVED", color=BLUE)
            else:
                frame.draw_string(8, 20, "NO RECT SOLUTION", color=RED)

        except Exception as error:
            progress("99", "runtime error: %s" % error)
            time.sleep_ms(100)
