"""Small-N polygon puzzle solver for OpenMV/MicroPython.

The solver only uses Python lists, dicts, sets and ``math``.  It assembles up
to four observed polygons by pairing equal-length edges with rotation and
translation only (reflection is never allowed), then verifies that their
union fills a rectangle.

Input piece format::

    {
        "id": 0,
        "polygon": [(x0, y0), ...],  # millimetres, counter-clockwise
        "pick": (cx, cy),            # current grasp centre in A4 coordinates
    }

``solve_puzzle`` returns None or a dictionary containing canonical target
placements.
"""

import math
import time


DEFAULT_CONFIG = {
    "edge_abs_tol": 4.0,
    "edge_rel_tol": 0.15,
    "partial_edge_min_ratio": 0.12,
    # If the unmatched remainder is this long, an apparently equal pair may
    # also be one arm of a T junction. Keep both interpretations available.
    "partial_remainder_min": 10.0,
    "pattern_weight": 2.5,
    "pattern_min_informative": 3,
    "partial_slide_fractions": (0.0, 1.0),
    "partial_event_fractions": False,
    "partial_event_limit": 11,
    "minimum_physical_edge": 12.0,
    "allow_partial_edges": True,
    "intersection_tol": 0.8,
    "outer_tol": 4.0,
    "min_fill": 0.87,
    "max_fill": 1.06,
    "long_min": 88.0,
    "long_max": 122.0,
    "short_min": 48.0,
    "short_max": 92.0,
    # Optional fixed-target limits. Broad defaults preserve generic use;
    # position_detector.py supplies the measured competition dimensions.
    "hard_long_min": 0.0,
    "hard_long_max": 1e9,
    "hard_short_min": 0.0,
    "hard_short_max": 1e9,
    "aspect_ratio_min": 0.0,
    "aspect_ratio_max": 1e9,
    "target_long": 0.0,
    "target_short": 0.0,
    "target_dimension_weight": 0.0,
    "target_aspect_weight": 0.0,
    "convex_pieces_only": False,
    "beam_width": 320,
    "target_center": (74.25, 105.0),
    "target_vertical": False,
    "low_memory": False,
    "low_memory_max_nodes": 16000,
    "state_dedup_limit": 5000,
    "state_angle_quant_deg": 0.25,
    "state_position_quant_mm": 0.25,
    "prune_target_bounds": False,
    "partial_bound_slack": 8.0,
    "corner_first_root": True,
    "frame_hypothesis_search": False,
    "prune_outer_axis": True,
    "outer_axis_tol_deg": 16.0,
    "frame_rank_weight": 0.35,
    "seam_tol": 3.0,
    "min_seam_coverage": 0.0,
    "seam_weight": 10.0,
    "side_weight": 3.0,
    "missing_outer_piece_penalty": 60.0,
}


def _ticks_ms():
    try:
        return time.ticks_ms()
    except AttributeError:
        return int(time.monotonic() * 1000.0)


def _ticks_diff(current, previous):
    try:
        return time.ticks_diff(current, previous)
    except AttributeError:
        return current - previous


def trace_binary_boundary(mask, seed_x, seed_y):
    """Trace an 8-connected foreground boundary in a byte-row mask."""
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

    directions = [
        (1, 0), (1, 1), (0, 1), (-1, 1),
        (-1, 0), (-1, -1), (0, -1), (1, -1),
    ]
    start = (x, y)
    current = start
    backtrack = (x - 1, y)
    first_next = None
    points = [start]

    for _ in range(width * height * 4):
        back_direction = (
            backtrack[0] - current[0], backtrack[1] - current[1])
        try:
            back_index = directions.index(back_direction)
        except ValueError:
            back_index = 4

        next_point = None
        for step in range(1, 9):
            direction_index = (back_index + step) % 8
            dx, dy = directions[direction_index]
            next_x = current[0] + dx
            next_y = current[1] + dy
            if (0 <= next_x < width and 0 <= next_y < height and
                    mask[next_y][next_x]):
                previous_index = (direction_index - 1) % 8
                previous_dx, previous_dy = directions[previous_index]
                backtrack = (
                    current[0] + previous_dx,
                    current[1] + previous_dy)
                next_point = (next_x, next_y)
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


def _config(user):
    cfg = dict(DEFAULT_CONFIG)
    if user:
        for key in user:
            cfg[key] = user[key]
    return cfg


def _dist(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def _cross(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) -
            (b[1] - a[1]) * (c[0] - a[0]))


def polygon_area(poly):
    value = 0.0
    count = len(poly)
    for i in range(count):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % count]
        value += x0 * y1 - x1 * y0
    return 0.5 * value


def ensure_ccw(poly):
    out = [(float(p[0]), float(p[1])) for p in poly]
    if polygon_area(out) < 0.0:
        out.reverse()
    return out


def _convex_hull(points):
    """Return the outer convex envelope, dropping all inward notches."""
    if len(points) <= 2:
        return points[:]
    ordered = sorted((float(point[0]), float(point[1]))
                     for point in points)
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


def polygon_centroid(poly):
    twice_area = 0.0
    cx = 0.0
    cy = 0.0
    count = len(poly)
    for i in range(count):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % count]
        term = x0 * y1 - x1 * y0
        twice_area += term
        cx += (x0 + x1) * term
        cy += (y0 + y1) * term
    if abs(twice_area) < 1e-9:
        return (sum(p[0] for p in poly) / count,
                sum(p[1] for p in poly) / count)
    return (cx / (3.0 * twice_area), cy / (3.0 * twice_area))


def _edge(poly, edge_index):
    return poly[edge_index], poly[(edge_index + 1) % len(poly)]


def _edge_length(poly, edge_index):
    a, b = _edge(poly, edge_index)
    return _dist(a, b)


def _right_angle_quality(poly):
    """Return likely rectangle-corner count and the best cosine error."""
    count = len(poly)
    likely = 0
    best_error = 1.0
    for index in range(count):
        previous = poly[(index - 1) % count]
        current = poly[index]
        following = poly[(index + 1) % count]
        ax = previous[0] - current[0]
        ay = previous[1] - current[1]
        bx = following[0] - current[0]
        by = following[1] - current[1]
        length_product = math.sqrt(
            (ax * ax + ay * ay) * (bx * bx + by * by))
        if length_product <= 1e-9:
            continue
        error = abs((ax * bx + ay * by) / length_product)
        best_error = min(best_error, error)
        # About 72..108 degrees. Camera vertex noise is allowed here because
        # this only orders roots; it never rejects a piece.
        if error <= 0.31:
            likely += 1
    return likely, best_error


def _ordered_search_ids(by_id, ids, cfg):
    """Start from a likely outer-corner piece without removing any branch."""
    if not cfg.get("corner_first_root", True):
        return sorted(ids, key=lambda piece_id: (
            by_id[piece_id]["area"], piece_id))

    ranked = []
    for piece_id in ids:
        likely, error = _right_angle_quality(by_id[piece_id]["polygon"])
        ranked.append((
            -min(likely, 1),
            by_id[piece_id]["area"],
            error,
            piece_id,
        ))
    ranked.sort()
    return [item[3] for item in ranked]


def _identity():
    return (1.0, 0.0, 0.0, 0.0)


def _compose_transform(outer, inner):
    """Return the transform produced by applying inner, then outer."""
    ac, ass, atx, aty = outer
    bc, bs, btx, bty = inner
    return (
        ac * bc - ass * bs,
        ass * bc + ac * bs,
        ac * btx - ass * bty + atx,
        ass * btx + ac * bty + aty,
    )


def _apply(transform, point):
    c, s, tx, ty = transform
    x, y = point
    return (c * x - s * y + tx, s * x + c * y + ty)


def _apply_poly(transform, poly):
    return [_apply(transform, point) for point in poly]


def _transform_angle(transform):
    return math.atan2(transform[1], transform[0])


def _align_reversed(new_poly, new_edge, placed_poly, placed_edge,
                    anchor_end=False, anchor_fraction=None):
    """Align opposite edge directions at a selected free-length fraction."""
    a0, a1 = _edge(new_poly, new_edge)
    b0, b1 = _edge(placed_poly, placed_edge)
    avx = a1[0] - a0[0]
    avy = a1[1] - a0[1]
    bvx = b0[0] - b1[0]
    bvy = b0[1] - b1[1]
    angle = math.atan2(bvy, bvx) - math.atan2(avy, avx)
    c = math.cos(angle)
    s = math.sin(angle)
    if anchor_fraction is None:
        fraction = 1.0 if anchor_end else 0.0
    else:
        fraction = max(0.0, min(1.0, float(anchor_fraction)))
    new_length = math.sqrt(avx * avx + avy * avy)
    old_length = math.sqrt(bvx * bvx + bvy * bvy)
    direction_length = max(old_length, 1e-9)
    unit_x = bvx / direction_length
    unit_y = bvy / direction_length
    difference = abs(old_length - new_length)
    source = a0
    if new_length <= old_length:
        target = (b1[0] + unit_x * difference * fraction,
                  b1[1] + unit_y * difference * fraction)
    else:
        target = (b1[0] - unit_x * difference * fraction,
                  b1[1] - unit_y * difference * fraction)
    rx = c * source[0] - s * source[1]
    ry = s * source[0] + c * source[1]
    return (c, s, target[0] - rx, target[1] - ry)


def _point_segment_distance(point, a, b):
    vx = b[0] - a[0]
    vy = b[1] - a[1]
    denom = vx * vx + vy * vy
    if denom <= 1e-12:
        return _dist(point, a)
    t = ((point[0] - a[0]) * vx + (point[1] - a[1]) * vy) / denom
    if t < 0.0:
        t = 0.0
    elif t > 1.0:
        t = 1.0
    projected = (a[0] + t * vx, a[1] + t * vy)
    return _dist(point, projected)


def _on_boundary(point, poly, tol):
    for i in range(len(poly)):
        a, b = _edge(poly, i)
        if _point_segment_distance(point, a, b) <= tol:
            return True
    return False


def _point_in_polygon(point, poly, tol):
    if _on_boundary(point, poly, tol):
        return False
    x, y = point
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)):
            cross_x = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < cross_x:
                inside = not inside
        j = i
    return inside


def _proper_intersection(a, b, c, d, tol):
    """True only for an interior crossing; touching/collinear is allowed."""
    ab_c = _cross(a, b, c)
    ab_d = _cross(a, b, d)
    cd_a = _cross(c, d, a)
    cd_b = _cross(c, d, b)
    scale = max(_dist(a, b), _dist(c, d), 1.0)
    eps = tol * scale
    if abs(ab_c) <= eps or abs(ab_d) <= eps:
        return False
    if abs(cd_a) <= eps or abs(cd_b) <= eps:
        return False
    return ((ab_c > 0.0) != (ab_d > 0.0) and
            (cd_a > 0.0) != (cd_b > 0.0))


def _polygon_bbox(poly):
    return (
        min(point[0] for point in poly),
        max(point[0] for point in poly),
        min(point[1] for point in poly),
        max(point[1] for point in poly),
    )


def _polygon_is_convex(poly):
    direction = 0
    for index in range(len(poly)):
        value = _cross(
            poly[index], poly[(index + 1) % len(poly)],
            poly[(index + 2) % len(poly)])
        if abs(value) <= 1e-7:
            continue
        current = 1 if value > 0.0 else -1
        if direction and current != direction:
            return False
        direction = current
    return direction != 0


def _polygon_sat_axes(poly):
    axes = []
    for index in range(len(poly)):
        a, b = _edge(poly, index)
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        length = math.sqrt(dx * dx + dy * dy)
        if length <= 1e-9:
            continue
        normal = (-dy / length, dx / length)
        duplicate = False
        for old_x, old_y in axes:
            if abs(normal[0] * old_x + normal[1] * old_y) >= 0.9999:
                duplicate = True
                break
        if not duplicate:
            axes.append(normal)
    return axes


def _transform_axes(transform, axes):
    c, s = transform[0], transform[1]
    return [(c * x - s * y, s * x + c * y) for x, y in axes]


def _convex_polygons_overlap(poly_a, poly_b, tol, axes_a, axes_b):
    """Exact convex overlap test; touching within tolerance is allowed."""
    for axis_x, axis_y in axes_a + axes_b:
        first = poly_a[0][0] * axis_x + poly_a[0][1] * axis_y
        min_a = first
        max_a = first
        for x, y in poly_a[1:]:
            value = x * axis_x + y * axis_y
            if value < min_a:
                min_a = value
            if value > max_a:
                max_a = value
        first = poly_b[0][0] * axis_x + poly_b[0][1] * axis_y
        min_b = first
        max_b = first
        for x, y in poly_b[1:]:
            value = x * axis_x + y * axis_y
            if value < min_b:
                min_b = value
            if value > max_b:
                max_b = value
        if max_a <= min_b + tol or max_b <= min_a + tol:
            return False
    return True


def _bboxes_separated(bbox_a, bbox_b, tol):
    min_ax, max_ax, min_ay, max_ay = bbox_a
    min_bx, max_bx, min_by, max_by = bbox_b
    return (max_ax <= min_bx + tol or max_bx <= min_ax + tol or
            max_ay <= min_by + tol or max_by <= min_ay + tol)


def _polygons_overlap(poly_a, poly_b, tol, bbox_a=None, bbox_b=None):
    bbox_a = bbox_a or _polygon_bbox(poly_a)
    bbox_b = bbox_b or _polygon_bbox(poly_b)
    if _bboxes_separated(bbox_a, bbox_b, tol):
        return False
    for i in range(len(poly_a)):
        a0, a1 = _edge(poly_a, i)
        for j in range(len(poly_b)):
            b0, b1 = _edge(poly_b, j)
            if _proper_intersection(a0, a1, b0, b1, tol):
                return True
    for point in poly_a:
        if _point_in_polygon(point, poly_b, tol):
            return True
    for point in poly_b:
        if _point_in_polygon(point, poly_a, tol):
            return True
    # Coincident polygons have every vertex on the other boundary, so the
    # vertex tests above cannot see their positive-area overlap. Probe just
    # inside each CCW edge; shared seams probe opposite sides and remain valid.
    probe_distance = max(tol * 2.25, 0.15)
    for source, other in ((poly_a, poly_b), (poly_b, poly_a)):
        for i in range(len(source)):
            a, b = _edge(source, i)
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            length = math.sqrt(dx * dx + dy * dy)
            if length <= probe_distance * 3.0:
                continue
            midpoint = ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5)
            probe = (midpoint[0] - dy / length * probe_distance,
                     midpoint[1] + dx / length * probe_distance)
            if _point_in_polygon(probe, source, tol * 0.25) and \
                    _point_in_polygon(probe, other, tol * 0.25):
                return True
    return False


def _state_polygons(state, by_id):
    cached = state.get("polygons")
    if cached is not None:
        return cached
    polygons = {}
    for piece_id in state["placed"]:
        polygons[piece_id] = _apply_poly(
            state["placed"][piece_id], by_id[piece_id]["polygon"])
    return polygons


def _all_axis_angles(polygons):
    values = []
    for poly in polygons.values():
        for i in range(len(poly)):
            a, b = _edge(poly, i)
            angle = math.atan2(b[1] - a[1], b[0] - a[0])
            angle %= (math.pi * 0.5)
            duplicate = False
            for old in values:
                if abs(angle - old) < math.radians(1.0):
                    duplicate = True
                    break
            if not duplicate:
                values.append(angle)
    return values


def _rect_at_angle(polygons, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    xs = []
    ys = []
    for poly in polygons.values():
        for x, y in poly:
            xs.append(c * x + s * y)
            ys.append(-s * x + c * y)
    return {
        "angle": angle,
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y": min(ys),
        "max_y": max(ys),
        "width": max(xs) - min(xs),
        "height": max(ys) - min(ys),
    }


def _best_partial_rect(polygons, area_sum):
    best = None
    for angle in _all_axis_angles(polygons):
        rect = _rect_at_angle(polygons, angle)
        excess = max(0.0, rect["width"] * rect["height"] - area_sum)
        if best is None or excess < best[0]:
            best = (excess, rect)
    if best is None:
        return 1e9, None
    return best


def _normalise_rect(polygons, rect):
    if rect["width"] >= rect["height"]:
        return rect
    return _rect_at_angle(polygons, rect["angle"] + math.pi * 0.5)


def _edge_is_outer(poly, rect, tol):
    c = math.cos(rect["angle"])
    s = math.sin(rect["angle"])
    for i in range(len(poly)):
        a, b = _edge(poly, i)
        ax = c * a[0] + s * a[1]
        ay = -s * a[0] + c * a[1]
        bx = c * b[0] + s * b[1]
        by = -s * b[0] + c * b[1]
        if abs(ax - rect["min_x"]) <= tol and abs(bx - rect["min_x"]) <= tol:
            return True
        if abs(ax - rect["max_x"]) <= tol and abs(bx - rect["max_x"]) <= tol:
            return True
        if abs(ay - rect["min_y"]) <= tol and abs(by - rect["min_y"]) <= tol:
            return True
        if abs(ay - rect["max_y"]) <= tol and abs(by - rect["max_y"]) <= tol:
            return True
    return False


def _covered_length(intervals, minimum, maximum):
    if not intervals:
        return 0.0
    clipped = []
    for start, end in intervals:
        start = max(minimum, min(maximum, start))
        end = max(minimum, min(maximum, end))
        if end > start:
            clipped.append((start, end))
    if not clipped:
        return 0.0
    clipped.sort(key=lambda item: item[0])
    covered = 0.0
    start, end = clipped[0]
    for next_start, next_end in clipped[1:]:
        if next_start <= end:
            end = max(end, next_end)
        else:
            covered += end - start
            start, end = next_start, next_end
    return covered + end - start


def _outer_coverages(polygons, rect, tol):
    c = math.cos(rect["angle"])
    s = math.sin(rect["angle"])
    sides = [[], [], [], []]
    for poly in polygons.values():
        for index in range(len(poly)):
            a, b = _edge(poly, index)
            ax = c * a[0] + s * a[1]
            ay = -s * a[0] + c * a[1]
            bx = c * b[0] + s * b[1]
            by = -s * b[0] + c * b[1]
            if (abs(ay - rect["min_y"]) <= tol and
                    abs(by - rect["min_y"]) <= tol):
                sides[0].append((min(ax, bx), max(ax, bx)))
            if (abs(ay - rect["max_y"]) <= tol and
                    abs(by - rect["max_y"]) <= tol):
                sides[1].append((min(ax, bx), max(ax, bx)))
            if (abs(ax - rect["min_x"]) <= tol and
                    abs(bx - rect["min_x"]) <= tol):
                sides[2].append((min(ay, by), max(ay, by)))
            if (abs(ax - rect["max_x"]) <= tol and
                    abs(bx - rect["max_x"]) <= tol):
                sides[3].append((min(ay, by), max(ay, by)))
    lengths = [
        _covered_length(sides[0], rect["min_x"], rect["max_x"]),
        _covered_length(sides[1], rect["min_x"], rect["max_x"]),
        _covered_length(sides[2], rect["min_y"], rect["max_y"]),
        _covered_length(sides[3], rect["min_y"], rect["max_y"]),
    ]
    covered = sum(lengths)
    perimeter = 2.0 * (rect["width"] + rect["height"])
    total = covered / perimeter if perimeter > 1e-9 else 0.0
    side_coverages = [
        lengths[0] / rect["width"] if rect["width"] > 1e-9 else 0.0,
        lengths[1] / rect["width"] if rect["width"] > 1e-9 else 0.0,
        lengths[2] / rect["height"] if rect["height"] > 1e-9 else 0.0,
        lengths[3] / rect["height"] if rect["height"] > 1e-9 else 0.0,
    ]
    return total, side_coverages


def _outer_coverage(polygons, rect, tol):
    return _outer_coverages(polygons, rect, tol)[0]


def _outer_metrics(polygons, rect, tol, corner_tol):
    """Compute all outer-boundary metrics from one coordinate projection."""
    c = math.cos(rect["angle"])
    s = math.sin(rect["angle"])
    sides = [[], [], [], []]
    outer_count = 0
    corners = (
        (rect["min_x"], rect["min_y"]),
        (rect["max_x"], rect["min_y"]),
        (rect["max_x"], rect["max_y"]),
        (rect["min_x"], rect["max_y"]),
    )
    corner_tolerance_squared = corner_tol * corner_tol
    corner_found = [False, False, False, False]

    for poly in polygons.values():
        local = []
        for x, y in poly:
            local_x = c * x + s * y
            local_y = -s * x + c * y
            local.append((local_x, local_y))
            for corner_index in range(4):
                if corner_found[corner_index]:
                    continue
                corner_x, corner_y = corners[corner_index]
                dx = local_x - corner_x
                dy = local_y - corner_y
                if dx * dx + dy * dy <= corner_tolerance_squared:
                    corner_found[corner_index] = True
        piece_outer = False
        for index in range(len(local)):
            ax, ay = local[index]
            bx, by = local[(index + 1) % len(local)]
            if (abs(ay - rect["min_y"]) <= tol and
                    abs(by - rect["min_y"]) <= tol):
                sides[0].append((min(ax, bx), max(ax, bx)))
                piece_outer = True
            if (abs(ay - rect["max_y"]) <= tol and
                    abs(by - rect["max_y"]) <= tol):
                sides[1].append((min(ax, bx), max(ax, bx)))
                piece_outer = True
            if (abs(ax - rect["min_x"]) <= tol and
                    abs(bx - rect["min_x"]) <= tol):
                sides[2].append((min(ay, by), max(ay, by)))
                piece_outer = True
            if (abs(ax - rect["max_x"]) <= tol and
                    abs(bx - rect["max_x"]) <= tol):
                sides[3].append((min(ay, by), max(ay, by)))
                piece_outer = True
        if piece_outer:
            outer_count += 1

    lengths = [
        _covered_length(sides[0], rect["min_x"], rect["max_x"]),
        _covered_length(sides[1], rect["min_x"], rect["max_x"]),
        _covered_length(sides[2], rect["min_y"], rect["max_y"]),
        _covered_length(sides[3], rect["min_y"], rect["max_y"]),
    ]
    perimeter = 2.0 * (rect["width"] + rect["height"])
    boundary_coverage = (
        sum(lengths) / perimeter if perimeter > 1e-9 else 0.0)
    side_coverages = [
        lengths[0] / rect["width"] if rect["width"] > 1e-9 else 0.0,
        lengths[1] / rect["width"] if rect["width"] > 1e-9 else 0.0,
        lengths[2] / rect["height"] if rect["height"] > 1e-9 else 0.0,
        lengths[3] / rect["height"] if rect["height"] > 1e-9 else 0.0,
    ]

    return (boundary_coverage, side_coverages,
            sum(corner_found) / 4.0,
            outer_count)


def _collinear_contact_length(a, b, c, d, tol):
    """Length shared by two nearly collinear segments."""
    abx = b[0] - a[0]
    aby = b[1] - a[1]
    cdx = d[0] - c[0]
    cdy = d[1] - c[1]
    ab_length = math.sqrt(abx * abx + aby * aby)
    cd_length = math.sqrt(cdx * cdx + cdy * cdy)
    if ab_length <= 1e-9 or cd_length <= 1e-9:
        return 0.0
    direction_error = abs(abx * cdy - aby * cdx) / (
        ab_length * cd_length)
    if direction_error > 0.14:
        return 0.0
    distance_c = abs(_cross(a, b, c)) / ab_length
    distance_d = abs(_cross(a, b, d)) / ab_length
    if distance_c > tol or distance_d > tol:
        return 0.0
    unit_x = abx / ab_length
    unit_y = aby / ab_length
    tc = (c[0] - a[0]) * unit_x + (c[1] - a[1]) * unit_y
    td = (d[0] - a[0]) * unit_x + (d[1] - a[1]) * unit_y
    overlap_start = max(0.0, min(tc, td))
    overlap_end = min(ab_length, max(tc, td))
    return max(0.0, overlap_end - overlap_start)


def _internal_seam_coverage(polygons, rect, tol):
    """Compare measured pair contacts with the seam length a rectangle needs."""
    piece_ids = list(polygons.keys())
    perimeter_sum = 0.0
    for poly in polygons.values():
        for edge_index in range(len(poly)):
            perimeter_sum += _edge_length(poly, edge_index)

    expected = max(
        0.0,
        (perimeter_sum - 2.0 * (rect["width"] + rect["height"])) * 0.5)
    if expected <= 1e-6:
        return 1.0

    contact = 0.0
    for first_index in range(len(piece_ids)):
        first = polygons[piece_ids[first_index]]
        for second_index in range(first_index + 1, len(piece_ids)):
            second = polygons[piece_ids[second_index]]
            for first_edge in range(len(first)):
                a, b = _edge(first, first_edge)
                for second_edge in range(len(second)):
                    c, d = _edge(second, second_edge)
                    contact += _collinear_contact_length(
                        a, b, c, d, tol)
    return min(1.0, contact / expected)


def _outer_corner_coverage(polygons, rect, tol):
    c = math.cos(rect["angle"])
    s = math.sin(rect["angle"])
    corners = [
        (rect["min_x"], rect["min_y"]),
        (rect["max_x"], rect["min_y"]),
        (rect["max_x"], rect["max_y"]),
        (rect["min_x"], rect["max_y"]),
    ]
    covered = 0
    for corner_x, corner_y in corners:
        found = False
        for poly in polygons.values():
            for x, y in poly:
                local_x = c * x + s * y
                local_y = -s * x + c * y
                dx = local_x - corner_x
                dy = local_y - corner_y
                if dx * dx + dy * dy <= tol * tol:
                    found = True
                    break
            if found:
                break
        if found:
            covered += 1
    return covered / 4.0


def _evaluate_complete(state, by_id, cfg, total_area):
    """Evaluate strict and best-effort quality in one geometry pass."""
    polygons = _state_polygons(state, by_id)
    valid_best = None
    fallback_best = None
    frame_axis = state.get("frame_axis")
    axes = ([frame_axis] if frame_axis is not None
            else _all_axis_angles(polygons))
    for axis in axes:
        rect = _normalise_rect(polygons, _rect_at_angle(polygons, axis))
        width = rect["width"]
        height = rect["height"]
        rect_area = width * height
        if rect_area <= 1e-9:
            continue
        aspect_ratio = width / height
        hard_dimensions_ok = (
            float(cfg.get("hard_long_min", 0.0)) <= width <=
            float(cfg.get("hard_long_max", 1e9)) and
            float(cfg.get("hard_short_min", 0.0)) <= height <=
            float(cfg.get("hard_short_max", 1e9)) and
            float(cfg.get("aspect_ratio_min", 0.0)) <= aspect_ratio <=
            float(cfg.get("aspect_ratio_max", 1e9)))
        # A geometrically tidy rectangle with the wrong physical size is not
        # a useful fallback. Reject it before the more expensive seam checks.
        if not hard_dimensions_ok:
            continue
        target_long = float(cfg.get("target_long", 0.0))
        target_short = float(cfg.get("target_short", 0.0))
        target_dimension_error = 0.0
        target_aspect_error = 0.0
        if target_long > 0.0:
            target_dimension_error += abs(width - target_long)
        if target_short > 0.0:
            target_dimension_error += abs(height - target_short)
        if target_long > 0.0 and target_short > 0.0:
            target_aspect_error = abs(
                aspect_ratio - target_long / target_short)
        fill = total_area / rect_area
        (boundary_coverage, side_coverages,
         corner_coverage, outer_count) = _outer_metrics(
            polygons, rect, cfg["outer_tol"],
            float(cfg.get("corner_tol", cfg["outer_tol"])))
        minimum_side_coverage = min(side_coverages)
        seam_coverage = _internal_seam_coverage(
            polygons, rect, float(cfg.get("seam_tol", 3.0)))

        dimension_error = (
            _outside_range(width, cfg["long_min"], cfg["long_max"]) +
            _outside_range(height, cfg["short_min"], cfg["short_max"]))
        fallback_score = (
            state["score"] + abs(1.0 - fill) * 140.0 +
            dimension_error * 2.0 +
            target_dimension_error *
            float(cfg.get("target_dimension_weight", 0.0)) +
            target_aspect_error *
            float(cfg.get("target_aspect_weight", 0.0)) +
            (len(polygons) - outer_count) *
            float(cfg.get("missing_outer_piece_penalty", 60.0)) +
            max(0.0, 1.0 - boundary_coverage) * 12.0 +
            max(0.0, 1.0 - minimum_side_coverage) * 6.0 +
            max(0.0, 1.0 - corner_coverage) * 40.0 +
            max(0.0, 1.0 - seam_coverage) *
            float(cfg.get("seam_weight", 10.0)))
        fallback = (fallback_score, rect, polygons, fill,
                    boundary_coverage, corner_coverage,
                    minimum_side_coverage, outer_count,
                    seam_coverage)
        if fallback_best is None or fallback_score < fallback_best[0]:
            fallback_best = fallback

        dimensions_ok = (
            cfg["long_min"] <= width <= cfg["long_max"] and
            cfg["short_min"] <= height <= cfg["short_max"])
        fill_ok = cfg["min_fill"] <= fill <= cfg["max_fill"]
        each_outer_ok = (
            not cfg.get("require_each_outer", True) or
            outer_count == len(polygons))
        boundary_ok = boundary_coverage >= float(
            cfg.get("min_boundary_coverage", 0.0))
        side_ok = minimum_side_coverage >= float(
            cfg.get("min_outer_side_coverage", 0.0))
        corners_ok = (
            not cfg.get("require_all_outer_corners", False) or
            corner_coverage >= 0.999)
        seam_ok = seam_coverage >= float(
            cfg.get("min_seam_coverage", 0.0))
        if not (dimensions_ok and fill_ok and each_outer_ok and
                boundary_ok and side_ok and corners_ok and seam_ok):
            continue

        score = (
            state["score"] + abs(1.0 - fill) * 140.0 +
            target_dimension_error *
            float(cfg.get("target_dimension_weight", 0.0)) +
            target_aspect_error *
            float(cfg.get("target_aspect_weight", 0.0)) +
            max(0.0, 1.0 - boundary_coverage) *
            float(cfg.get("boundary_weight", 0.0)) +
            max(0.0, 1.0 - minimum_side_coverage) *
            float(cfg.get("side_weight", 0.0)) +
            max(0.0, 1.0 - seam_coverage) *
            float(cfg.get("seam_weight", 0.0)))
        valid = (score, rect, polygons, fill, boundary_coverage,
                 corner_coverage, minimum_side_coverage,
                 outer_count, seam_coverage)
        if valid_best is None or score < valid_best[0]:
            valid_best = valid
    return valid_best, fallback_best


def _final_rect(state, by_id, cfg, total_area):
    return _evaluate_complete(state, by_id, cfg, total_area)[0]


def _outside_range(value, minimum, maximum):
    if value < minimum:
        return minimum - value
    if value > maximum:
        return value - maximum
    return 0.0


def _best_effort_rect(state, by_id, cfg, total_area):
    """Score every complete non-overlapping assembly, even if not valid."""
    return _evaluate_complete(state, by_id, cfg, total_area)[1]


def _state_key(state, cfg=None):
    cfg = cfg or DEFAULT_CONFIG
    angle_step = max(0.01, float(cfg.get("state_angle_quant_deg", 0.25)))
    position_step = max(0.01, float(cfg.get("state_position_quant_mm", 0.25)))
    parts = []
    for piece_id in sorted(state["placed"]):
        transform = state["placed"][piece_id]
        angle = int(round(
            math.degrees(_transform_angle(transform)) / angle_step))
        tx = int(round(transform[2] / position_step))
        ty = int(round(transform[3] / position_step))
        parts.append((piece_id, angle, tx, ty))
    used = tuple(sorted(state["used"]))
    frame_axis = state.get("frame_axis")
    frame_key = (None if frame_axis is None else int(round(
        math.degrees(frame_axis) / angle_step)))
    return (tuple(parts), used, frame_key)


def _beam_rank(state, by_id, placed_area):
    polygons = _state_polygons(state, by_id)
    excess, _ = _best_partial_rect(polygons, placed_area)
    return state["score"] + excess * 0.0015


def _search_rank(state, by_id):
    placed_area = 0.0
    for piece_id in state["placed"]:
        piece_area = by_id[piece_id].get("area")
        if piece_area is None:
            piece_area = abs(polygon_area(by_id[piece_id]["polygon"]))
        placed_area += piece_area
    return _beam_rank(state, by_id, placed_area)


def _partial_target_possible(polygons, cfg):
    """Check whether a partial assembly can still fit a target rectangle."""
    slack = float(cfg.get("partial_bound_slack", 8.0))
    long_max = min(float(cfg["long_max"]),
                   float(cfg.get("hard_long_max", 1e9))) + slack
    short_max = min(float(cfg["short_max"]),
                    float(cfg.get("hard_short_max", 1e9))) + slack
    total_area = float(cfg.get("_total_area", 0.0))
    minimum_fill = max(0.01, float(cfg.get("min_fill", 0.60)))
    maximum_area = (total_area / minimum_fill if total_area > 0.0
                    else 1e30)
    for axis in _all_axis_angles(polygons):
        rect = _normalise_rect(polygons, _rect_at_angle(polygons, axis))
        rect_area = rect["width"] * rect["height"]
        if (rect["width"] <= long_max and rect["height"] <= short_max and
                rect_area <= maximum_area + slack * slack):
            return True
    return False


def _partial_frame_rank(polygons, placed_area, cfg):
    """Softly prefer states already resembling the required outer frame."""
    best = None
    corner_tol = float(cfg.get("corner_tol", cfg["outer_tol"]))
    for axis in _all_axis_angles(polygons):
        rect = _normalise_rect(polygons, _rect_at_angle(polygons, axis))
        rect_area = rect["width"] * rect["height"]
        excess = max(0.0, rect_area - placed_area)
        boundary, unused_sides, corners, outer_count = _outer_metrics(
            polygons, rect, cfg["outer_tol"], corner_tol)
        missing_outer = max(0, len(polygons) - outer_count)
        shape_penalty = (
            missing_outer * 1.0 +
            max(0.0, 1.0 - corners) * 0.75 +
            max(0.0, 1.0 - boundary) * 0.25)
        value = (excess * 0.0015 + shape_penalty *
                 float(cfg.get("frame_rank_weight", 0.35)))
        if best is None or value < best:
            best = value
    return best if best is not None else 1e9


def _axis_difference(angle, reference):
    quarter_turn = math.pi * 0.5
    difference = abs((angle - reference) % quarter_turn)
    return min(difference, quarter_turn - difference)


def _common_outer_axis_possible(polygons, used, cfg, required_axis=None):
    """Every piece must retain an edge on one common partial outer frame."""
    if not polygons:
        return False
    tolerance = math.radians(float(cfg.get("outer_axis_tol_deg", 16.0)))
    minimum = float(cfg.get("minimum_physical_edge", 12.0))
    piece_ids = list(polygons.keys())
    reference_id = piece_ids[0]
    references = []
    if required_axis is not None:
        references.append(required_axis % (math.pi * 0.5))
    else:
        reference_poly = polygons[reference_id]
        for edge_index in range(len(reference_poly)):
            if (reference_id, edge_index) in used:
                continue
            a, b = _edge(reference_poly, edge_index)
            if _dist(a, b) < minimum:
                continue
            angle = math.atan2(b[1] - a[1], b[0] - a[0])
            reference = angle % (math.pi * 0.5)
            duplicate = False
            for old_reference in references:
                if (_axis_difference(reference, old_reference) <=
                        math.radians(1.0)):
                    duplicate = True
                    break
            if not duplicate:
                references.append(reference)
    if not references:
        return False

    for reference in references:
        cosine = math.cos(reference)
        sine = math.sin(reference)
        minimum_x = 1e30
        maximum_x = -1e30
        minimum_y = 1e30
        maximum_y = -1e30
        for poly in polygons.values():
            for x, y in poly:
                local_x = cosine * x + sine * y
                local_y = -sine * x + cosine * y
                minimum_x = min(minimum_x, local_x)
                maximum_x = max(maximum_x, local_x)
                minimum_y = min(minimum_y, local_y)
                maximum_y = max(maximum_y, local_y)

        all_pieces_possible = True
        position_tolerance = float(cfg.get("outer_tol", 4.0))
        for piece_id in piece_ids:
            poly = polygons[piece_id]
            piece_possible = False
            for edge_index in range(len(poly)):
                if (piece_id, edge_index) in used:
                    continue
                a, b = _edge(poly, edge_index)
                if _dist(a, b) < minimum:
                    continue
                angle = math.atan2(b[1] - a[1], b[0] - a[0])
                if _axis_difference(angle, reference) > tolerance:
                    continue
                ax = cosine * a[0] + sine * a[1]
                ay = -sine * a[0] + cosine * a[1]
                bx = cosine * b[0] + sine * b[1]
                by = -sine * b[0] + cosine * b[1]
                if ((abs(ax - minimum_x) <= position_tolerance and
                     abs(bx - minimum_x) <= position_tolerance) or
                    (abs(ax - maximum_x) <= position_tolerance and
                     abs(bx - maximum_x) <= position_tolerance) or
                    (abs(ay - minimum_y) <= position_tolerance and
                     abs(by - minimum_y) <= position_tolerance) or
                    (abs(ay - maximum_y) <= position_tolerance and
                     abs(by - maximum_y) <= position_tolerance)):
                    piece_possible = True
                    break
            if not piece_possible:
                all_pieces_possible = False
                break
        if all_pieces_possible:
            return True
    return False


def _root_frame_axes(poly, minimum_edge):
    """Unique rectangle-axis hypotheses contributed by one outer piece."""
    axes = []
    duplicate_tolerance = math.radians(1.0)
    for edge_index in range(len(poly)):
        a, b = _edge(poly, edge_index)
        if _dist(a, b) < minimum_edge:
            continue
        axis = math.atan2(b[1] - a[1], b[0] - a[0]) % (math.pi * 0.5)
        duplicate = False
        for old_axis in axes:
            if _axis_difference(axis, old_axis) <= duplicate_tolerance:
                duplicate = True
                break
        if not duplicate:
            axes.append(axis)
    return axes


def _partial_anchor_fractions(new_len, old_len, by_id, cfg):
    """Return bounded slide events inferred from other measured edge lengths."""
    values = []

    def add(value):
        value = max(0.0, min(1.0, float(value)))
        for old_value in values:
            if abs(old_value - value) <= 0.008:
                return
        values.append(value)

    for value in cfg.get("partial_slide_fractions", (0.0, 1.0)):
        add(value)
    if not cfg.get("partial_event_fractions", False):
        values.sort()
        return values

    free_length = abs(old_len - new_len)
    if free_length <= 1e-6:
        values.sort()
        return values

    events = []
    for piece in by_id.values():
        for edge_length in piece["edge_lengths"]:
            if 0.0 < edge_length < free_length:
                fraction = edge_length / free_length
                events.append(fraction)
                events.append(1.0 - fraction)
    events.sort()
    limit = max(len(values), int(cfg.get("partial_event_limit", 11)))
    if len(events) > max(0, limit - len(values)):
        slots = max(0, limit - len(values))
        sampled = []
        if slots == 1:
            sampled = [events[len(events) // 2]]
        elif slots > 1:
            for index in range(slots):
                event_index = int(round(
                    index * (len(events) - 1) / float(slots - 1)))
                sampled.append(events[event_index])
        events = sampled
    for value in events:
        add(value)
    values.sort()
    return values


def _pattern_edge_cost(new_pattern, old_pattern, new_len, old_len, cfg):
    """Penalize incompatible ink continuity while keeping geometry primary."""
    if not new_pattern or not old_pattern:
        return 0.0
    if min(new_len, old_len) / max(new_len, old_len) < 0.86:
        return 0.0
    if len(new_pattern) != len(old_pattern):
        return 0.0
    informative = 0
    mismatched = 0
    for index, new_label in enumerate(new_pattern):
        old_label = old_pattern[len(old_pattern) - 1 - index]
        if new_label == 3 or old_label == 3:
            continue
        if new_label == 0 and old_label == 0:
            continue
        informative += 1
        if new_label != old_label:
            mismatched += 1
    minimum = int(cfg.get("pattern_min_informative", 3))
    if informative < minimum:
        return 0.0
    confidence = min(1.0, informative / float(max(minimum, 1) * 3))
    return (mismatched / float(informative) *
            float(cfg.get("pattern_weight", 2.5)) * confidence)


def _reverse_pattern_labels(pattern):
    """Reverse one compact edge-label sequence for a reversed traversal."""
    if pattern is None:
        return None
    try:
        count = len(pattern)
    except TypeError:
        return None
    output = bytearray(count)
    for index in range(count):
        output[index] = int(pattern[count - 1 - index])
    return output


def _normalize_edge_patterns(raw_polygon, raw_pattern):
    """Keep edge labels aligned after polygon orientation normalization."""
    if not raw_pattern:
        return raw_pattern
    count = len(raw_polygon)
    if len(raw_pattern) != count:
        return raw_pattern
    area = polygon_area(raw_polygon)
    if area >= 0.0:
        return raw_pattern

    normalized = []
    for new_index in range(count):
        old_index = (count - 2 - new_index) % count
        normalized.append(_reverse_pattern_labels(raw_pattern[old_index]))
    return normalized


def _prepare_edge_matches(by_id, ids, cfg):
    """Precompute edge compatibility, costs and local relative transforms."""
    table = {}
    allow_partial = cfg.get("allow_partial_edges", True)
    for new_id in ids:
        new_poly = by_id[new_id]["polygon"]
        new_lengths = by_id[new_id]["edge_lengths"]
        for old_id in ids:
            if new_id == old_id:
                continue
            old_poly = by_id[old_id]["polygon"]
            old_lengths = by_id[old_id]["edge_lengths"]
            records = []
            for new_edge in range(len(new_poly)):
                new_len = new_lengths[new_edge]
                if new_len < cfg["minimum_physical_edge"]:
                    continue
                for old_edge in range(len(old_poly)):
                    old_len = old_lengths[old_edge]
                    if old_len < cfg["minimum_physical_edge"]:
                        continue
                    maximum = max(new_len, old_len)
                    tolerance = max(
                        cfg["edge_abs_tol"], cfg["edge_rel_tol"] * maximum)
                    length_error = abs(new_len - old_len)
                    exact = length_error <= tolerance
                    ratio = min(new_len, old_len) / maximum
                    partial = (
                        allow_partial and
                        ratio >= cfg["partial_edge_min_ratio"] and
                        length_error >= float(
                            cfg.get("partial_remainder_min", 10.0)))
                    if not exact and not partial:
                        continue
                    if exact:
                        # Centre alignment splits measurement error between
                        # both seam endpoints instead of accumulating it.
                        pattern_cost = _pattern_edge_cost(
                            by_id[new_id].get("edge_pattern", ())[
                                new_edge] if by_id[new_id].get(
                                    "edge_pattern") and new_edge < len(
                                        by_id[new_id]["edge_pattern"]) else None,
                            by_id[old_id].get("edge_pattern", ())[
                                old_edge] if by_id[old_id].get(
                                    "edge_pattern") and old_edge < len(
                                        by_id[old_id]["edge_pattern"]) else None,
                            new_len, old_len, cfg)
                        records.append((
                            new_edge, old_edge, True,
                            _align_reversed(
                                new_poly, new_edge, old_poly, old_edge,
                                anchor_fraction=0.5),
                            length_error / max(maximum, 1.0) * 8.0 +
                            pattern_cost,
                            new_len < old_len,
                        ))
                    if partial:
                        anchor_fractions = _partial_anchor_fractions(
                            new_len, old_len, by_id, cfg)
                        for anchor_fraction in anchor_fractions:
                            records.append((
                                new_edge, old_edge, False,
                                _align_reversed(
                                    new_poly, new_edge, old_poly, old_edge,
                                    anchor_fraction=anchor_fraction),
                                3.0,
                                new_len < old_len,
                            ))
            exact_records = []
            partial_records = []
            for record in records:
                compact = (record[0], record[1], record[3],
                           record[4], record[5])
                if record[2]:
                    exact_records.append(compact)
                else:
                    partial_records.append(compact)
            table[(new_id, old_id, True)] = exact_records
            table[(new_id, old_id, False)] = partial_records
    return table


def _build_result(best, state, by_id, cfg, valid=True):
    (score, rect, polygons, fill, boundary_coverage,
     corner_coverage, minimum_side_coverage,
     outer_piece_count, seam_coverage) = best
    c = math.cos(rect["angle"])
    s = math.sin(rect["angle"])
    target_cx, target_cy = cfg["target_center"]
    target_vertical = bool(cfg.get("target_vertical", False))
    if target_vertical:
        target_width = rect["height"]
        target_height = rect["width"]
    else:
        target_width = rect["width"]
        target_height = rect["height"]
    offset_x = target_cx - target_width * 0.5
    offset_y = target_cy - target_height * 0.5
    placements = []

    def target_point(world_point):
        local_x = (c * world_point[0] + s * world_point[1] -
                   rect["min_x"])
        local_y = (-s * world_point[0] + c * world_point[1] -
                   rect["min_y"])
        if target_vertical:
            # A 90-degree rotation only; determinant stays positive, so this
            # never mirrors or flips a physical piece.
            return (local_y + offset_x,
                    rect["width"] - local_x + offset_y)
        return (local_x + offset_x, local_y + offset_y)

    for piece_id in state["placed"]:
        piece = by_id[piece_id]
        world_pick = _apply(state["placed"][piece_id], piece["pick"])
        target = target_point(world_pick)
        rotation = math.degrees(
            _transform_angle(state["placed"][piece_id]) - rect["angle"])
        if target_vertical:
            rotation -= 90.0
        rotation = (rotation + 180.0) % 360.0 - 180.0

        target_polygon = []
        for world_point in polygons[piece_id]:
            target_polygon.append(target_point(world_point))

        placements.append({
            "id": piece_id,
            "pick": piece["pick"],
            "target": target,
            "angle": rotation,
            "polygon": target_polygon,
        })

    placements.sort(key=lambda item: (item["target"][0], item["target"][1]))
    for slot, placement in enumerate(placements):
        placement["slot"] = slot

    return {
        "score": score,
        "valid": valid,
        "edge_score": state["score"],
        "width": rect["width"],
        "height": rect["height"],
        "aspect_ratio": (rect["width"] / rect["height"]
                         if rect["height"] > 1e-9 else 1e9),
        "target_dimension_error": (
            (abs(rect["width"] - float(cfg.get("target_long", 0.0)))
             if float(cfg.get("target_long", 0.0)) > 0.0 else 0.0) +
            (abs(rect["height"] - float(cfg.get("target_short", 0.0)))
             if float(cfg.get("target_short", 0.0)) > 0.0 else 0.0)),
        "target_width": target_width,
        "target_height": target_height,
        "target_vertical": target_vertical,
        "fill": fill,
        "boundary_coverage": boundary_coverage,
        "minimum_side_coverage": minimum_side_coverage,
        "corner_coverage": corner_coverage,
        "outer_piece_count": outer_piece_count,
        "seam_coverage": seam_coverage,
        "placements": placements,
    }


def _solve_low_memory(by_id, ids, cfg, total_area):
    """Depth-first four-piece search with bounded live state for MicroPython."""
    node_count = [0]
    best_score = [1e30]
    best_result = [None]
    fallback_score = [1e30]
    fallback_result = [None]
    max_nodes = int(cfg.get("low_memory_max_nodes", 16000))
    branch_width = int(cfg.get("low_memory_branch_width", 0))
    cache_geometry = bool(cfg.get("cache_geometry", branch_width > 0))
    stop_score = cfg.get("low_memory_stop_score")
    time_limit_ms = int(cfg.get("time_limit_ms", 0))
    search_start_ms = _ticks_ms()
    edge_matches = _prepare_edge_matches(by_id, ids, cfg)
    dedup_limit = int(cfg.get("state_dedup_limit", 5000))
    seen_by_depth = [{} for _ in range(len(ids) + 1)]
    stats = {
        "generated": 0,
        "overlap_pruned": 0,
        "bounds_pruned": 0,
        "outer_axis_pruned": 0,
        "duplicate_pruned": 0,
        "score_pruned": 0,
        "complete": 0,
        "sat_checks": 0,
        "general_overlap_checks": 0,
        "frame_hypotheses": 0,
    }

    def deadline_reached():
        return (time_limit_ms > 0 and
                _ticks_diff(_ticks_ms(), search_start_ms) >= time_limit_ms)

    def visit(state):
        if deadline_reached():
            return True
        if node_count[0] >= max_nodes:
            return True
        # The final score only adds a non-negative fill penalty.
        if state["score"] >= best_score[0]:
            stats["score_pruned"] += 1
            return False
        depth = len(state["placed"])
        if dedup_limit > 0:
            key = _state_key(state, cfg)
            seen = seen_by_depth[depth]
            previous_score = seen.get(key)
            if previous_score is not None and previous_score <= state["score"]:
                stats["duplicate_pruned"] += 1
                return False
            if previous_score is not None or len(seen) < dedup_limit:
                seen[key] = state["score"]
        node_count[0] += 1
        if len(state["placed"]) >= len(ids):
            stats["complete"] += 1
            best, fallback = _evaluate_complete(
                state, by_id, cfg, total_area)
            if fallback and fallback[0] < fallback_score[0]:
                fallback_score[0] = fallback[0]
                fallback_result[0] = _build_result(
                    fallback, state, by_id, cfg, valid=False)
            if best and best[0] < best_score[0]:
                best_score[0] = best[0]
                best_result[0] = _build_result(
                    best, state, by_id, cfg, valid=True)
                # Pair cost and fill penalty are non-negative, so zero is
                # already the global optimum and no later branch can improve it.
                if best_score[0] <= 1e-9:
                    return True
                if (stop_score is not None and
                        best_score[0] <= float(stop_score)):
                    return True
            return False

        polygons = _state_polygons(state, by_id)
        bboxes = state.get("bboxes")
        if bboxes is None:
            bboxes = {
                piece_id: _polygon_bbox(poly)
                for piece_id, poly in polygons.items()
            }
        sat_axes = state.get("sat_axes")
        if sat_axes is None:
            sat_axes = {
                piece_id: _transform_axes(
                    state["placed"][piece_id],
                    by_id[piece_id]["sat_axes"])
                for piece_id in state["placed"]
            }
        children = []

        def keep_child(child):
            stats["generated"] += 1
            if branch_width <= 0 or len(children) < branch_width:
                children.append(child)
                return
            worst_index = 0
            worst_rank = children[0]["_rank"]
            for index in range(1, len(children)):
                if children[index]["_rank"] > worst_rank:
                    worst_index = index
                    worst_rank = children[index]["_rank"]
            if child["_rank"] < worst_rank:
                children[worst_index] = child

        # Try exact edge pairs before partial T-junction pairs.
        match_modes = ((True, False) if cfg.get("allow_partial_edges", True)
                       else (True,))
        for require_exact in match_modes:
            # MRV ordering: constrained pieces are attempted first, while all
            # attachable pieces remain in the search to preserve completeness.
            piece_options = []
            for new_id in ids:
                if new_id in state["placed"]:
                    continue
                option_count = 0
                for old_id in state["placed"]:
                    for record in edge_matches.get(
                            (new_id, old_id, require_exact), ()):
                        if (old_id, record[1]) not in state["used"]:
                            option_count += 1
                if option_count:
                    piece_options.append((option_count, new_id))
            piece_options.sort(key=lambda item: (item[0], item[1]))

            for option_count, new_id in piece_options:
                if deadline_reached():
                    return True
                new_poly = by_id[new_id]["polygon"]
                for old_id in state["placed"]:
                    old_transform = state["placed"][old_id]
                    for record in edge_matches.get(
                            (new_id, old_id, require_exact), ()):
                        new_edge, old_edge = record[:2]
                        exact_pair = require_exact
                        if (old_id, old_edge) in state["used"]:
                            continue
                        transform = _compose_transform(
                            old_transform, record[2])
                        candidate = _apply_poly(transform, new_poly)
                        candidate_bbox = _polygon_bbox(candidate)
                        candidate_axes = _transform_axes(
                            transform, by_id[new_id]["sat_axes"])
                        overlaps = False
                        for placed_id in polygons:
                            placed_bbox = bboxes[placed_id]
                            if _bboxes_separated(
                                    candidate_bbox, placed_bbox,
                                    cfg["intersection_tol"]):
                                continue
                            if (by_id[new_id]["convex"] and
                                    by_id[placed_id]["convex"]):
                                stats["sat_checks"] += 1
                                is_overlap = _convex_polygons_overlap(
                                    candidate, polygons[placed_id],
                                    cfg["intersection_tol"],
                                    candidate_axes, sat_axes[placed_id])
                            else:
                                stats["general_overlap_checks"] += 1
                                is_overlap = _polygons_overlap(
                                    candidate, polygons[placed_id],
                                    cfg["intersection_tol"],
                                    candidate_bbox, placed_bbox)
                            if is_overlap:
                                overlaps = True
                                break
                        if overlaps:
                            stats["overlap_pruned"] += 1
                            continue

                        child_polygons = None
                        if (cache_geometry or branch_width > 0 or
                                cfg.get("prune_target_bounds", False) or
                                cfg.get("prune_outer_axis", True)):
                            child_polygons = dict(polygons)
                            child_polygons[new_id] = candidate
                        child_bboxes = None
                        if cache_geometry:
                            child_bboxes = dict(bboxes)
                            child_bboxes[new_id] = candidate_bbox
                        child_sat_axes = dict(sat_axes)
                        child_sat_axes[new_id] = candidate_axes
                        if (cfg.get("prune_target_bounds", False) and
                                len(state["placed"]) + 1 < len(ids) and
                                not _partial_target_possible(
                                    child_polygons, cfg)):
                            stats["bounds_pruned"] += 1
                            continue

                        placed = dict(state["placed"])
                        placed[new_id] = transform
                        used = set(state["used"])
                        if exact_pair:
                            used.add((new_id, new_edge))
                            used.add((old_id, old_edge))
                        elif record[4]:
                            used.add((new_id, new_edge))
                        else:
                            used.add((old_id, old_edge))
                        if (cfg.get("prune_outer_axis", True) and
                                not _common_outer_axis_possible(
                                    child_polygons, used, cfg,
                                    state.get("frame_axis"))):
                            stats["outer_axis_pruned"] += 1
                            continue
                        score = state["score"] + record[3]
                        base_placed_area = state.get("placed_area")
                        if base_placed_area is None:
                            base_placed_area = sum(
                                by_id[piece_id]["area"]
                                for piece_id in state["placed"])
                        placed_area = base_placed_area + by_id[new_id]["area"]
                        frame_rank = 0.0
                        if branch_width > 0:
                            frame_rank = _partial_frame_rank(
                                child_polygons, placed_area, cfg)
                        child = {
                            "placed": placed,
                            "used": used,
                            "score": score,
                            "bboxes": (child_bboxes if child_bboxes is not None
                                       else dict(bboxes)),
                            "sat_axes": child_sat_axes,
                            "frame_axis": state.get("frame_axis"),
                            "_rank": (score + frame_rank +
                                      option_count * 0.0001),
                        }
                        if child_bboxes is None:
                            child["bboxes"][new_id] = candidate_bbox
                        if cache_geometry:
                            child["polygons"] = child_polygons
                            child["placed_area"] = placed_area
                        keep_child(child)
        children.sort(key=lambda child: child["_rank"])
        for child in children:
            if visit(child):
                return True
        return False

    root_polygon = by_id[ids[0]]["polygon"]
    if cfg.get("frame_hypothesis_search", False):
        frame_axes = _root_frame_axes(
            root_polygon, float(cfg["minimum_physical_edge"]))
    else:
        frame_axes = [None]
    stats["frame_hypotheses"] = len(frame_axes)
    for frame_axis in frame_axes:
        root_state = {
            "placed": {ids[0]: _identity()},
            "used": set(),
            "score": 0.0,
            "bboxes": {ids[0]: _polygon_bbox(root_polygon)},
            "sat_axes": {ids[0]: by_id[ids[0]]["sat_axes"]},
            "frame_axis": frame_axis,
        }
        if cache_geometry:
            root_state["polygons"] = {ids[0]: root_polygon}
            root_state["placed_area"] = by_id[ids[0]]["area"]
        if visit(root_state):
            break
    result = best_result[0] if best_result[0] is not None else fallback_result[0]
    if result is not None:
        result["search_nodes"] = node_count[0]
        result["search_ms"] = _ticks_diff(_ticks_ms(), search_start_ms)
        result["search_stats"] = stats
    return result


def solve_puzzle(input_pieces, user_config=None):
    """Solve a 1..4-piece rectangular puzzle, returning canonical placements."""
    cfg = _config(user_config)
    if not input_pieces or len(input_pieces) > 4:
        return None

    pieces = []
    used_ids = set()
    for index, original in enumerate(input_pieces):
        piece = dict(original)
        piece_id = int(piece.get("id", index))
        if piece_id in used_ids:
            return None
        used_ids.add(piece_id)
        raw_polygon = piece.get("polygon", [])
        try:
            raw_polygon = [(float(point[0]), float(point[1]))
                           for point in raw_polygon]
        except (TypeError, IndexError, ValueError):
            return None
        poly = ensure_ccw(raw_polygon)
        if cfg.get("convex_pieces_only", False):
            poly = ensure_ccw(_convex_hull(poly))
        if len(poly) < 3 or len(poly) > 5:
            return None
        area = abs(polygon_area(poly))
        if area < 80.0:
            return None
        piece["id"] = piece_id
        piece["polygon"] = poly
        piece["area"] = area
        piece["convex"] = _polygon_is_convex(poly)
        piece["sat_axes"] = _polygon_sat_axes(poly)
        piece["edge_lengths"] = [
            _edge_length(poly, edge) for edge in range(len(poly))]
        if cfg.get("convex_pieces_only", False):
            # Hull edges may bridge several printed-edge fragments, so old
            # edge labels no longer correspond to the normalized polygon.
            piece["edge_pattern"] = None
        else:
            piece["edge_pattern"] = _normalize_edge_patterns(
                raw_polygon, piece.get("edge_pattern"))
        piece.setdefault("pick", polygon_centroid(poly))
        pieces.append(piece)

    pieces.sort(key=lambda item: item["id"])
    by_id = {piece["id"]: piece for piece in pieces}
    ids = [piece["id"] for piece in pieces]
    total_area = sum(piece["area"] for piece in pieces)
    cfg["_total_area"] = total_area
    if len(pieces) == 1:
        state = {"placed": {ids[0]: _identity()}, "used": set(), "score": 0.0}
        best, fallback = _evaluate_complete(
            state, by_id, cfg, total_area)
        if best:
            return _build_result(best, state, by_id, cfg, valid=True)
        return (_build_result(fallback, state, by_id, cfg, valid=False)
                if fallback else None)

    if cfg.get("low_memory"):
        search_ids = _ordered_search_ids(by_id, ids, cfg)
        return _solve_low_memory(by_id, search_ids, cfg, total_area)

    states = [{
        "placed": {ids[0]: _identity()},
        "used": set(),
        "score": 0.0,
    }]

    while states and len(states[0]["placed"]) < len(pieces):
        expanded = []
        for state in states:
            polygons = _state_polygons(state, by_id)
            unplaced = [piece_id for piece_id in ids
                        if piece_id not in state["placed"]]
            for new_id in unplaced:
                new_piece = by_id[new_id]
                new_poly = new_piece["polygon"]
                new_lengths = new_piece["edge_lengths"]
                for new_edge in range(len(new_poly)):
                    new_len = new_lengths[new_edge]
                    if new_len < cfg["minimum_physical_edge"]:
                        continue
                    for old_id in list(state["placed"].keys()):
                        old_poly = polygons[old_id]
                        old_lengths = by_id[old_id]["edge_lengths"]
                        for old_edge in range(len(old_poly)):
                            if (old_id, old_edge) in state["used"]:
                                continue
                            old_len = old_lengths[old_edge]
                            if old_len < cfg["minimum_physical_edge"]:
                                continue
                            tolerance = max(
                                cfg["edge_abs_tol"],
                                cfg["edge_rel_tol"] * max(new_len, old_len))
                            length_error = abs(new_len - old_len)
                            exact_pair = length_error <= tolerance
                            length_ratio = min(new_len, old_len) / max(new_len, old_len)
                            partial_pair = (
                                cfg.get("allow_partial_edges", True) and
                                length_ratio >= cfg["partial_edge_min_ratio"] and
                                length_error >= float(
                                    cfg.get("partial_remainder_min", 10.0)))
                            if not exact_pair and not partial_pair:
                                continue

                            match_options = []
                            if exact_pair:
                                match_options.append((True, 0.5))
                            if partial_pair:
                                for anchor_fraction in _partial_anchor_fractions(
                                        new_len, old_len, by_id, cfg):
                                    match_options.append(
                                        (False, anchor_fraction))
                            for match_exact, anchor_fraction in match_options:
                                transform = _align_reversed(
                                    new_poly, new_edge, old_poly, old_edge,
                                    anchor_fraction=anchor_fraction)
                                candidate = _apply_poly(transform, new_poly)
                                invalid = False
                                for placed_id in polygons:
                                    if _polygons_overlap(
                                            candidate, polygons[placed_id],
                                            cfg["intersection_tol"]):
                                        invalid = True
                                        break
                                if invalid:
                                    continue

                                # Partial matches are legitimate at a T junction.
                                # Give them a small complexity cost, while the final
                                # rectangle/fill checks decide whether they are real.
                                pair_cost = (
                                    length_error / max(new_len, old_len, 1.0) * 8.0
                                    if match_exact else 3.0)
                                if match_exact:
                                    new_patterns = by_id[new_id].get(
                                        "edge_pattern")
                                    old_patterns = by_id[old_id].get(
                                        "edge_pattern")
                                    pair_cost += _pattern_edge_cost(
                                        new_patterns[new_edge]
                                        if new_patterns and new_edge < len(
                                            new_patterns) else None,
                                        old_patterns[old_edge]
                                        if old_patterns and old_edge < len(
                                            old_patterns) else None,
                                        new_len, old_len, cfg)
                                score = state["score"] + pair_cost
                                placed = dict(state["placed"])
                                placed[new_id] = transform
                                used = set(state["used"])
                                if match_exact:
                                    used.add((new_id, new_edge))
                                    used.add((old_id, old_edge))
                                elif new_len < old_len:
                                    used.add((new_id, new_edge))
                                else:
                                    used.add((old_id, old_edge))
                                expanded.append({
                                    "placed": placed,
                                    "used": used,
                                    "score": score,
                                })

        if not expanded:
            return None

        dedup = {}
        for state in expanded:
            key = _state_key(state)
            old = dedup.get(key)
            if old is None or state["score"] < old["score"]:
                dedup[key] = state
        states = list(dedup.values())
        placed_area = sum(by_id[piece_id]["area"]
                          for piece_id in states[0]["placed"])
        states.sort(key=lambda state: _beam_rank(state, by_id, placed_area))
        states = states[:int(cfg["beam_width"])]

    finalists = []
    fallback_finalists = []
    for state in states:
        best, fallback = _evaluate_complete(
            state, by_id, cfg, total_area)
        if best:
            finalists.append((best[0], best, state))
        if fallback:
            fallback_finalists.append((fallback[0], fallback, state))
    if not finalists:
        if not fallback_finalists:
            return None
        fallback_finalists.sort(key=lambda item: item[0])
        _, best, state = fallback_finalists[0]
        return _build_result(best, state, by_id, cfg, valid=False)
    finalists.sort(key=lambda item: item[0])
    _, best, state = finalists[0]
    return _build_result(best, state, by_id, cfg, valid=True)
