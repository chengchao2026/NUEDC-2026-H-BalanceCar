"""Hardware-independent geometry and filtering for steel-ball tracking."""

import math

def project_onto_axis(point, axis_start, axis_end):
    """Return normalized position and perpendicular distance to an image axis."""
    px, py = point
    x0, y0 = axis_start
    x1, y1 = axis_end
    dx = x1 - x0
    dy = y1 - y0
    length_squared = dx * dx + dy * dy
    if length_squared <= 0:
        raise ValueError("calibration endpoints must be different")

    relative_x = px - x0
    relative_y = py - y0
    ratio = (relative_x * dx + relative_y * dy) / float(length_squared)
    projected_x = x0 + ratio * dx
    projected_y = y0 + ratio * dy
    distance = math.hypot(px - projected_x, py - projected_y)
    return ratio, distance


def axis_point(ratio, axis_start, axis_end):
    """Return the image-space point at *ratio* along an axis."""
    x0, y0 = axis_start
    x1, y1 = axis_end
    return (
        x0 + ratio * (x1 - x0),
        y0 + ratio * (y1 - y0),
    )


def axis_velocity(vx, vy, axis_start, axis_end, start_position_cm, end_position_cm):
    """Project a pixel-space velocity (px/s) onto the axis and convert to cm/s."""
    x0, y0 = axis_start
    x1, y1 = axis_end
    dx = x1 - x0
    dy = y1 - y0
    length_squared = dx * dx + dy * dy
    if length_squared <= 0:
        raise ValueError("calibration endpoints must be different")
    ratio_rate = (dx * vx + dy * vy) / float(length_squared)
    return ratio_rate * (end_position_cm - start_position_cm)


def position_from_pixel(
    point,
    axis_start,
    axis_end,
    start_position_cm,
    end_position_cm,
):
    """Project an image point onto the beam and convert it to centimeters."""
    if start_position_cm == end_position_cm:
        raise ValueError("physical calibration positions must be different")
    ratio, distance = project_onto_axis(point, axis_start, axis_end)
    position_cm = start_position_cm + ratio * (
        end_position_cm - start_position_cm
    )
    return position_cm, ratio, distance


class AdaptiveAlphaBetaFilter:
    """Velocity-aware position filter with adaptive measurement gain."""

    def __init__(
        self,
        alpha_slow=0.20,
        alpha_fast=0.95,
        beta_slow=0.01,
        beta_fast=0.12,
        fast_error_px=10.0,
        reset_ms=250,
        gate_base_px=40.0,
        gate_growth_px=25.0,
        gate_max_misses=6,
    ):
        self.alpha_slow = alpha_slow
        self.alpha_fast = alpha_fast
        self.beta_slow = beta_slow
        self.beta_fast = beta_fast
        self.fast_error_px = fast_error_px
        self.reset_ms = reset_ms
        # Temporal plausibility gate: a fresh detection too far from where the
        # track predicts it should be is rejected as a likely false positive
        # (background clutter, chassis, reflections) rather than accepted and
        # allowed to yank the filter/control loop. The gate widens after
        # consecutive misses so a real re-acquisition after a gap isn't
        # permanently blocked.
        self.gate_base_px = gate_base_px
        self.gate_growth_px = gate_growth_px
        self.gate_max_misses = gate_max_misses
        self.missed_count = 0
        self.reset()

    def reset(self):
        self.x = None
        self.y = None
        self.vx = 0.0
        self.vy = 0.0
        self.last_ms = None
        self.missed_count = 0

    def update(self, measured_x, measured_y, now_ms):
        if self.x is None:
            self.x = float(measured_x)
            self.y = float(measured_y)
            self.last_ms = now_ms
            return self.x, self.y

        dt = (now_ms - self.last_ms) * 0.001
        dt = max(0.005, min(0.100, dt))

        predicted_x = self.x + self.vx * dt
        predicted_y = self.y + self.vy * dt
        error_x = measured_x - predicted_x
        error_y = measured_y - predicted_y
        error = math.hypot(error_x, error_y)

        motion = min(1.0, error / self.fast_error_px) ** 2
        alpha = self.alpha_slow + (self.alpha_fast - self.alpha_slow) * motion
        beta = self.beta_slow + (self.beta_fast - self.beta_slow) * motion

        self.x = predicted_x + alpha * error_x
        self.y = predicted_y + alpha * error_y
        self.vx += beta * error_x / dt
        self.vy += beta * error_y / dt
        self.last_ms = now_ms
        self.missed_count = 0
        return self.x, self.y

    def mark_missing(self, now_ms):
        self.missed_count += 1
        if self.last_ms is not None and now_ms - self.last_ms >= self.reset_ms:
            self.reset()

    def is_plausible(self, measured_x, measured_y, now_ms):
        """Reject a detection that jumped further than the track predicts.

        Returns True when there is no established track yet (nothing to
        gate against) or when the detection is within the allowed radius of
        the predicted position. The allowed radius grows with consecutive
        misses so a genuine re-acquisition after a real gap is not blocked
        forever.
        """
        predicted = self.predict(now_ms)
        if predicted is None:
            return True
        px, py = predicted
        gate = self.gate_base_px + self.gate_growth_px * min(
            self.missed_count, self.gate_max_misses
        )
        return math.hypot(measured_x - px, measured_y - py) <= gate

    def predict(self, now_ms):
        """Extrapolate position from the last known state without updating it.

        Returns None if there is no established track yet. Callers should
        only trust this for a short hold window right after detection is
        lost -- it does not know about track boundaries or collisions.
        """
        if self.x is None or self.last_ms is None:
            return None
        dt = (now_ms - self.last_ms) * 0.001
        dt = max(0.0, min(0.100, dt))
        return self.x + self.vx * dt, self.y + self.vy * dt


def validate_calibration(
    axis_start, axis_end, start_position_cm, end_position_cm, width, height
):
    """Validate calibration against the detector image dimensions."""
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    for name, point in (("start", axis_start), ("end", axis_end)):
        if not (0 <= point[0] < width and 0 <= point[1] < height):
            raise ValueError("{} calibration point is outside the image".format(name))
    position_from_pixel(
        axis_start,
        axis_start,
        axis_end,
        start_position_cm,
        end_position_cm,
    )
