"""
core/slam/scan.py

**What a scan IS.** One place, because chunks 21, 22 and 23 must all agree.

A scan reaches SLAM as `Observation.ranges` — a float array parallel to
`Observation.range_angles`, in the robot frame, with `inf` where the beam hit
nothing inside `range_max`. Everything downstream wants it as a **point cloud**
instead, and the conversion has exactly two decisions in it. Both are made here
so that no later module gets to make them differently.

TWO DECISIONS
-------------
**`inf` beams are DROPPED, not clamped to `range_max`.** A beam that returned
nothing is the absence of a measurement, not a measurement of empty space at the
sensor limit, and a matcher fed max-range points would score a wall where there
is none. This is the same distinction `core/geometry.py` preserves by returning
`inf` rather than `range_max` in the first place.

⚠️ Note what chunk 20 does to this. Dropout and spurious max-range returns both
produce `inf` on beams that *did* hit something, so under sensor noise a scan
silently loses real points. That is realistic — it is what a real lidar does —
and it is why the matcher must tolerate missing data rather than assume a fixed
point count.

**The cloud carries no pose.** Points are in the robot frame, and placing them
in the world is the *caller's* problem, because that placement is precisely what
the matcher is trying to solve for. A scan that arrived pre-transformed by the
believed pose would bake the answer into the question.

SIM-FREE.
"""

import numpy as np

# ---- below this line is code, not data.


def endpoints(ranges, angles):
    """Scan -> (N, 2) points in the ROBOT frame, `inf` beams dropped.

    `N` is therefore not `len(angles)` and varies between steps under sensor
    noise. Callers must not index the result against the beam array.
    """
    ranges = np.asarray(ranges, dtype=float)
    angles = np.asarray(angles, dtype=float)
    if ranges.shape != angles.shape:
        raise ValueError(
            f"ranges {ranges.shape} and angles {angles.shape} must be parallel; "
            f"they are the two halves of one scan")

    keep = np.isfinite(ranges)
    r = ranges[keep]
    a = angles[keep]
    return np.stack([r * np.cos(a), r * np.sin(a)], axis=-1)


def transform(points, pose):
    """Rigid-transform robot-frame points into the frame `pose` is expressed in.

    `pose` is `(x, y, theta)`. Rotation first, then translation — the usual
    convention, stated because getting it backwards produces a cloud that looks
    plausible and matches to the wrong place.
    """
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        return points.reshape(0, 2)
    x, y, theta = float(pose[0]), float(pose[1]), float(pose[2])
    c, s = np.cos(theta), np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([x, y])


def compose(a, b):
    """Pose composition: the pose of `b` expressed in `a`'s parent frame.

    Used to chain a match onto a previous estimate. Yaw is wrapped to (-pi, pi]
    HERE rather than at each call site, because an unwrapped yaw accumulating
    over a few thousand steps is one of the classic ways a pose graph ends up
    spun once round — chunk 23's own warning, applied one layer earlier.
    """
    ax, ay, at = float(a[0]), float(a[1]), float(a[2])
    bx, by, bt = float(b[0]), float(b[1]), float(b[2])
    c, s = np.cos(at), np.sin(at)
    return (ax + c * bx - s * by,
            ay + s * bx + c * by,
            wrap_angle(at + bt))


def relative(a, b):
    """The pose of `b` relative to `a`. The inverse of `compose`."""
    ax, ay, at = float(a[0]), float(a[1]), float(a[2])
    bx, by, bt = float(b[0]), float(b[1]), float(b[2])
    dx, dy = bx - ax, by - ay
    c, s = np.cos(-at), np.sin(-at)
    return (c * dx - s * dy,
            s * dx + c * dy,
            wrap_angle(bt - at))


def wrap_angle(theta):
    """To (-pi, pi]. One implementation, because chunk 23's residuals depend on
    the wrap being the same everywhere it is applied."""
    return float((theta + np.pi) % (2 * np.pi) - np.pi)
