"""
recorder.py

Offline frame capture for the Isaac Sim scripts.

The interactive viewport renders far slower than real time on this machine, so
watching a run live is painful. This renders through dedicated cameras into an
ffmpeg pipe instead: the sim runs headless as fast as it can, and you watch the
result afterwards at true speed.

Each frame is two panes side by side in one file: a fixed top-down shot of the
whole floorplan on the left, the chase camera on the right. The top-down pane
shows where in the building the robot is, the chase pane whether the gait is
stable. Pass top_down=False for the single-pane behaviour.

A third pane can sit ABOVE those two: a caller-supplied strip of pixels, set
with set_banner() before each capture. The recorder does not know or care what
is in it — `run_episode.py` fills it with the behaviour tree from
`nodes/diagram.py`, and keeping the recorder ignorant is deliberate. This file
imports omni; `nodes/` may not. A recorder that knew what a behaviour tree was
would drag the simulator into the one package the import guard keeps clean.

Import this only after SimulationApp has been constructed, since it pulls in
omni modules at import time.

    rec = Recorder("walk.mp4", resolution=(1280, 720), fps=30)
    # writes <repo>/out/walk_20260822-141530.mp4, see resolve_out_path()
    ...
    rec.look_from(eye, target)   # before the render
    world.step(render=True)
    rec.capture()                # after the render
    ...
    rec.close()
"""

import datetime
import math
import os
import shutil
import subprocess

import numpy as np
import omni.replicator.core as rep
from pxr import Gf, Usd, UsdGeom
import omni.usd

# Frames are only rendered when one is actually wanted, so the physics loop is
# not paying for a render it will throw away.
DEFAULT_RESOLUTION = (1280, 720)
DEFAULT_FPS = 30
# USD cameras default to 50mm, which is telephoto enough that the robot fills
# the frame and you cannot see where it is in the room. lab42 is too small to
# fix that by backing the camera up, so widen the lens instead.
DEFAULT_FOCAL_LENGTH = 24.0
# ffmpeg infers the container from the output extension, so a recording path
# without one has to be given a default rather than passed through.
DEFAULT_EXT = ".mp4"
# Recordings belong in the repo's out/, not in whichever directory the script
# happened to be launched from. Derived from this file's location (scripts/ is
# one level under the root) so it survives the repo being moved or renamed.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUT_DIR = os.path.join(REPO_ROOT, "out")

# --- top-down pane ---------------------------------------------------------
# The chase camera shows the gait but not where the robot is in the building.
# A fixed overhead shot of the whole floorplan is composited into the left half
# of each frame, with the chase view on the right.
#
# A longer lens than the chase camera's: there is nothing to back away from up
# here, and 24mm at this height keystones the walls badly enough to misread
# where the robot is relative to them.
DEFAULT_TOP_FOCAL_LENGTH = 35.0
# Multiplier on the fitted height. 1.0 shows the whole floorplan; raise it to
# crop into the middle, which is worth doing when the run stays in one half of
# the building.
DEFAULT_TOP_ZOOM = 1.0
# Breathing room around the floorplan so the perimeter walls are not flush
# against the frame edge. Small, because with both apertures authored the fit
# is exact rather than approximate.
TOP_MARGIN = 1.02
# USD's default, used when the attribute is somehow unauthored. Every aperture
# figure here is in mm, matching focal length.
DEFAULT_HORIZONTAL_APERTURE = 20.955
# Walls bound the interesting part of the scene. The ground plane is hundreds
# of metres across, so framing on that would show a dot in a field of grey.
SCENE_BOUNDS_PRIM = "/World/Walls"
# lab42, from build_scene.py WALLS: x in [0, 12], y in [0, 8]. Used when the
# stage has no walls to measure, or when what it measured is not a room.
FALLBACK_SCENE_BOUNDS = (0.0, 0.0, 12.0, 8.0)
# Above this span, the measurement has caught the ground plane or an
# environment reference rather than the floorplan.
MAX_SCENE_SPAN = 60.0
# Seam between the two panes, dark enough to read as a border on both.
DIVIDER_WIDTH = 2
DIVIDER_COLOR = (24, 24, 28)

# --- banner pane -----------------------------------------------------------
# Height in pixels of the optional strip above the two rendered panes, 0 for
# none. Its WIDTH is not a choice: it spans the composite, because a banner
# narrower than the frame would need padding and a wider one would resize the
# video. The caller asks for banner_size() and hands back exactly that.
DEFAULT_BANNER_HEIGHT = 0


def resolve_out_path(path, out_dir=DEFAULT_OUT_DIR):
    """Anchor a recording path under the repo's out/ directory.

        1               ->  <repo>/out/1.mp4
        walk.mp4        ->  <repo>/out/walk.mp4
        out/walk.mp4    ->  <repo>/out/walk.mp4
        runs/a/walk.mp4 ->  <repo>/out/runs/a/walk.mp4
        /tmp/walk.mp4   ->  /tmp/walk.mp4

    Relative paths land in out/ regardless of the working directory, which is
    the whole point: the same command run from the repo root and from scripts/
    should put the video in the same place. Subdirectories are kept, and a
    leading out/ means the out directory rather than one nested inside it.

    An absolute path is honoured as given, so writing somewhere else stays
    possible without a flag for it.
    """
    if os.path.isabs(path):
        resolved = path
    else:
        parts = [p for p in os.path.normpath(path).split(os.sep) if p != "."]
        if parts and parts[0] == os.path.basename(out_dir):
            parts = parts[1:]
        resolved = os.path.join(out_dir, *parts) if parts else out_dir

    resolved = os.path.normpath(resolved)
    if not os.path.splitext(resolved)[1]:
        resolved += DEFAULT_EXT
    return resolved


def timestamped_path(path):
    """Insert a timestamp before the extension so runs do not overwrite.

        walk.mp4  ->  <repo>/out/walk_20260822-141530.mp4
        1         ->  <repo>/out/1_20260822-141530.mp4

    resolve_out_path() runs first, so the result is under out/ and always
    carries an extension.

    Two runs that start inside the same second get a -2, -3, ... suffix. The
    PNG fallback directory is derived from the returned path, so it is checked
    for collisions too and stays unique alongside the mp4.
    """
    stem, ext = os.path.splitext(resolve_out_path(path))
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")

    candidate = f"{stem}_{stamp}{ext}"
    n = 2
    while os.path.exists(candidate) or os.path.exists(
            os.path.splitext(candidate)[0] + "_frames"):
        candidate = f"{stem}_{stamp}-{n}{ext}"
        n += 1
    return candidate


def look_at_matrix(eye, target, up):
    """World transform for a camera at eye looking at target."""
    view = Gf.Matrix4d()
    view.SetLookAt(Gf.Vec3d(*[float(v) for v in eye]),
                   Gf.Vec3d(*[float(v) for v in target]),
                   Gf.Vec3d(*[float(v) for v in up]))
    return view.GetInverse()


def measure_scene_bounds(stage, prim_path=SCENE_BOUNDS_PRIM,
                         fallback=FALLBACK_SCENE_BOUNDS):
    """Return (min_x, min_y, max_x, max_y) of the walls, in world units.

    Measured off the stage rather than hardcoded so a new floorplan frames
    itself, with two guards against measuring the wrong thing:

    Each wall is measured individually and unioned, rather than taking the
    bound of their parent. A parent bound sweeps up everything in the subtree,
    and the default ground plane is a hundred metres across, so one stray prim
    under it pushes the camera far enough back that the room becomes a smudge.

    A result larger than MAX_SCENE_SPAN is rejected outright. At that point
    something other than a room has been measured, and falling back to known
    extents beats framing on it.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print(f"[rec] no {prim_path} to measure, framing top-down on {fallback}")
        return fallback

    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    # The walls themselves, not their container.
    children = list(prim.GetChildren()) or [prim]
    box = Gf.Range3d()
    measured = 0
    for child in children:
        extent = cache.ComputeWorldBound(child).ComputeAlignedRange()
        if not extent.IsEmpty():
            box.UnionWith(extent)
            measured += 1

    if measured == 0 or box.IsEmpty():
        print(f"[rec] nothing measurable under {prim_path}, framing top-down "
              f"on {fallback}")
        return fallback

    lo, hi = box.GetMin(), box.GetMax()
    bounds = (float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1]))
    span = max(bounds[2] - bounds[0], bounds[3] - bounds[1])

    if span > MAX_SCENE_SPAN:
        print(f"[rec] {prim_path} measures {span:.0f}m across from {measured} "
              f"prim(s), too large to be a room. Framing top-down on {fallback} "
              f"instead. Pass explicit bounds if the scene really is that big.")
        return fallback

    print(f"[rec] measured {measured} prim(s) under {prim_path}, "
          f"{bounds[2] - bounds[0]:.1f}x{bounds[3] - bounds[1]:.1f}m")
    return bounds


def top_down_view(bounds, focal_length, width, height, zoom=1.0,
                  horizontal_aperture=DEFAULT_HORIZONTAL_APERTURE):
    """Overhead camera framing bounds: (eye, target, up, apertures).

    Height comes from the lens rather than a guessed number: at distance d a
    camera sees d * aperture / focal across, so d = span * focal / aperture is
    the distance at which the floorplan exactly fills the pane. Both axes are
    checked and the larger distance wins, since the pane's aspect ratio decides
    which one is the binding constraint.

    Both apertures are returned so the caller can author them onto the prim.
    That is not cosmetic. USD leaves verticalAperture at 15.2908 unless told
    otherwise, and renderers disagree about which aperture is authoritative:
    some derive vertical from horizontal and the render product's aspect, some
    keep the authored vertical and derive horizontal. Authoring both in the
    pane's own aspect ratio (h/v == width/height) makes the two conventions
    agree, so the distance computed here is the distance actually used. Left
    unauthored, the default vertical is 1.3x too long at 16:9 and the room
    lands at about 60% of the frame.

    zoom > 1 moves the camera closer, cropping into the middle of the
    floorplan. zoom = 1 fits the whole thing.

    up is +y, not +z. Looking straight down makes the view direction parallel
    to +z, and SetLookAt cannot build a basis from a degenerate pair. +y up
    also puts north at the top, which is how a floorplan is normally read.
    """
    min_x, min_y, max_x, max_y = bounds
    span_x, span_y = max_x - min_x, max_y - min_y
    centre = ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)

    vertical_aperture = horizontal_aperture * (float(height) / float(width))
    distance = TOP_MARGIN / float(zoom) * max(
        span_x * focal_length / horizontal_aperture,
        span_y * focal_length / vertical_aperture,
    )

    return ((centre[0], centre[1], distance), (centre[0], centre[1], 0.0),
            (0.0, 1.0, 0.0), (horizontal_aperture, vertical_aperture))


class Recorder:
    """Renders one or two camera prims to an mp4, side by side.

    Pane 1 is a fixed top-down shot of the floorplan, pane 2 the chase camera.
    Two render products means two renders per captured frame, so recording
    costs roughly twice what it did with one pane. It does not touch the
    physics rate, only the frames actually captured.
    """

    def __init__(self, out_path, resolution=DEFAULT_RESOLUTION, fps=DEFAULT_FPS,
                 focal_length=DEFAULT_FOCAL_LENGTH,
                 camera_path="/World/RecorderCam", unique=True,
                 top_down=True, top_camera_path="/World/RecorderCamTop",
                 top_focal_length=DEFAULT_TOP_FOCAL_LENGTH,
                 top_zoom=DEFAULT_TOP_ZOOM, scene_bounds=None,
                 banner_height=DEFAULT_BANNER_HEIGHT):
        # Every run gets its own file by default. Overwriting the previous one
        # is almost never what you want when the whole point is comparing runs.
        self.out_path = (timestamped_path(out_path) if unique
                         else resolve_out_path(out_path))
        # Per pane, not per file. The chase pane keeps the framing it had
        # before the top-down was added; the composite is simply wider.
        self.width, self.height = int(resolution[0]), int(resolution[1])
        self.fps = int(fps)
        self.frames = 0
        self._proc = None
        self._png_dir = None
        self._top_annotator = None
        self.banner_height = max(0, int(banner_height))
        self._banner = None

        parent = os.path.dirname(self.out_path)
        if parent:
            os.makedirs(parent, exist_ok=True)

        stage = omni.usd.get_context().get_stage()
        self._camera = UsdGeom.Camera.Define(stage, camera_path)
        # One transform op, reused every frame. Adding an op per frame would
        # pile up hundreds of them on the prim.
        self._xform_op = self._camera.AddTransformOp()
        self._camera.GetFocalLengthAttr().Set(float(focal_length))
        self.look_from((0.5, 4.5, 2.5), (1.5, 1.5, 0.3))
        self._annotator = self._attach_pane(camera_path)

        if top_down:
            bounds = (measure_scene_bounds(stage) if scene_bounds is None
                      else scene_bounds)
            top_camera = UsdGeom.Camera.Define(stage, top_camera_path)
            top_op = top_camera.AddTransformOp()
            top_camera.GetFocalLengthAttr().Set(float(top_focal_length))

            aperture = (top_camera.GetHorizontalApertureAttr().Get()
                        or DEFAULT_HORIZONTAL_APERTURE)
            eye, target, up, apertures = top_down_view(
                bounds, float(top_focal_length), self.width, self.height,
                float(top_zoom), float(aperture))

            # Author both, so the framing above is the framing rendered. See
            # top_down_view() for why leaving vertical alone throws it off.
            top_camera.GetHorizontalApertureAttr().Set(float(apertures[0]))
            top_camera.GetVerticalApertureAttr().Set(float(apertures[1]))

            # Fixed for the whole run: it frames the building, not the robot.
            top_op.Set(look_at_matrix(eye, target, up))
            self._top_annotator = self._attach_pane(top_camera_path)

            # Ground coverage, so a mis-framed pane can be diagnosed from the
            # log instead of by squinting at the video.
            covers = (eye[2] * apertures[0] / top_focal_length,
                      eye[2] * apertures[1] / top_focal_length)
            print(f"[rec] top-down pane framing x[{bounds[0]:.1f}, "
                  f"{bounds[2]:.1f}] y[{bounds[1]:.1f}, {bounds[3]:.1f}] "
                  f"({bounds[2] - bounds[0]:.1f}x{bounds[3] - bounds[1]:.1f}m) "
                  f"from {eye[2]:.1f}m up on a {top_focal_length:.0f}mm lens, "
                  f"covering {covers[0]:.1f}x{covers[1]:.1f}m at zoom "
                  f"{top_zoom:.2f}")

        out_w, out_h = self.frame_size()
        if shutil.which("ffmpeg") is None:
            self._png_dir = os.path.splitext(self.out_path)[0] + "_frames"
            os.makedirs(self._png_dir, exist_ok=True)
            print(f"[rec] ffmpeg not found, writing PNGs to {self._png_dir}")
        else:
            panes = 2 if self._top_annotator is not None else 1
            banner = (f", plus a {out_w}x{self.banner_height} banner"
                      if self.banner_height > 0 else "")
            print(f"[rec] recording {out_w}x{out_h} at {self.fps} fps, "
                  f"{panes} pane(s) of {self.width}x{self.height}{banner}, "
                  f"{focal_length:.0f}mm chase lens, to {self.out_path}")

    def _attach_pane(self, camera_path):
        """Wire one camera prim up to its own rgb annotator."""
        render_product = rep.create.render_product(camera_path,
                                                   (self.width, self.height))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([render_product])
        return annotator

    def panes_width(self):
        """Width of the rendered panes alone, before the banner is stacked on.

        Separate from frame_size() because the banner has to be built at this
        width, and asking frame_size() for it during construction would be
        circular — the frame is only that tall BECAUSE there is a banner.
        """
        if self._top_annotator is None:
            return self.width
        return self.width * 2 + DIVIDER_WIDTH

    def banner_size(self):
        """(width, height) the banner must be, or None if there is no banner."""
        if self.banner_height <= 0:
            return None
        return (self.panes_width(), self.banner_height)

    def set_banner(self, rgb):
        """Supply the next frame's banner. Call before capture().

        The banner PERSISTS until replaced, so a caller that only refreshes it
        when something changed still gets it on every frame. Passing None goes
        back to a blank strip rather than removing the pane, because removing it
        mid-run would change the frame size and ffmpeg is already running at the
        old one.
        """
        if rgb is None:
            self._banner = None
            return
        want = self.banner_size()
        if want is None:
            raise RuntimeError("this Recorder was built with banner_height=0; "
                               "there is nowhere to put a banner")
        rgb = np.asarray(rgb, dtype=np.uint8)
        if rgb.shape[:2] != (want[1], want[0]):
            raise RuntimeError(
                f"banner is {rgb.shape[1]}x{rgb.shape[0]}, expected "
                f"{want[0]}x{want[1]} — see Recorder.banner_size()")
        self._banner = rgb

    def frame_size(self):
        """Size of the composited frame, which is what ends up in the mp4."""
        width, height = self.panes_width(), self.height
        if self.banner_height > 0:
            height += self.banner_height + DIVIDER_WIDTH
        return (width, height)

    # --- camera placement ---------------------------------------------------

    def look_from(self, eye, target, up=(0.0, 0.0, 1.0)):
        """Point the chase camera at target from eye. Call before the render.

        Only the chase pane moves. The top-down camera is placed once in the
        constructor and left alone.
        """
        self._xform_op.Set(look_at_matrix(eye, target, up))

    def follow(self, position, offset=(-1.0, 3.0, 2.4), lead=(0.0, 0.0, 0.0)):
        """Track a moving subject from a fixed world-axis offset.

        The offset does not rotate with the subject's heading. A camera that
        spins with yaw is hard to watch when the yaw is the thing you are
        trying to judge.

        The offset has to keep the camera inside the room. lab42's walls are
        1.88 m tall and solid, so a camera parked outside them films a wall.
        """
        px, py, pz = float(position[0]), float(position[1]), float(position[2])
        self.look_from((px + offset[0], py + offset[1], pz + offset[2]),
                       (px + lead[0], py + lead[1], pz + lead[2]))

    # --- frames -------------------------------------------------------------

    @staticmethod
    def _pane_rgb(annotator):
        """One pane's frame as HxWx3 uint8, or None if it is not ready yet."""
        frame = np.asarray(annotator.get_data())
        if frame.size == 0:
            return None  # renderer still warming up
        if frame.ndim == 3 and frame.shape[2] == 4:
            frame = frame[:, :, :3]
        return frame.astype(np.uint8)

    def _composite(self, left, right):
        """Lay the two panes side by side with a seam between them."""
        if left.shape[0] != right.shape[0]:
            raise RuntimeError(
                f"pane heights differ ({left.shape[0]} vs {right.shape[0]}), "
                "cannot composite")
        divider = np.full((left.shape[0], DIVIDER_WIDTH, 3), DIVIDER_COLOR,
                          dtype=np.uint8)
        return np.hstack((left, divider, right))

    def _stack_banner(self, panes):
        """Put the banner above the rendered panes, with the same seam."""
        if self.banner_height <= 0:
            return panes
        banner = self._banner
        if banner is None:
            banner = np.full((self.banner_height, panes.shape[1], 3),
                             DIVIDER_COLOR, dtype=np.uint8)
        divider = np.full((DIVIDER_WIDTH, panes.shape[1], 3), DIVIDER_COLOR,
                          dtype=np.uint8)
        return np.vstack((banner, divider, panes))

    def capture(self):
        """Grab the frame just rendered. Silently skips frames not yet ready."""
        rgb = self._pane_rgb(self._annotator)
        if rgb is None:
            return False

        if self._top_annotator is not None:
            top = self._pane_rgb(self._top_annotator)
            # Both panes have to be ready, otherwise the frame would pair a
            # live chase view with a blank half.
            if top is None:
                return False
            # Top-down on the left: it is the pane you look at to know where
            # the robot is, and the chase view follows from it.
            rgb = self._composite(top, rgb)

        rgb = np.ascontiguousarray(self._stack_banner(rgb))

        if self._png_dir is not None:
            from PIL import Image
            Image.fromarray(rgb).save(
                os.path.join(self._png_dir, f"frame_{self.frames:05d}.png"))
        else:
            if self._proc is None:
                self._proc = self._start_ffmpeg(rgb.shape[1], rgb.shape[0])
            try:
                self._proc.stdin.write(rgb.tobytes())
            except BrokenPipeError:
                # ffmpeg died mid-run. Report it here rather than letting the
                # run finish and hand back a file that will not play.
                raise RuntimeError(
                    "ffmpeg exited early, recording aborted. Its output is "
                    f"above; partial file at {self.out_path}") from None

        self.frames += 1
        return True

    def _start_ffmpeg(self, width, height):
        cmd = [
            "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
            "-f", "rawvideo", "-pix_fmt", "rgb24",
            "-s", f"{width}x{height}", "-r", str(self.fps),
            "-i", "-",
            "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "20",
            "-pix_fmt", "yuv420p", self.out_path,
        ]
        return subprocess.Popen(cmd, stdin=subprocess.PIPE)

    def close(self):
        """Finalise the file. Must run before SimulationApp.close().

        ffmpeg writes the mp4 index last, so the file is unplayable until it
        exits cleanly. Waiting is not optional, and neither is checking how it
        exited: a non-zero ffmpeg leaves a file that looks written but has no
        moov atom.
        """
        if self._proc is not None:
            self._proc.stdin.close()
            code = self._proc.wait()
            self._proc = None
            if code != 0:
                print(f"[rec] ffmpeg exited {code}, {self.out_path} is "
                      f"probably unplayable")
                return

        seconds = self.frames / float(self.fps) if self.fps else 0.0
        where = self._png_dir if self._png_dir is not None else self.out_path
        print(f"[rec] wrote {self.frames} frames ({seconds:.1f}s of video) to {where}")
