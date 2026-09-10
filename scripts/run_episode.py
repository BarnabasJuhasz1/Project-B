"""
run_episode.py

One scenario, one controller, headless, scored, with provenance. This is the
"one scripted episode running headless and reproducibly" that Week 1 asks for,
and the thing the pilot, the unit harness and the four-condition ablation all
reuse.

    python scripts/run_episode.py --scenario doorway_traverse --controller reactive_avoid
    python scripts/run_episode.py --scenario blind_corner --controller goal_seeker --no-record

Writes one self-contained run directory, plus a committed copy of the score:

    out/runs/<run_id>__<utc>/trace.csv      per-control-step trace
    out/runs/<run_id>__<utc>/manifest.json  provenance
    out/runs/<run_id>__<utc>/result.json    the same {result, manifest} as below
    out/runs/<run_id>__<utc>/episode.mp4    two-pane video, on by default
    results/<run_id>.json                   scored result + manifest, committed

`<run_id>` is `<scenario>__<controller>__seed<N>` — the identity of the EPISODE,
and the name of the committed result. The run directory adds the start time, so
it is the identity of one INVOCATION and nothing ever overwrites it. That
distinction is not cosmetic: check_determinism.sh runs the same episode twice
with --results-dir pointing at a temp directory, and under the old flat layout
its second run silently replaced the trace belonging to a committed result.
`manifest["run_dir"]` names the directory a committed result came from, and
core/tests/test_results_integrity.py fails if the two ever disagree.

RECORDING IS ON BY DEFAULT and dominates wall clock on the authoring GPU:
measured 0.87 s per frame at 1280x720 and 0.97 s at 960x540, so the per-frame
cost is fixed overhead rather than pixels and --fps is the only real lever.
A 15 s episode takes 21 s headless, 240 s at 15 fps, 425 s at 30 fps; a 120 s
timeout episode at 15 fps is closer to half an hour.

EPISODES ARE CAPPED AT 60 SIM-SECONDS BY DEFAULT (--max-seconds), which caps
the video with them: at 15 fps and ~0.9 s per frame the difference is ~15 min
against ~30 for a 120 s timeout scenario. The cap is a SCENARIO CHANGE, not a
rendering option — a controller that would have reached the goal at 80 s times
out at 60 — so when it actually bites it renames the scenario `<id>+cap60s`,
the same discipline --goal-hidden follows, and a capped run can never land on
top of the committed full-length result. A scenario already inside the cap is
untouched: same id, same trace, same score. The evaluation sweep passes
`--max-seconds 0` and is not capped.

Rendering never advances physics, so the trace is byte-identical either way —
this re-run reproduced a trace from two days and one refactor earlier to the
last bit. What you pay for is the video, nothing else. Pass --no-record for
batch runs, seed sweeps and anything scored in bulk.

The reference controllers are harness validation, NOT benchmark conditions —
goal_seeker has no obstacle awareness and is expected to walk into the divider.
See controllers/*.py.

Local runs are for authoring and debugging. Every number that gets reported must
come from a headless run inside the pinned container; the manifest records
in_container so the two can never be confused.
"""

import argparse
import datetime
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

# argparse first, so a malformed flag is rejected before the simulator spends
# 20 s starting up. Everything imported here is sim-free.
from core.scenario import DriftSpec, all_scenarios, load
import controllers


# The default ceiling on an episode, in SIM seconds. Rendering costs ~0.9 s per
# frame whatever its size, so the video's wall clock is (time_limit * fps *
# 0.9) and the only two levers are fps and this. 60 s covers every dev scenario
# a controller actually solves — the longest committed success is 15.5 s — and
# halves the worst case, a 120 s timeout, from ~30 min of rendering to ~15.
DEFAULT_MAX_SECONDS = 60.0


def max_seconds(value):
    """--max-seconds, validated before the simulator boots. 0 disables it."""
    seconds = float(value)
    if seconds < 0:
        raise argparse.ArgumentTypeError("--max-seconds cannot be negative "
                                         "(0 means uncapped)")
    return seconds


def scenario_id(value):
    """Validate --scenario before the simulator boots, WITHOUT listing the
    held-out ids in --help.

    `choices=` would have to be built from every runnable scenario, and Week 5
    has to run the sealed split — so choices would print the held-out ids to
    anyone typing --help, and pasting those where a model can see them is the
    one failure mode sealing cannot survive. core.scenario.load() already
    resolves a held-out id by name and already refuses to enumerate them, so
    delegating to it is both the smaller change and the safer one.

    It also means a drift-carrying scenario is rejected here, with load()'s own
    explanation, rather than 20 s into a simulator boot.
    """
    try:
        load(value)
    except (KeyError, ValueError) as exc:
        raise argparse.ArgumentTypeError(str(exc).strip('"')) from exc
    return value


parser = argparse.ArgumentParser()
parser.add_argument("--scenario", required=True, type=scenario_id,
                    metavar="ID",
                    help="dev: " + ", ".join(s.id for s in all_scenarios()) +
                         "; held-out ids resolve by name but are deliberately "
                         "not listed here")
def controller_name(value):
    """Resolve --controller before the simulator boots.

    Not `choices=controllers.available()`: available() deliberately excludes
    scaffolding so nothing can sweep it into an evaluation, and choices= would
    then make it unrunnable — the same trap --scenario hit when all_scenarios()
    stopped returning the sealed split. Naming one is explicit; listing it is
    what we avoid.
    """
    try:
        controllers.make(value)
    except KeyError as exc:
        raise argparse.ArgumentTypeError(str(exc).strip('"')) from exc
    return value


parser.add_argument("--controller", required=True, type=controller_name,
                    metavar="NAME",
                    help="conditions: " + ", ".join(controllers.available()) +
                         "; scaffolding (never a condition): " +
                         ", ".join(controllers.scaffolding()))
parser.add_argument("--seed", type=int, default=None,
                    help="override the scenario's seed, for a seed sweep")
parser.add_argument("--scene", default=None,
                    help="override the scenario's scene USD")
parser.add_argument("--drift", type=float, default=None, metavar="SIGMA_XY",
                    help="inject localisation drift into a scenario that does "
                         "not declare any, for debugging. SIGMA_XY is metres "
                         "per root-second of RMS radial error, so 0.12 crosses "
                         "LOC_FAIL_THRESHOLD at ~69 s. A scenario's own "
                         "DriftSpec is the real source; this only fills one in.")
parser.add_argument("--sensor-noise", type=float, default=None, metavar="SCALE",
                    help="inject lidar noise into a scenario that declares "
                         "none, for debugging. SCALE multiplies "
                         "core.sensor_noise.NOISE_MODERATE, so 1.0 is ~3 cm at "
                         "6 m with 2%% dropout. LIKE --goal-hidden and UNLIKE "
                         "--drift this RENAMES the scenario (<id>+noise): "
                         "drift perturbs the pose ESTIMATE and leaves what the "
                         "robot can see alone, whereas sensor noise changes "
                         "Observation.ranges itself — which every controller "
                         "and every node reads — so it is a different scenario "
                         "and must not be scored under the same run id")
parser.add_argument("--localisation", choices=("perfect", "slam"), default=None,
                    metavar="KIND",
                    help="replace the pose estimate's producer, for debugging "
                         "epic 03. 'perfect' is the ground-truth CONTROL and is "
                         "never a result; 'slam' runs core/slam end to end and "
                         "needs a sensor, so pass --sensor-noise with it or the "
                         "matcher is fed core.geometry's EXACT ranges and "
                         "reports an accuracy that means nothing. LIKE "
                         "--sensor-noise and UNLIKE --drift this RENAMES the "
                         "scenario (<id>+slam / <id>+perfect): it changes what "
                         "the controller believes about where it is, so it is a "
                         "different scenario and must not be scored under the "
                         "same run id. Mutually exclusive with --drift, which "
                         "core.scenario.validate() also enforces")
parser.add_argument("--slam-map", action="store_true",
                    help="keep the map and the pose graph, and write them to "
                         "slam_map.npz beside slam.csv, for "
                         "scripts/visualise_slam.py to draw. INERT: a recorded "
                         "run produces a byte-identical trace.csv and "
                         "slam.csv, on the same terms nodes/trace.py is inert, "
                         "and core/tests/test_slam_record.py pins that. Off by "
                         "default because it costs a copy of every scan point, "
                         "which a sweep has no use for. Ignored by a localiser "
                         "that has no map, which is all of them but slam")
parser.add_argument("--goal-hidden", action="store_true",
                    help="withhold the goal from a scenario that discloses it, "
                         "for debugging a search-family controller against a "
                         "dev scenario. The goal then latches on line of sight "
                         "within detect_radius, by the same rule the sealed "
                         "split uses. UNLIKE --drift this RENAMES the scenario "
                         "(<id>+hidden): drift adds a noise process to an "
                         "otherwise identical episode, whereas hiding the goal "
                         "changes what the controller can see, so it is a "
                         "different scenario and must not be scored under the "
                         "same run id")
parser.add_argument("--max-seconds", type=max_seconds, metavar="S",
                    default=DEFAULT_MAX_SECONDS,
                    help=f"ceiling on the episode in SIM seconds, default "
                         f"{DEFAULT_MAX_SECONDS:.0f}; 0 is uncapped. This caps "
                         f"the simulation, and therefore the video with it. A "
                         f"scenario whose own time_limit is already inside the "
                         f"cap is untouched; one that is not is RENAMED "
                         f"<id>+cap<S>s, because timing out early is a "
                         f"different episode and must not overwrite the "
                         f"full-length result. The evaluation sweep passes 0")
parser.add_argument("--gui", action="store_true",
                    help="open a viewport. Renders well below real time on the "
                         "authoring machine; use --record instead")
parser.add_argument("--record", metavar="PATH", default=None,
                    help="override where the mp4 goes. Relative paths go under "
                         "out/. By default a run records to its own run "
                         "directory and this flag is not needed")
parser.add_argument("--no-record", action="store_true",
                    help="skip the video. Rendering costs ~0.9 s per frame on "
                         "the authoring GPU (a 15 s episode: 425 s recorded vs "
                         "21 s headless), so batch runs and seed sweeps want "
                         "this. The trace is identical either way")
parser.add_argument("--fps", type=int, default=15,
                    help="video frame rate, and THE cost lever: a frame costs "
                         "about the same to render whatever its size, so 15 "
                         "halves the wall clock against 30 and is still smooth "
                         "enough to review navigation")
parser.add_argument("--res", default="1280x720",
                    help="per pane, so the composite is twice this wide. "
                         "Measured at ~1 s per frame at both 1280x720 and "
                         "960x540, so lowering it buys nothing and takes DLSS "
                         "below its minimum input size")
parser.add_argument("--no-tree-pane", action="store_true",
                    help="drop the behaviour-tree banner from the video. The "
                         "pane is added automatically for any controller that "
                         "offers node tracing and costs nothing measurable "
                         "(a few ms of PIL against ~0.9 s of rendering), but "
                         "it makes the frame taller and a run being compared "
                         "against an older recording may want the old size")
parser.add_argument("--results-dir", default="results",
                    help="where the scored result.json goes")
parser.add_argument("--tag", default=None,
                    help="frozen library snapshot tag, recorded in the "
                         "manifest. Non-negotiable #8; None until Week 3")
args = parser.parse_args()

if args.record and args.no_record:
    parser.error("--record and --no-record contradict each other")

# --gui is for looking at something, and recording defeats it. An explicit
# --record still wins, as it always has; the automatic one steps aside rather
# than silently ignoring the flag the user actually typed.
if args.gui and not args.record:
    args.no_record = True
elif args.record and args.gui:
    print("[run_episode] --record turns off --gui, the viewport is not needed")
    args.gui = False

# Resolve the scenario before booting: an unreachable goal or a bad seed should
# fail in milliseconds, not after the simulator has started.
import dataclasses
import json
import math
import time

import numpy as np

from core.geometry import ROBOT_RADIUS, clearance, raycast
from core.manifest import capture, run_id, write as write_manifest
from core.localisation import make_localiser
from core.sensor_noise import make_sensor_noise
from core.metrics import EpisodeTrace, score
from core.observation import Observation, clamp
from core.scenario import load, validate

scenario = load(args.scenario)
if args.seed is not None:
    scenario = dataclasses.replace(scenario, seed=args.seed)
if args.scene is not None:
    scenario = dataclasses.replace(scenario, scene=args.scene)
if args.drift is not None:
    # sigma_yaw at half sigma_xy, the ratio the sealed split uses, so
    # `--drift 0.12` reproduces DRIFT_MODERATE on a dev scenario.
    scenario = dataclasses.replace(
        scenario, drift=DriftSpec(sigma_xy=args.drift,
                                  sigma_yaw=args.drift / 2.0))
if args.sensor_noise is not None:
    # The id changes with it, for the reason in the flag's help: this alters
    # what the controller can SEE, which is the line --goal-hidden is on and
    # --drift is not. A committed result called doorway_traverse that was run
    # with a degraded sensor is exactly the kind of thing nobody catches until
    # the write-up.
    import dataclasses as _dc

    from core.sensor_noise import NOISE_MODERATE
    scale = args.sensor_noise
    scenario = _dc.replace(
        scenario,
        id=f"{scenario.id}+noise",
        sensor_noise=_dc.replace(
            NOISE_MODERATE,
            sigma_range_frac=NOISE_MODERATE.sigma_range_frac * scale,
            dropout=min(1.0, NOISE_MODERATE.dropout * scale),
            spurious_max_range=min(1.0, NOISE_MODERATE.spurious_max_range * scale),
            sigma_angle=NOISE_MODERATE.sigma_angle * scale))

if args.localisation is not None:
    # Odometry noise at the same magnitude the drift model uses, and for the
    # same reason --drift halves sigma_yaw: so that a SLAM debugging run and a
    # DRIFT_MODERATE run are insulted by comparable amounts and the two numbers
    # can be put beside each other. `scripts/compare_localisers.py` is what
    # actually does that comparison; this flag is for looking at one episode.
    import dataclasses as _dc

    from core.scenario import LocalisationSpec as _LocSpec
    _sigma = args.drift if args.drift is not None else 0.12
    scenario = _dc.replace(
        scenario,
        id=f"{scenario.id}+{args.localisation}",
        # validate() refuses a scenario carrying both; --drift here is being
        # read as "how noisy", not as "use the random walk".
        drift=None,
        localisation=_LocSpec(kind=args.localisation,
                              odom_sigma_lin=_sigma,
                              odom_sigma_ang=_sigma / 2.0))

CAPPED = False
if args.goal_hidden:
    # The id changes with it. `results/<run_id>.json` is committed and names the
    # scenario, and a file called doorway_traverse that was not doorway_traverse
    # is the kind of thing nobody catches until the write-up.
    scenario = dataclasses.replace(scenario, id=f"{scenario.id}+hidden",
                                   goal_known=False)
    # load() validated the scenario it read; this one was built after that, and
    # the search-task bounds on detect_radius only apply once goal_known is
    # False. Re-running the check is the difference between a clear error here
    # and a meaningless episode twenty minutes later.
    validate(scenario)
if args.max_seconds and scenario.time_limit > args.max_seconds:
    # Truncating changes the OUTCOME, not just the wall clock: a controller
    # that reaches the goal at 80 s times out at 60, and goal-reach rate is
    # G2's primary metric. So the id changes with it, exactly as --goal-hidden
    # does — three committed results ran to a 120 s timeout, and a 60 s re-run
    # writing over one of them under the same run id is the kind of thing
    # nobody catches until the write-up.
    print(f"[run_episode] --max-seconds {args.max_seconds:g} truncates "
          f"{scenario.id} ({scenario.time_limit:.0f}s); scoring it as "
          f"{scenario.id}+cap{args.max_seconds:g}s. Pass --max-seconds 0 for "
          f"the scenario's own limit")
    scenario = dataclasses.replace(
        scenario, id=f"{scenario.id}+cap{args.max_seconds:g}s",
        time_limit=float(args.max_seconds))
    validate(scenario)
    CAPPED = True
controller = controllers.make(args.controller)

# The episode's identity: what the committed result is named after. Two
# invocations of the same scenario, controller and seed share it, and under the
# determinism guarantee they produce the same trace.
#
# core.manifest.run_id() rather than an f-string here, because a composed
# controller is named `composed:<id>` and a colon does not belong in a
# filename. One definition, shared with the batch runner and the integrity
# test, so the three cannot disagree about what an episode is called.
RUN_ID = run_id(scenario.id, args.controller, scenario.seed)

# One invocation's identity. Same thing plus the start time, so a re-run never
# lands on top of a directory a committed result still points at.
RUN_STARTED = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%d-%H%M%S")
RUN_DIR_NAME = f"{RUN_ID}__{RUN_STARTED}"

# resolve_out_path() in recorder.py anchors this under <repo>/out/, whatever the
# working directory. unique=False below because the run directory already
# carries the timestamp that would otherwise go in the filename.
RECORD_PATH = args.record or f"runs/{RUN_DIR_NAME}/episode.mp4"

# SimulationApp must be constructed before any other isaacsim or omni import.
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": not args.gui})

# Imported only now: both pull in omni at import time.
from go2_policy import open_episode                       # noqa: E402


# The robot is spawned slightly high and dropped so it settles into its own
# stance rather than being posed into one. walk_forward.py uses 0.40 against a
# nominal stance of ~0.32 — a short drop. build_scene.py's 0.55 is a longer one
# and the legs compress through FALL_HEIGHT on the way down, which is not a fall.
SPAWN_HEIGHT = 0.40

# Zero-command seconds before episode t=0. The episode clock starts once the
# robot is standing, so the settle is not charged against the time limit and a
# controller is never asked what to do about a falling robot.
SETTLE_SECONDS = 1.5


def log(msg):
    print(f"[run_episode] {msg}", flush=True)


def parse_resolution(text):
    width, _, height = text.lower().partition("x")
    return (int(width), int(height))


def observe(t, pose, angles, goal_known, localiser, dt, sensor):
    """Build the controller's view of the world.

    Ground truth does not enter here. `pose_estimate` is whatever the localiser
    believes — the true pose when nothing is configured, the drifted pose under
    a DriftSpec, and a SLAM estimate once chunk 23 lands. It is a separate field
    from the truth precisely so the producer can be swapped without rewriting
    anything downstream of it.

    `goal_known` and `localiser` are passed in rather than built here, because
    both carry episode state and this function is otherwise pure — the caller
    owns the memory.

    `pose_error_estimate` is what the robot BELIEVES its error to be, never what
    it actually is: the realised error is ground truth and stays with the
    runner, where the trace records it for scoring.

    THE SCAN IS CAST BEFORE THE LOCALISER UPDATES, and the order matters once
    the localiser is a real one: it matches the scan it is about to be judged
    on, and it must not see a scan from a step it has already reported a pose
    for. `raycast` is a pure function of pose, so hoisting it above the update
    changes nothing at all on the drift path — the trace stays byte-identical,
    which is chunk 19's acceptance criterion.

    `clearance` is deliberately NOT in the Observation. It is ground truth about
    geometry the robot cannot see, and it exists only to be scored.
    """
    x, y, z, yaw = pose
    gap = float(clearance((x, y))) - ROBOT_RADIUS
    ranges = sensor.scan((x, y), yaw, angles, scenario.range_max)
    localiser.update(dt, (x, y, yaw), ranges, angles)
    obs = Observation(
        t=t,
        ranges=ranges,
        range_angles=angles,
        range_max=scenario.range_max,
        pose_estimate=localiser.estimate(),
        pose_error_estimate=localiser.error_estimate(),
        goal=scenario.goal if goal_known else None,
        contact=gap <= 0.0,
    )
    return obs, gap


def main():
    from core.metrics import FALL_HEIGHT, GOAL_RADIUS

    scene_path = os.path.join(REPO_ROOT, scenario.scene)
    manifest = capture(scene=scenario.scene, scenario_id=scenario.id,
                       seed=scenario.seed, controller=args.controller,
                       library_snapshot=args.tag)
    # What bounded the episode, and whether that was the scenario's own limit
    # or this run's cap. A scored result has to say which, because the two
    # answer different questions and only the first is reportable.
    manifest["time_limit"] = scenario.time_limit
    manifest["time_limit_capped"] = CAPPED
    # What produced Observation.ranges and Observation.pose_estimate. Recorded
    # as VALUES, not as a flag: chunk 20's sigma is a calibration choice, and a
    # result that says only "noise: on" cannot be compared with one run at a
    # different sigma. None on both is the state every committed result is in.
    manifest["sensor_noise"] = (
        dataclasses.asdict(scenario.sensor_noise)
        if scenario.sensor_noise is not None else None)
    manifest["localisation"] = (
        dataclasses.asdict(scenario.localisation)
        if scenario.localisation is not None else None)
    manifest["drift"] = (
        dataclasses.asdict(scenario.drift)
        if scenario.drift is not None else None)
    if manifest["usd_sha256"] is None:
        sim_app.close()
        sys.exit(f"Scene not found: {scene_path}. Run build_scene.py first.")

    start = (scenario.start[0], scenario.start[1], SPAWN_HEIGHT, scenario.start[2])
    episode = open_episode(scene_path, start, log=log)

    # Settle before the clock starts. Without this the first second of every
    # episode is the drop, the base passes below FALL_HEIGHT as the legs
    # compress, and every run terminates as "fell" one second in.
    settle_steps = int(SETTLE_SECONDS / episode.control_dt)
    for _ in range(settle_steps):
        episode.drive((0.0, 0.0, 0.0))
    settled = episode.pose()
    log(f"settled after {SETTLE_SECONDS:.1f}s at "
        f"({settled[0]:.2f}, {settled[1]:.2f}, {settled[2]:.2f})")
    if settled[2] < FALL_HEIGHT:
        sim_app.close()
        sys.exit(f"robot did not stand up: base at z={settled[2]:.3f} after "
                 f"{SETTLE_SECONDS:.1f}s. Check the locomotion policy with "
                 f"scripts/testing/walk_forward.py before running episodes.")

    # Node tracing, for anything that offers it. The runner does not know what
    # a behaviour tree is — it asks, and a controller that cannot explain itself
    # simply does not answer. Recording is inert: a traced run produces a
    # byte-identical trace.csv, asserted in nodes/tests/test_trace.py.
    #
    # THIS RUNS BEFORE THE RECORDER, and the order is load-bearing: the tree
    # banner's height decides the frame size, and the frame size has to be
    # settled before ffmpeg is started.
    tick_log = None
    if hasattr(controller, "enable_trace"):
        tick_log = controller.enable_trace()
        log("node tracing on -> nodes.csv")

    # The third pane. Offered for any traced controller that is actually a tree
    # — `nodes/diagram.py` draws a tree, and asking a controller that merely
    # logs ticks to be one would be the runner assuming what it just took care
    # to ask about.
    diagram = None
    banner_height = 0
    if tick_log is not None and hasattr(controller, "root") \
            and not args.no_tree_pane and not args.no_record:
        from nodes.diagram import banner_height_for
        banner_height = banner_height_for(controller)

    recorder = None
    if not args.no_record:
        sys.path.insert(0, os.path.join(REPO_ROOT, "scripts"))
        from recorder import Recorder
        # unique=False for the default path: the run directory is already
        # unique, and a second timestamp inside it makes the file harder to
        # find, not safer. An explicit --record keeps the old behaviour.
        recorder = Recorder(RECORD_PATH, resolution=parse_resolution(args.res),
                            fps=args.fps, unique=bool(args.record),
                            banner_height=banner_height)
        log(f"recording to {os.path.relpath(recorder.out_path, REPO_ROOT)} "
            f"at {args.fps} fps — rendering dominates wall clock, "
            f"pass --no-record to skip it")
        if banner_height:
            from nodes.diagram import TreeDiagram
            width, height = recorder.banner_size()
            diagram = TreeDiagram(controller, width=width, height=height,
                                  subtitle=f"{args.controller}"
                                           f"{'  ·  ' + controller.snapshot if controller.snapshot else ''}")
            log(f"tree pane on — {len(diagram.boxes)} nodes, "
                f"{width}x{height}, pass --no-tree-pane to drop it")

    controller.reset(scenario, scenario.rng())
    angles = scenario.beam_angles()
    trace = EpisodeTrace()

    control_dt = episode.control_dt
    steps = int(scenario.time_limit / control_dt)
    log(f"{RUN_ID}: {steps} control steps at {1.0 / control_dt:.1f} Hz, "
        f"limit {scenario.time_limit:.0f}s, optimal {scenario.optimal_distance():.2f}m")

    started = time.time()
    next_frame = 0.0
    t = 0.0
    stop_reason = "timeout"

    # The disclosure latch. True from the start on every scenario that hands the
    # controller its goal, which is all three dev scenarios; on a search task it
    # flips once and never flips back. Held here because it is the only piece of
    # episode state the runner owns on the controller's behalf.
    goal_known = scenario.goal_known
    localiser = make_localiser(scenario)
    # Before reset(), so the anchor snapshot is taken. Duck-typed rather than
    # isinstance-checked against SlamLocaliser: `--slam-map` on a perfect or
    # drift run is a no-op and not an error, because the flag says what to keep
    # if there is anything to keep, and chunk 25's PerfectLocaliser control runs
    # the same command line as the SLAM run it is a control for.
    if args.slam_map and hasattr(localiser, "enable_recording"):
        localiser.enable_recording()
    localiser.reset(scenario, scenario.localisation_rng())
    sensor = make_sensor_noise(scenario)
    if scenario.sensor_noise is not None:
        log(f"sensor noise on: {scenario.sensor_noise}")
    if scenario.drift is not None:
        log(f"drift on: sigma_xy={scenario.drift.sigma_xy} m/sqrt(s), "
            f"crosses LOC_FAIL_THRESHOLD at ~"
            f"{(1.0 / scenario.drift.sigma_xy) ** 2:.0f}s of "
            f"{scenario.time_limit:.0f}s")
    elif scenario.localisation is not None:
        log(f"localiser: {scenario.localisation.kind} "
            f"({localiser.__class__.__name__})")
    # Diagnostics sidecar. Written only when a localiser actually reports
    # something, so a drift or no-drift run produces the same run directory it
    # always did -- TRACE_COLUMNS is fixed-width because scoring is a pure
    # function of the trace and rescore.py depends on it, so this goes beside
    # the trace and never into it. Same shape as nodes.csv (chunk 18).
    slam_log = []

    for step in range(steps):
        if args.gui and not sim_app.is_running():
            stop_reason = "window closed"
            break

        pose = episode.pose()
        # Evaluate the rule BEFORE building the observation, so the step the
        # robot first sees the goal is the step it is told about it.
        goal_known = goal_known or scenario.disclosed_at(pose[:2])
        obs, gap = observe(t, pose, angles, goal_known, localiser, control_dt,
                           sensor)
        diag = localiser.diagnostics()
        if diag:
            slam_log.append(dict(diag, t=t))
        command = clamp(controller.step(obs), scenario.max_lin, scenario.max_ang)
        trace.append(t, pose, command, gap, obs.contact, obs.pose_estimate,
                     goal_known)

        # Terminate on the same conditions core.metrics scores, so the trace
        # does not run on past the outcome. score() decides; this only stops.
        if math.hypot(pose[0] - scenario.goal[0],
                      pose[1] - scenario.goal[1]) <= GOAL_RADIUS \
                and pose[2] >= FALL_HEIGHT:
            stop_reason = "goal"
            break
        if pose[2] < FALL_HEIGHT:
            stop_reason = "fell"
            break

        want_frame = recorder is not None and t >= next_frame
        if want_frame:
            recorder.follow(np.array(pose[:3]))
            if diagram is not None:
                # Drawn from the tick log the step above just wrote, so the
                # tree shown is the tree that produced the command being
                # executed in this frame. Read-only: the log is evidence and
                # the video is not.
                recorder.set_banner(diagram.render(tick_log, t=t))
        # Rendering never advances physics (see Episode.drive), so a recorded
        # or watched run is the same run as a headless one.
        episode.drive(command.as_tuple(), render=want_frame or args.gui)
        if want_frame:
            recorder.capture()
            next_frame += 1.0 / args.fps

        t += control_dt

        if step % 200 == 0:
            log(f"t={t:6.1f}s pose=({pose[0]:5.2f},{pose[1]:5.2f},{pose[2]:4.2f}) "
                f"gap={gap:6.3f} cmd=({command.vx:.2f},{command.vy:.2f},{command.wz:.2f})")

    if recorder is not None:
        recorder.close()

    log(f"loop ended: {stop_reason}")
    result = score(trace, scenario)
    manifest["sim_seconds"] = float(t)
    manifest["wall_seconds"] = round(time.time() - started, 1)
    manifest["control_hz"] = round(1.0 / control_dt, 3)
    manifest["physics_dt"] = episode.physics_dt
    manifest["decimation"] = episode.decimation

    run_dir = os.path.join(REPO_ROOT, "out", "runs", RUN_DIR_NAME)
    os.makedirs(run_dir, exist_ok=True)

    # The result names the directory it came from, and the video it came with.
    # Without this a committed result cannot be tied back to its own trace, and
    # that is exactly the drift this layout exists to prevent.
    manifest["run_id"] = RUN_ID
    manifest["run_dir"] = os.path.relpath(run_dir, REPO_ROOT)
    manifest["video"] = (None if recorder is None
                         else os.path.relpath(recorder.out_path, REPO_ROOT))

    payload = {"result": result, "manifest": manifest}

    trace.write_csv(os.path.join(run_dir, "trace.csv"))
    if tick_log is not None and len(tick_log):
        tick_log.write_csv(os.path.join(run_dir, "nodes.csv"))
        log(f"{len(tick_log)} node ticks over {tick_log.steps} steps -> nodes.csv")
        # ⚠️ These four lines were INSIDE `if slam_log:` between chunks 19 and
        # 23. Nothing caught it because no run produced a slam_log until chunk
        # 23's first SLAM episode, and then it crashed AFTER the simulation had
        # finished but BEFORE the result was written -- three minutes of physics
        # discarded on an AttributeError in a log line. Same family as the sweep
        # rehearsal's "exit 0, wrote nothing": the expensive part succeeded and
        # the cheap part threw it away.
        for line in tick_log.format_summary().splitlines():
            log("  " + line)
        log("  timeline  # drove, + ticked, . gated out:")
        for node_id in tick_log.summary():
            log(f"  {node_id[:24]:24s} |{tick_log.timeline(node_id, 72)}|")

    # The localiser sidecar, written on exactly the same terms as nodes.csv:
    # a run that has nothing to report produces no file, so every run directory
    # committed before chunk 19 is reproduced byte for byte.
    if slam_log:
        import csv as _csv
        columns = sorted(slam_log[0])
        with open(os.path.join(run_dir, "slam.csv"), "w", newline="",
                  encoding="utf-8") as fh:
            writer = _csv.DictWriter(fh, fieldnames=columns)
            writer.writeheader()
            for row in slam_log:
                writer.writerow(row)
        log(f"{len(slam_log)} localiser samples -> slam.csv")
        last = slam_log[-1]
        log(f"  {last['submaps']} submaps, {last['graph_edges']} edges "
            f"({last['closure_edges']} closures), {last['solves']} solves; "
            f"closures {last['closures_accepted']}/"
            f"{last['closures_considered']} accepted")

    # The map, on the same terms again: only if something recorded one.
    recorder = getattr(localiser, "recorder", None)
    if recorder is not None and recorder.scans:
        map_path = os.path.join(run_dir, "slam_map.npz")
        recorder.save_npz(map_path)
        log(f"map: {recorder.n_submaps} submaps, {recorder.scans} scans, "
            f"{len(recorder.closures())} closure edges -> slam_map.npz "
            f"({os.path.getsize(map_path) / 1e6:.1f} MB)")
        log(f"  draw it:  $P scripts/visualise_slam.py {run_dir}")
    write_manifest(manifest, os.path.join(run_dir, "manifest.json"))

    # Written in the run directory FIRST, and copied to results/ second. The run
    # directory is the record; results/ is the committed projection of it.
    def write_payload(path):
        with open(path, "w") as fh:
            json.dump(payload, fh, indent=2, sort_keys=True)
            fh.write("\n")

    write_payload(os.path.join(run_dir, "result.json"))

    results_dir = os.path.join(REPO_ROOT, args.results_dir)
    os.makedirs(results_dir, exist_ok=True)
    result_path = os.path.join(results_dir, f"{RUN_ID}.json")
    write_payload(result_path)

    log(json.dumps(result, indent=2, sort_keys=True))
    log(f"wrote {os.path.relpath(result_path, REPO_ROOT)} and "
        f"{manifest['run_dir']}/{{trace.csv, manifest.json, result.json"
        f"{'' if recorder is None else ', episode.mp4'}}}")
    if not manifest["in_container"]:
        log("NOT in the pinned container: this number is for debugging, "
            "not for reporting")
    sim_app.close()


if __name__ == "__main__":
    main()
