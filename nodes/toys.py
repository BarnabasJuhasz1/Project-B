"""
nodes/toys.py

Toy leaves, and the smallest trees that are Controllers.

**NOT NAVIGATION, and they must never appear in a benchmark condition.** They
exist to prove the contract holds end to end: that a Sequence of typed leaves
drops into `scripts/run_episode.py` with no change to the runner, that ports
type-check at composition time, and that a leaf handles an undisclosed goal
without reading a stale one. `DriveToGoal` walks straight at the goal and into
the divider, exactly as `controllers/goal_seeker.py` does, and for the same
reason: a fixture that always succeeds proves nothing.

`AvoidObstacles` is the one exception to that last sentence and says why in its
own docstring: chunk 16 needed a composed tree that actually reaches the goal.

They are HANDWRITTEN, and their metadata says so. Origin is what G1's provenance
fraction is computed from, so a fixture claiming a source span would corrupt the
one number this whole contract exists to make countable.
"""

import math

from core.observation import Command

from nodes.contract import (Action, Condition, Direction, NodeMetadata, Origin,
                            Port, PortType, Status)

GOAL_KNOWN = Port("goal_known", PortType.FLAG, Direction.OUT)
GOAL_KNOWN_IN = Port("goal_known", PortType.FLAG, Direction.IN)


class IsGoalKnown(Condition):
    """Has the goal been disclosed to us yet?

    Reads `obs.goal is None` — the contract from chunk 11a — and publishes it as
    a Flag so that no other node has to re-derive it. It is monotonic within an
    episode because the runner latches disclosure, which is what makes it a
    precondition rather than a reading that could flap.
    """

    PORTS = (GOAL_KNOWN,)
    PRECONDITION = "always"
    EFFECT = "publishes whether the goal has been disclosed, as a Flag"
    TAGS = ("goal", "disclosure")

    def tick(self, obs, ports):
        known = obs.goal is not None
        ports["goal_known"] = known
        return Status.SUCCESS if known else Status.FAILURE


class DriveToGoal(Action):
    """Turn towards the goal, then walk at it. No sensing whatsoever.

    NOT NAVIGATION. It has no obstacle awareness and will walk into walls.
    """

    PORTS = (GOAL_KNOWN_IN,)
    PRECONDITION = "the goal has been disclosed"
    EFFECT = "emits a Twist that reduces the bearing and range to the goal"
    TAGS = ("goal", "drive")

    HEADING_TOLERANCE = 0.25
    TURN_GAIN = 2.0
    APPROACH_GAIN = 0.6

    def precondition(self, obs, ports):
        return ports["goal_known"]

    def tick(self, obs, ports):
        # The precondition guarantees a goal, so this cannot read a stale one:
        # if it were ever reached with obs.goal None it would raise here rather
        # than steer at wherever the goal was last episode.
        x, y, yaw = obs.pose_estimate
        dx, dy = obs.goal[0] - x, obs.goal[1] - y
        dist = math.hypot(dx, dy)

        err = math.atan2(dy, dx) - yaw
        err = (err + math.pi) % (2 * math.pi) - math.pi

        if abs(err) > self.HEADING_TOLERANCE:
            ports.drive(Command(0.0, 0.0, self.TURN_GAIN * err))
        else:
            ports.drive(Command(self.APPROACH_GAIN * min(dist, 1.0), 0.0,
                                self.TURN_GAIN * err))
        return Status.RUNNING


def toy_metadata(node_id):
    """Handwritten provenance. Deliberately carries no card and no source span:
    these are fixtures, and G1 must count them as untraceable."""
    return NodeMetadata(node_id=node_id, origin=Origin.HANDWRITTEN)


class SweepForGoal(Action):
    """Wander until something turns up. Never reads `obs.goal`.

    NOT NAVIGATION and never a benchmark condition. It delegates to
    `controllers.scaffold_sweep`, which exists for the same reason and says so
    itself: real exploration is what a generated condition has to produce, and
    a reference implementation of it would quietly become a baseline nobody
    pre-registered.

    It is here so that `search_tree()` has two branches that genuinely take turns
    — which is the thing a node trace is for.
    """

    PORTS = (GOAL_KNOWN_IN,)
    PRECONDITION = "the goal has NOT been disclosed"
    EFFECT = "emits a Twist that covers unexplored floor"
    TAGS = ("search", "coverage")

    def __init__(self, metadata):
        super().__init__(metadata)
        import controllers
        self._inner = controllers.make("scaffold_sweep")

    def reset(self, scenario, rng):
        self._inner.reset(scenario, rng)

    def precondition(self, obs, ports):
        # The mirror image of DriveToGoal's precondition: exactly one of the two
        # can run at any step, which is what makes the handover visible.
        return not ports["goal_known"]

    def tick(self, obs, ports):
        ports.drive(self._inner.step(obs))
        return Status.RUNNING


def search_tree():
    """Fallback: drive to the goal if we have one, otherwise go and find it.

    NOT NAVIGATION and never a benchmark condition — both leaves are toys. It
    exists because a tree whose branches take turns is the only way to see that
    a node trace is reporting anything: on a search scenario SweepForGoal drives
    until the goal enters detection range, and DriveToGoal takes over on exactly
    the step the disclosure latch flips.
    """
    from nodes.tree import Fallback, Sequence, Tree
    return Tree(Fallback(toy_metadata("toy_search_root"), [
        Sequence(toy_metadata("toy_goal_branch"), [
            IsGoalKnown(toy_metadata("toy_is_goal_known")),
            DriveToGoal(toy_metadata("toy_drive_to_goal")),
        ]),
        SweepForGoal(toy_metadata("toy_sweep_for_goal")),
    ]))


def toy_tree():
    """The smallest tree that is a Controller: IsGoalKnown then DriveToGoal.

    Registered in `controllers/` so that `run_episode.py --controller toy_tree`
    runs a behaviour tree through the real runner. That is chunk 12's headline
    claim — a composed tree drops in with no runner changes — and until this
    existed it was only ever demonstrated against the kinematic rollout.

    NOT NAVIGATION and never a benchmark condition, for the same reason the
    leaves above are not: DriveToGoal has no sensing and walks into walls.
    """
    from nodes.tree import Sequence, Tree
    return Tree(Sequence(toy_metadata("toy_seq"), [
        IsGoalKnown(toy_metadata("toy_is_goal_known")),
        DriveToGoal(toy_metadata("toy_drive_to_goal")),
    ]))


class AvoidObstacles(Action):
    """Walk towards the goal, going round what is in the way.

    NOT NAVIGATION IN THE SENSE THIS PROJECT MEANS IT, and never a benchmark
    condition. It delegates to `controllers.reactive_avoid`, which is harness
    validation — a hand-written monolith with goal seeking, gap selection and
    speed control welded into one `step()`, which is exactly the shape
    `nodes/contract.py` exists to forbid. Wrapping it in a leaf does not make it
    a leaf: it has one port and one docstring, and everything it actually
    decides is still inside the monolith.

    It is here for one reason. Chunk 16 needs a composed tree that genuinely
    reaches the goal, so that the composition path can be shown to work end to
    end while the generated library is one node long — and a fixture that always
    fails would prove as little as `DriveToGoal` walking into the divider does.
    The moment the library holds real Actions, compositions stop retrieving this
    and it goes back to being what `SweepForGoal` is: a fixture.
    """

    PORTS = (GOAL_KNOWN_IN,)
    PRECONDITION = "the goal has been disclosed"
    EFFECT = "emits a Twist that reduces range to the goal while keeping clear of obstacles"
    TAGS = ("goal", "drive", "obstacle", "avoid", "clearance")

    def __init__(self, metadata):
        super().__init__(metadata)
        import controllers
        self._inner = controllers.make("reactive_avoid")

    def reset(self, scenario, rng):
        self._inner.reset(scenario, rng)

    def precondition(self, obs, ports):
        return ports["goal_known"]

    def tick(self, obs, ports):
        ports.drive(self._inner.step(obs))
        return Status.RUNNING
