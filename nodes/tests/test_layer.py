"""
nodes/tests/test_layer.py

**The layered command pipeline** — chunk 52.

The question these tests answer is not "does a Layer tick everybody" (it
obviously does, it is six lines). It is whether the three properties the chunk
claims are STRUCTURAL actually are:

1. a modifier cannot write a field it did not declare — and the test for this is
   not that it politely refrains, but that it RAISES when it tries;
2. a modifier that declines leaves the driver's command **byte-identical**,
   which is what makes "supports the base rather than replacing it" true by
   construction;
3. the two combinators are the boring ones they were declared to be —
   last-writer-wins for `adjust`, `min()` of magnitude for `cap` — because the
   arbitration rule is part of the experiment and is pre-registered.

The fixtures here are HANDWRITTEN and are not navigation. They exist to exercise
the contract and can never be a benchmark condition (#3).
"""

import math

import pytest

from core.observation import Command
from nodes.contract import (Action, CommandFieldError, Condition, Kind,
                            PortStore, PortView, Status, TWIST_FIELDS)
from nodes.toys import toy_metadata
from nodes.tree import Fallback, Layer, Sequence, tick_node


# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

class ConstantDriver(Action):
    """Proposes the same whole Twist every tick. Stands in for the base."""

    PRECONDITION = "always"
    EFFECT = "proposes a fixed command"

    def __init__(self, metadata, command=Command(0.8, 0.0, 0.5)):
        super().__init__(metadata)
        self.command = command

    def tick(self, obs, ports):
        ports.drive(self.command)
        return Status.RUNNING


class SpeedCap(Action):
    """A modifier: caps forward speed, and may touch nothing else."""

    WRITES = ("vx",)
    PRECONDITION = "always"
    EFFECT = "bounds vx"

    def __init__(self, metadata, limit=0.2, applies=True):
        super().__init__(metadata)
        self.limit = limit
        self.applies = applies

    def precondition(self, obs, ports):
        return self.applies

    def tick(self, obs, ports):
        ports.cap(vx=self.limit)
        return Status.RUNNING


class Overreacher(Action):
    """A modifier that declares vx and writes wz anyway. Must raise."""

    WRITES = ("vx",)
    PRECONDITION = "always"
    EFFECT = "tries to steer while claiming to limit speed"

    def tick(self, obs, ports):
        ports.adjust(wz=1.0)
        return Status.RUNNING


class Counter(Condition):
    """Records that it was ticked. Never touches the Twist."""

    PRECONDITION = "always"
    EFFECT = "counts ticks"

    def __init__(self, metadata):
        super().__init__(metadata)
        self.ticks = 0

    def tick(self, obs, ports):
        self.ticks += 1
        return Status.SUCCESS


class FailingLeaf(Condition):
    PRECONDITION = "never"
    EFFECT = "always fails"

    def precondition(self, obs, ports):
        return False

    def tick(self, obs, ports):          # pragma: no cover - never reached
        raise AssertionError("a gated node must not tick")


def _store():
    return PortStore()


def _tick(node, store, obs=None):
    return tick_node(node, obs, store)


# --- 1. the composite ------------------------------------------------------

def test_layer_ticks_every_child_where_sequence_and_fallback_do_not():
    """The structural fact chunk 52 turns on: a driver returning RUNNING stops
    both existing composites dead, so 'tree order is priority' needed a new
    composite whatever else was decided."""
    def children():
        return [ConstantDriver(toy_metadata("driver")),
                Counter(toy_metadata("after"))]

    for composite in (Sequence, Fallback):
        kids = children()
        _tick(composite(toy_metadata("c"), kids), _store())
        assert kids[1].ticks == 0, f"{composite.__name__} reached the second child"

    kids = children()
    status = _tick(Layer(toy_metadata("layer"), kids), _store())
    assert kids[1].ticks == 1
    assert status is Status.RUNNING


def test_a_layer_where_everybody_fails_reports_that_nobody_handled_the_step():
    store = _store()
    status = _tick(Layer(toy_metadata("layer"),
                         [FailingLeaf(toy_metadata("a")),
                          FailingLeaf(toy_metadata("b"))]), store)
    assert status is Status.FAILURE
    assert store.command is None


# --- 2. field scope: the load-bearing restriction ---------------------------

def test_a_modifier_cannot_write_a_field_it_did_not_declare():
    store = _store()
    _tick(ConstantDriver(toy_metadata("driver")), store)
    with pytest.raises(CommandFieldError, match="does not declare in WRITES"):
        _tick(Overreacher(toy_metadata("overreach")), store)


def test_a_modifier_may_not_seize_the_whole_command_with_drive():
    """Narrowing WRITES is a promise to revise rather than replace. drive()
    would break the promise, so it is refused rather than trusted."""
    store = _store()
    _tick(ConstantDriver(toy_metadata("driver")), store)
    node = SpeedCap(toy_metadata("cap"))
    with pytest.raises(CommandFieldError, match="is a modifier"):
        PortView(store, node).drive(Command(1.0, 1.0, 1.0))


def test_a_modifier_with_nothing_to_modify_says_so():
    """A modifier placed before any driver is a composition error, and the
    message has to name the fix rather than report a None."""
    with pytest.raises(CommandFieldError, match="before anything proposed"):
        _tick(SpeedCap(toy_metadata("cap")), _store())


def test_writes_is_validated_at_construction_not_at_tick_time():
    class Typo(Action):
        WRITES = ("vx", "omega")
        PRECONDITION = "x"
        EFFECT = "y"

    class ConditionThatWrites(Condition):
        WRITES = ("vx",)
        PRECONDITION = "x"
        EFFECT = "y"

    class MuteAction(Action):
        WRITES = ()
        PRECONDITION = "x"
        EFFECT = "y"

    with pytest.raises(ValueError, match="omega"):
        Typo(toy_metadata("typo"))
    with pytest.raises(ValueError, match="Only an Action"):
        ConditionThatWrites(toy_metadata("cw"))
    with pytest.raises(ValueError, match="can never emit"):
        MuteAction(toy_metadata("mute"))


def test_an_unnarrowed_action_is_a_driver_and_every_pre_chunk_52_node_is_one():
    """The compatibility claim: WRITES defaults to all three on Action, so
    nothing written before chunk 52 had to change to stay valid."""
    node = ConstantDriver(toy_metadata("driver"))
    assert tuple(node.WRITES) == TWIST_FIELDS
    assert not node.is_modifier
    assert SpeedCap(toy_metadata("cap")).is_modifier


# --- 3. the arbitration rule, which is pre-registration material ------------

def test_a_modifier_that_declines_leaves_the_command_byte_identical():
    """`fixed` and `broke` in the growth loop are only attributable if a gated
    modifier is provably inert. Identity, not equality: nothing reconstructed
    the command."""
    store = _store()
    driver = ConstantDriver(toy_metadata("driver"))
    layer = Layer(toy_metadata("layer"),
                  [driver, SpeedCap(toy_metadata("cap"), applies=False)])
    _tick(layer, store)
    assert store.command is driver.command
    assert store.command_writers == {f: "driver" for f in TWIST_FIELDS}


def test_a_cap_that_does_not_bite_leaves_the_command_byte_identical():
    """Same property one level down: a modifier that fires but has nothing to
    do must also not rewrite the command."""
    store = _store()
    driver = ConstantDriver(toy_metadata("driver"), Command(0.1, 0.0, 0.5))
    _tick(Layer(toy_metadata("layer"),
                [driver, SpeedCap(toy_metadata("cap"), limit=0.6)]), store)
    assert store.command is driver.command


def test_cap_is_min_of_magnitude_and_keeps_the_sign():
    store = _store()
    _tick(Layer(toy_metadata("layer"),
                [ConstantDriver(toy_metadata("driver"), Command(-0.8, 0.0, 0.5)),
                 SpeedCap(toy_metadata("cap"), limit=0.25)]), store)
    assert store.command.vx == pytest.approx(-0.25)
    assert store.command.wz == pytest.approx(0.5), "wz was not this node's to touch"
    assert store.command_writers["vx"] == "cap"
    assert store.command_writers["wz"] == "driver"


def test_two_caps_commute():
    """Order-independence between caps is why `cap` exists as its own
    combinator rather than being spelled as an adjust: it removes the composer's
    ordering decision from the result whenever every modifier is a cap."""
    def run(order):
        store = _store()
        _tick(Layer(toy_metadata("layer"),
                    [ConstantDriver(toy_metadata("driver"))] + order), store)
        return store.command.as_tuple()

    a = SpeedCap(toy_metadata("a"), limit=0.5)
    b = SpeedCap(toy_metadata("b"), limit=0.3)
    assert run([a, b]) == run([b, a])
    assert run([a, b])[0] == pytest.approx(0.3)


def test_adjust_is_last_writer_wins():
    class Setter(Action):
        WRITES = ("vx",)
        PRECONDITION = "always"
        EFFECT = "sets vx"

        def __init__(self, metadata, value):
            super().__init__(metadata)
            self.value = value

        def tick(self, obs, ports):
            ports.adjust(vx=self.value)
            return Status.RUNNING

    store = _store()
    _tick(Layer(toy_metadata("layer"),
                [ConstantDriver(toy_metadata("driver")),
                 Setter(toy_metadata("first"), 0.1),
                 Setter(toy_metadata("second"), 0.4)]), store)
    assert store.command.vx == pytest.approx(0.4)
    assert store.command_writers["vx"] == "second"


def test_a_modifier_can_read_what_was_proposed():
    """Point 2 of the decision. Without a readable pending command, 'cap the
    speed and leave the heading alone' is inexpressible even if everybody
    ticks."""
    seen = []

    class Reader(Action):
        WRITES = ("vx",)
        PRECONDITION = "always"
        EFFECT = "reads the pending command"

        def tick(self, obs, ports):
            seen.append(ports.command)
            return Status.RUNNING

    store = _store()
    driver = ConstantDriver(toy_metadata("driver"))
    _tick(Layer(toy_metadata("layer"), [driver, Reader(toy_metadata("r"))]), store)
    assert seen == [driver.command]


def test_the_store_is_cleared_between_ticks_so_a_command_is_never_stale():
    store = _store()
    _tick(ConstantDriver(toy_metadata("driver")), store)
    store.clear()
    assert store.command is None and store.command_writers == {}
