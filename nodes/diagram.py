"""
nodes/diagram.py

The behaviour tree, drawn, with each node coloured by what it did on the step
being shown.

WHY THIS EXISTS
---------------
`nodes/trace.py` already answers "which node drove, and when" — but it answers
it as a table and an ASCII strip, after the fact. Watching a run, the question
is the same one in the present tense: the robot just turned left, *which node
decided that*, and what was the rest of the tree doing at the time. A composed
tree is the deliverable of this project's composition step, and a reviewer who
cannot see it working is reviewing a JSON plan.

So this renders one frame of the tree per video frame, and `scripts/recorder.py`
composites it above the top-down and chase panes.

SIM-FREE, AND A PURE FUNCTION OF THE TICK LOG
----------------------------------------------
Nothing here imports isaacsim, omni, pxr or torch — `core/tests/test_import_guard.py`
enforces that for `nodes/`, and this module is the reason to say so out loud: it
is a *rendering* module, which is exactly the kind of thing that reaches for a
simulator. It does not. It takes a `Tree` and a `TickLog` and returns a numpy
array, which is why it can be unit tested with no GPU.

It is also strictly READ-ONLY on the log. Drawing must never be able to change
what the trace says, for the same reason `Tree.enable_trace()` is inert: a
recorded run and a headless run are the same run, and the video is not evidence.

LAYOUT
------
Conventional top-down: root at the top, depth increasing downward, leaves spread
across the width. The banner is wide and short (the composite is ~2560 px across
and a few hundred tall), so the horizontal axis is the one with room in it and
leaf labels are what need the room.
"""

import numpy as np

# ---- below this line is data, not code -------------------------------------

# Dark, because it sits above two rendered panes and a light banner would be the
# brightest thing in the frame — the eye would go to the diagram instead of the
# robot. Tuned against the lab42 top-down pane, which is mid-grey concrete.
BACKGROUND = (22, 23, 28)
TITLE_COLOR = (168, 172, 184)
SUBTITLE_COLOR = (108, 112, 124)
EDGE_COLOR = (62, 66, 78)
EDGE_ACTIVE_COLOR = (128, 136, 156)

# One colour per state, and the states are the four distinctions the tick log
# actually draws. `drove` is the loudest because "who is driving the robot right
# now" is the question the pane exists to answer; a node that ticked and was
# gated out is dim but still legible, because a precondition that never fires is
# a finding and an invisible node would hide it.
STATE_COLORS = {
    #  state    fill              border            text
    "drove":   ((236, 158, 46),  (255, 196, 110),  (24, 20, 12)),
    "active":  ((44, 92, 148),   (94, 150, 214),   (226, 236, 248)),
    # Ticked, precondition passed, and returned FAILURE. Distinct from `gated`,
    # which is a node the tree declined to run at all. The difference is the
    # whole diagnosis on a search task: a Condition that fails every step is why
    # the branch behind it is dark, and drawing it the same blue as a Condition
    # that passes would hide the one fact the pane is there to show.
    "failed":  ((92, 46, 52),    (152, 78, 84),    (232, 190, 194)),
    "gated":   ((58, 52, 60),    (92, 84, 96),     (156, 148, 158)),
    "idle":    ((32, 34, 40),    (52, 55, 64),     (104, 108, 118)),
    # A COMPOSITE on the driving path. The tick log marks it `drove`, and that
    # is not wrong — a Sequence whose child drove did preside over the Twist —
    # but filling it amber puts three loud boxes on screen when exactly one node
    # emitted the command. Only an Action may drive (nodes/contract.py: "the
    # Twist is NOT a port"), so the amber FILL is reserved for the leaf that
    # actually did, and its ancestors get the amber BORDER instead. Same fact,
    # drawn so the eye lands on the answer.
    "path":    ((44, 92, 148),   (236, 158, 46),   (240, 224, 196)),
}
STATE_LABELS = [("drove", "drove"), ("path", "on the driving path"),
                ("active", "ticked"), ("failed", "returned failure"),
                ("gated", "gated out"), ("idle", "not ticked")]

# Geometry. All in pixels at the banner's native size; the banner is not scaled.
TITLE_HEIGHT = 34
ROW_HEIGHT = 74            # one tree depth level
BOX_HEIGHT = 40
BOX_MIN_WIDTH = 90
# Capped, and the cap is the point. Leaves are given equal slots across a
# ~2560 px banner, so a three-leaf tree would otherwise draw three 800 px
# lozenges and read as a colour chart rather than as a tree. Boxes stay a
# label's width and the SPACE between them carries the structure.
BOX_MAX_WIDTH = 340
BOX_GAP = 14               # horizontal breathing room between sibling boxes
BOX_RADIUS = 6
MARGIN_X = 18
MARGIN_BOTTOM = 12
LEGEND_SWATCH = 11

# A composite carries no useful id (`composed_sequence_0`), so it is drawn by
# TYPE — Sequence and Fallback tick their children differently and that is the
# thing a reader needs off the diagram. Leaves are drawn by node_id, which is
# the vocabulary the composition plan was written in.
COMPOSITE_LABELS = {"Sequence": "→ Sequence", "Fallback": "? Fallback"}


def _font(size):
    """DejaVu at `size`, or PIL's bitmap default if it cannot be found.

    matplotlib ships DejaVuSans and is already a dependency of the env; PIL's
    own default is a 10 px bitmap that is unreadable next to a 720 p render.
    Falling back rather than raising, because a diagram with ugly text is worth
    more than a recording that refuses to start.
    """
    from PIL import ImageFont
    try:
        import matplotlib
        import os
        path = os.path.join(matplotlib.get_data_path(), "fonts", "ttf",
                            "DejaVuSans.ttf")
        return ImageFont.truetype(path, size)
    except Exception:                                    # noqa: BLE001
        return ImageFont.load_default()


def _elide(draw, text, font, max_width):
    """Trim from the RIGHT with an ellipsis. Node ids share long prefixes
    (`toy_`, `composed_`, `gen_b_`), so the tail is where they differ — but the
    head is what a reader scans, and a column of `…_goal` is worse than a column
    of `toy_avoid…`."""
    if draw.textlength(text, font=font) <= max_width:
        return text
    while text and draw.textlength(text + "…", font=font) > max_width:
        text = text[:-1]
    return text + "…" if text else ""


class Node2D:
    """One node's box, in banner pixel coordinates."""

    __slots__ = ("node_id", "label", "depth", "cx", "y0", "x0", "x1", "parent")

    def __init__(self, node_id, label, depth, parent):
        self.node_id = node_id
        self.label = label
        self.depth = depth
        self.parent = parent
        self.cx = 0.0
        self.x0 = self.x1 = 0.0
        self.y0 = 0

    @property
    def y1(self):
        return self.y0 + BOX_HEIGHT


def _walk(node, depth, parent, out):
    """Flatten in tick order, which is also left-to-right draw order.

    Deliberately mirrors `nodes.tree.flatten` rather than calling it: this needs
    the depth and the parent link, and duplicating six lines is cheaper than
    changing a function the wiring check depends on.
    """
    children = getattr(node, "children", ())
    label = (COMPOSITE_LABELS.get(type(node).__name__, type(node).__name__)
             if children else node.metadata.node_id)
    box = Node2D(node.metadata.node_id, label, depth, parent)
    out.append(box)
    for child in children:
        _walk(child, depth + 1, box, out)
    return box


def banner_height_for(tree):
    """The height a TreeDiagram of this tree will need, WITHOUT laying it out.

    `scripts/run_episode.py` has to tell the Recorder how tall the banner is
    before it can ask the Recorder how wide the banner is — the frame is only
    that tall because there is a banner. Depth is the only input to the height,
    so it can be answered from the tree alone and the circularity goes away.
    """
    boxes = []
    _walk(tree.root, 0, None, boxes)
    depth = max(b.depth for b in boxes) + 1
    return TITLE_HEIGHT + depth * ROW_HEIGHT + MARGIN_BOTTOM


class TreeDiagram:
    """A `Tree`, laid out once, rendered once per frame.

    The layout is computed in the constructor because it never changes: the tree
    is fixed for the episode and only the colours move. Re-solving it per frame
    would put a layout pass inside the render loop for no reason.

        panel = TreeDiagram(tree, width=2562)
        rgb = panel.render(tick_log, t=4.5)      # HxWx3 uint8
    """

    def __init__(self, tree, width, height=None, title=None, subtitle=None):
        self.width = int(width)
        self.tree = tree
        self.boxes = []
        _walk(tree.root, 0, None, self.boxes)
        self.depth = max(b.depth for b in self.boxes) + 1
        self.height = int(height) if height else (
            TITLE_HEIGHT + self.depth * ROW_HEIGHT + MARGIN_BOTTOM)
        leaves = sum(1 for b in self.boxes if not self._children_of(b))
        self.title = title or f"behaviour tree — {leaves} leaves"
        self.subtitle = subtitle or (tree.snapshot or "")
        self._layout()
        self._font = _font(15)
        self._small = _font(12)
        self._title_font = _font(14)

    # -- layout --------------------------------------------------------------

    def _layout(self):
        """Leaves get equal slots across the width; a parent centres over the
        span of its children. The classic tidy-tree placement, and enough for
        the shapes composition produces — plans are a handful of nodes three or
        four deep, not a hundred-node tree needing Reingold-Tilford."""
        leaves = [b for b in self.boxes if not self._children_of(b)]
        usable = self.width - 2 * MARGIN_X
        slot = usable / max(1, len(leaves))
        for i, box in enumerate(leaves):
            box.cx = MARGIN_X + slot * (i + 0.5)

        # Parents after children, so a parent always sees placed children.
        for box in reversed(self.boxes):
            kids = self._children_of(box)
            if kids:
                box.cx = sum(k.cx for k in kids) / len(kids)

        half = min(BOX_MAX_WIDTH, max(BOX_MIN_WIDTH, slot - BOX_GAP)) / 2.0
        for box in self.boxes:
            # Nudge rather than clip: a box pushed off the edge by the centring
            # is moved back inside whole, because a half-drawn node reads as a
            # rendering bug and this pane exists to be trusted at a glance.
            cx = min(max(box.cx, half + 2.0), self.width - half - 2.0)
            box.x0, box.x1 = cx - half, cx + half
            box.y0 = TITLE_HEIGHT + box.depth * ROW_HEIGHT + \
                (ROW_HEIGHT - BOX_HEIGHT) // 2

    def _children_of(self, box):
        return [b for b in self.boxes if b.parent is box]

    # -- state ---------------------------------------------------------------

    @staticmethod
    def states_from(tick_log):
        """{node_id: state} for the log's MOST RECENT step.

        Returns {} for a log with nothing in it, which is the honest answer
        before the first tick and keeps the first frames from claiming activity
        the tree has not had yet.
        """
        if tick_log is None or not tick_log.rows:
            return {}
        step = tick_log.rows[-1][0]
        states = {}
        for row in tick_log.rows:
            if row[0] != step:
                continue
            _, _, _, node_id, _, pre, status, drove = row
            if drove:
                states[node_id] = "drove"
            elif not pre:
                states[node_id] = "gated"
            else:
                # FAILURE is called out; SUCCESS and RUNNING share `active`,
                # because a node that is working and a node that has finished
                # working are both a node doing its job, and the pane already
                # says who is driving.
                states[node_id] = "failed" if status == "failure" else "active"
        return states

    # -- rendering -----------------------------------------------------------

    def render(self, tick_log=None, t=None, states=None):
        """One frame, HxWx3 uint8. Never mutates the log."""
        from PIL import Image, ImageDraw

        if states is None:
            states = self.states_from(tick_log)
        image = Image.new("RGB", (self.width, self.height), BACKGROUND)
        draw = ImageDraw.Draw(image)

        # Edges first, so boxes sit on top of them.
        for box in self.boxes:
            if box.parent is None:
                continue
            live = states.get(box.node_id, "idle") != "idle"
            draw.line([(box.parent.cx, box.parent.y1),
                       (box.parent.cx, box.y0 - 12),
                       (box.cx, box.y0 - 12), (box.cx, box.y0)],
                      fill=EDGE_ACTIVE_COLOR if live else EDGE_COLOR,
                      width=2 if live else 1)

        for box in self.boxes:
            state = states.get(box.node_id, "idle")
            # Composites report `drove` when a descendant did; see STATE_COLORS.
            if state == "drove" and self._children_of(box):
                state = "path"
            fill, border, text_color = STATE_COLORS[state]
            draw.rounded_rectangle([box.x0, box.y0, box.x1, box.y1],
                                   radius=BOX_RADIUS, fill=fill,
                                   outline=border, width=2)
            label = _elide(draw, box.label, self._font, (box.x1 - box.x0) - 14)
            draw.text(((box.x0 + box.x1) / 2, (box.y0 + box.y1) / 2), label,
                      font=self._font, fill=text_color, anchor="mm")

        self._draw_title(draw, t)
        self._draw_legend(draw)
        return np.asarray(image, dtype=np.uint8)

    def _draw_title(self, draw, t):
        clock = f"t = {t:6.2f} s" if t is not None else ""
        draw.text((MARGIN_X, TITLE_HEIGHT / 2), self.title,
                  font=self._title_font, fill=TITLE_COLOR, anchor="lm")
        head = draw.textlength(self.title, font=self._title_font)
        if self.subtitle:
            draw.text((MARGIN_X + head + 14, TITLE_HEIGHT / 2), self.subtitle,
                      font=self._small, fill=SUBTITLE_COLOR, anchor="lm")
        if clock:
            # Monospaced-ish by padding rather than by font: the clock is the
            # only thing on the banner that changes width every frame, and a
            # jittering readout is distracting to watch.
            draw.text((self.width - MARGIN_X, TITLE_HEIGHT / 2), clock,
                      font=self._title_font, fill=TITLE_COLOR, anchor="rm")

    def _draw_legend(self, draw):
        y = self.height - MARGIN_BOTTOM / 2 - LEGEND_SWATCH / 2
        x = MARGIN_X
        for state, caption in STATE_LABELS:
            fill, border, _ = STATE_COLORS[state]
            draw.rounded_rectangle([x, y, x + LEGEND_SWATCH, y + LEGEND_SWATCH],
                                   radius=2, fill=fill, outline=border)
            x += LEGEND_SWATCH + 5
            draw.text((x, y + LEGEND_SWATCH / 2), caption, font=self._small,
                      fill=SUBTITLE_COLOR, anchor="lm")
            x += draw.textlength(caption, font=self._small) + 18
