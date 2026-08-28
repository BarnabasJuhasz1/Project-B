# Corpus brief — find 20–30 papers for Project B

**Hand this whole file to a literature-search agent.** It is written to be read cold by
someone who knows nothing about this project.

---

## 1. What you are being asked to do

Find and download **20–30 open-access PDFs** whose findings can each be turned into a single,
small, testable navigation behaviour for a legged robot in an indoor room.

This is *not* a general reading list on robot navigation. Each paper has to survive a narrow
filter described in §3. A paper that is excellent and unusable here is a miss.

Deliver: the PDFs, plus one metadata row per paper (§7).

## 2. Why, and what the papers are for

The project tests one claim: **does injecting principles extracted from research literature into
an LLM's code-generation step produce better navigation code than the same model produces on its
own?** Four conditions run the same benchmark — a naive model, a literature-informed model, a
human-tuned baseline, and a model told to invent its own principles.

Each paper becomes a **principle card** with six fields:

| Field | What it holds |
|---|---|
| **Source** | citation + a *verbatim span* from the paper, quoted, with a page/section |
| **Mechanism** | what the finding says happens |
| **Trigger** | the condition under which it applies |
| **Derivation** | how it becomes a rule for *this* robot |
| **Prediction** | a falsifiable claim: named metric, named baseline, minimum effect size |
| **Confidence** | high / medium / low |

The card is then handed to a model that writes one behaviour-tree leaf node, and the node is
automatically accepted or rejected by running it in a purpose-built micro-world and checking
whether the Prediction actually held.

**So the binding constraint is the Prediction.** If you cannot imagine a sentence of the form
*"a controller doing X will score better on <metric> than <baseline>, by at least <amount>"*,
the paper is not usable, however good it is.

## 3. The system the paper has to fit

The robot is a Unitree Go2 quadruped in a simulated single indoor room, ~12 m × 8 m, with a
dividing wall, one doorway, and one blind corner. The environment is **static** — no other
agents, no moving obstacles, no outdoors.

### What the robot can sense — the complete list

```
t                    seconds since episode start
ranges               181-beam 2D lidar scan, metres, inf where no return
range_angles         beam bearings in the robot frame, radians
range_max            6.0 m
pose_estimate        (x, y, yaw) — a BELIEF, not ground truth; it drifts
pose_error_estimate  scalar, metres — the robot's own uncertainty estimate
contact              bool — touching something right now
goal                 (x, y) or None — None means "not found yet"; latches once found
```

Field of view is the full 360°. **There is no camera, no colour, no texture, no visual
landmarks, no semantic labels, no map, no odometry beyond the drifting pose estimate, and no
communication.** A mechanism that needs any of those cannot be implemented this cycle.

### What the robot can do — the complete list

One command per control step: `(vx, vy, wz)` — forward, lateral and yaw velocity, capped at
0.8 m/s and 1.0 rad/s. Nothing else. The walking gait is an inherited, frozen, pre-trained
policy that is never modified. **No learning or training of any kind happens in this project**,
so a paper whose contribution is a trained policy is out unless the *principle behind it* can be
written as an explicit rule.

### Metrics a Prediction may name

`goal_reached`, `time_to_goal`, `spl` (path efficiency), `path_length`, `collisions`,
`near_misses`, `stuck_events`, `recovery_time`, `time_stuck`, `min_clearance`, `duration`.

Those are all of them. There is no metric for legibility, comfort, social acceptability,
energy, human preference, or map quality.

## 4. The filter — usable vs. not

**A paper is usable if it states a mechanism that:**

1. is triggered by something in the sensor list above,
2. changes what velocity command gets sent,
3. would plausibly move one of the metrics in §3, and
4. is stated concretely enough to quote — a threshold, an exponent, a rule, a measured
   relationship, a decision criterion. **Not** a diagram, a claim of general importance, or
   an architecture.

**Prefer** primary research with a specific quantitative or procedural result.
**Accept** a review only when it states an arbitration rule or a specific finding you can
quote and attribute.
**Reject** surveys and position papers whose contribution is taxonomy.

### The interesting-vs-obvious screen

Rate each paper on one axis: **would a frontier LLM produce this mechanism if simply asked to
write good navigation code?**

- Mostly find papers where the answer is **no** — that is where the whole experiment lives.
- Deliberately include **2–3 where the answer is yes** (e.g. classic reactive obstacle
  avoidance). These are the internal controls: they measure what a card is worth when the model
  already knew the answer.

Say which each one is, in the metadata row. Do not silently filter the obvious ones out.

## 5. Coverage — what to look for, and roughly how many

Aim for breadth across these buckets rather than depth in one. Animal-navigation and
robotics literature are both in scope, and mixing them is desirable.

| # | Bucket | Target | Examples of what counts |
|---|---|---|---|
| 1 | **Path integration / homing** | 4–5 | dead-reckoning home vectors, systematic vs random error accumulation, when the home vector is trusted or abandoned |
| 2 | **Geometric reorientation** | 3–4 | recovering heading from room shape and boundaries rather than features; boundary-based place recognition |
| 3 | **Search under uncertainty** | 4–5 | coverage strategies, correlated / Lévy walks, systematic sweeps, frontier selection, when to switch search modes |
| 4 | **Speed regulation under limited visibility** | 3–4 | braking-distance admissibility, inevitable collision states, velocity obstacles, speed limits from sensor horizon or occlusion |
| 5 | **Cue arbitration and confidence weighting** | 3–4 | when to trust dead reckoning vs. current sensing, uncertainty-gated switching, degraded-localisation behaviour |
| 6 | **Boundary use in novel space** | 2–3 | thigmotaxis, wall-following as an exploration strategy, edge preference in unfamiliar environments |
| 7 | **Escaping stuck states** | 2–3 | local-minimum escape, oscillation detection and damping, recovery behaviours |
| 8 | **Recent work (2024–2026)** | 3–5 | anything in buckets 1–7 published recently; see §6 |

## 6. Recency matters, specifically

One of the three things a corpus is supposed to supply that a model cannot is **novelty:
findings published after the model's training cutoff.** So papers from **2024 onward** are
disproportionately valuable, even where an older paper covers similar ground.

Please make bucket 8 a genuine sweep of recent venues, not an afterthought — recent
bio-inspired navigation, recent classical/geometric navigation, and recent work on navigation
under localisation uncertainty. Note the publication date prominently in every row.

## 7. What to deliver

A folder of PDFs, and a table (CSV or markdown) with one row per paper:

| Column | Notes |
|---|---|
| `filename` | the PDF as saved |
| `citation` | full, with year, venue, DOI |
| `year` | |
| `bucket` | 1–8 from §5 |
| `mechanism` | one or two sentences: what the paper says happens |
| `trigger` | one sentence: when it applies |
| `quotable_span` | a **verbatim** sentence or two from the paper stating the mechanism, with page or section. This is the single most important column — it is what makes the card traceable rather than remembered |
| `metric` | which metric from §3 you would predict it moves |
| `obvious?` | `yes` / `no` — the §4 screen |
| `confidence` | high / medium / low: how directly the paper supports the mechanism as stated |
| `access` | open access / preprint / author copy / **paywalled — not downloaded** |

**Only download what is legitimately available**: open access, arXiv/bioRxiv preprints, author
copies, or institutional repositories. If a paper is essential but paywalled, list it in the
table with `access: paywalled` and no PDF; do not attempt to bypass anything.

## 8. Explicitly out of scope — do not spend effort here

Each of these was considered and ruled out for a stated reason. Finding papers on them is
wasted work.

- **Legibility / intent-signalling motion.** The dependent variable is an *observer's* belief,
  and no metric in §3 is a property of anyone's belief. A legible path is also longer than the
  optimal one, so it scores negatively on path efficiency.
- **Area-restricted search.** ARS concentrates search around an encounter *rate*; this task has
  a single goal detected once, so there is no rate and no gradient to follow. (Papers on *other*
  search strategies — bucket 3 — are very much wanted.)
- **Multi-sensor cross-validation / sensor agreement.** There is one sensing modality; there is
  nothing to cross-check against.
- **SLAM, scan matching, loop closure, pose-graph optimisation.** Genuinely relevant to a later
  phase of this project, but deliberately scheduled *after* the current evaluation and must not
  be mixed into this batch. Skip entirely.
- **Vision, VO, semantic navigation, learned perception.** Not in the sensor contract.
- **Multi-agent, human-robot interaction, crowd navigation, dynamic obstacles.**
- **Outdoor, rough-terrain, and gait/locomotion control.** The gait is frozen and inherited.
- **Anything requiring training a policy.**

## 9. One thing to be careful about

Do not write a span from memory. An unverified or reconstructed quote is worse than no quote,
because provenance is one of the three things the project is measuring — the point is precisely
to distinguish a mechanism that is *in a document* from one a model stated fluently. If you
cannot open the paper and copy the sentence, mark the row `access: paywalled` and move on.
