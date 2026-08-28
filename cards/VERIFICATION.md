# Card source verification

**Status: 4 of 5 cards verified. G1's provenance fraction is 0.80.**
Outstanding: `limit_speed_ics` — see "The one still open" below.

Verified on 2026-08-27 by juhasz.barna12@gmail.com, against PDFs actually on
disk. Spans were extracted mechanically by `scripts/testing/fetch_and_verify.py`
and each page number was checked against the **printed** page, not the PDF page
index. No span in this store was written from memory.

## What each verification found

| Card | Confidence | Outcome |
|---|---|---|
| `limit_speed_dwa` | medium *(unchanged)* | The paper states the closed form **explicitly**, as eq. (14) in "Admissible Velocities": `Va = {(v,w) \| v <= sqrt(2*dist(v,w)*v_b) and w <= sqrt(2*dist(v,w)*w_b)}`. The plan's contingency — "if adapted, drop Confidence to LOW" — **does not apply**. Held at MEDIUM rather than raised to HIGH for one reason: the paper's `dist` is measured along the *corresponding curvature*, the card measures `d` along the *intended heading*. The form is theirs; that one substitution is ours. |
| `reorient_geometry` | low → **high** | The card's whole claim is stated verbatim and quantitatively on p. 158: 78% geometric vs 20% featural, a 3:2 ratio of correct digs to rotational errors, and correct-vs-rotational indistinguishable from 1:1 (`p > .10` in each animal). Geometry in preference to features, with numbers. |
| `path_integration_ants` | medium → **high** | The falsifiable part — that the errors are **systematic**, not random — is explicit in the abstract on p. 5287, and Fig. 2B quantifies the error angle against turning angle and leg-length ratio. |
| `path_integration_mammals` | low → **high** | Better than the plan expected. A review that yields an explicit **threshold**, not just a section: p. 186 gives landmarks predominating over the directional component of PI "as long as both types of information do not diverge by more than 90 deg". |

Three raises, one hold. Every raise is to HIGH under `schema.Confidence`'s
written criterion — *the mechanism **and** its quantitative form are stated
explicitly, and the span points at the equation, threshold or exponent itself.*

## The one still open

`limit_speed_ics` is **unverified, and deliberately so.**

The first PDF obtained for it was *Proc. IEEE-RSJ IROS 2003* — the conference
paper. The card cites *Advanced Robotics* 18(10), 1001–1024 (2004), the journal
version. These are different documents: page numbers differ and wording usually
does. A span quoted from one may not be attributed to the other, and doing it
anyway is exactly the confabulation this file exists to prevent.

**What the conference version already establishes**, to be re-checked against
the journal version once it is in hand:

- The ICS definition, near-verbatim to the card: *"a state for which, no matter
  what the future trajectory followed by the system is, a collision with an
  obstacle eventually occurs."*
- **The occlusion reading is NOT ours.** The card's note says it is, and that
  note is wrong. The paper makes the claim directly: *"The system is subject to
  sensing constraints (a limited field of view)"*, and *"an unsafe situation
  occurs when an unexpected obstacle suddenly appears in the field of view …
  **Safety is therefore closely related to the limited field of view.**"*
  Figs. 7–8 are specifically about the field-of-view boundary. On the plan's own
  instruction — *"confirm the paper does not claim it, or raise Confidence"* —
  this is a raise, and the card's disclaimer needs deleting.

That matters beyond one card: `limit_speed_ics` is the deliberate contrast
against `limit_speed_dwa`, which is the pilot's internal control for "the model
already knew the answer". Strengthening the contrast strengthens the comparison.

## Reproducing this

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P scripts/testing/fetch_and_verify.py --manifest cards/manifest_pilot.csv \
     --out out/corpus/ --skip-download
$P -m pytest cards/tests/ -q
$P -c "from cards.store import CardStore; import json; print(json.dumps(CardStore.load().provenance(), indent=2))"
```

`out/corpus/` is gitignored: it holds third-party PDFs, and the PNAS copy has
the downloader's IP address stamped on every page. **Do not commit it.** The
extractor strips those stamps before cutting sentences, so `spans_candidates.md`
and the card spans are clean, but the PDFs themselves are not.

`cards/manifest_pilot.csv` is committed — it is the record of which document
each span came from, and two of the five rows are marked `manual` because no
fetcher could reach them.

## Citations are starting points

Chunk 14's own plan warns that these citations came from a model and must be
checked against the actual paper. **Author, year, venue and volume may be
wrong.** That warning earned its place: it is what caught the IROS-vs-Advanced
Robotics mismatch above — the provenance axis catching exactly what it was built
to catch, on the first five cards.
