---
created: 2026-11-02
---

# armX — the length-matched off-topic control

Mindfire plan §2.2, step 3 of §5. `manifest.json` beside this file is the
machine-readable record and the one the tests read. This file explains it.

## What armX is

The **same prompt block** armB gets (`prompt.md`'s `PAPER BLOCK`, instruction
paragraph included, untouched), holding a published paper on **acoustic gas
thermometry** instead of D\*. So **armB − armX is the paper's content** and
**armX − arm0 is the cost of a long document**.

⚠️ It controls for **length and distraction**, not for "a relevant-looking
paper pulls the model off course" (plan R8). Say so wherever it is reported.

| | |
|---|---|
| source | Feng, Zhang, Lin, Gillis, Moldover (2015), *Determination of the Boltzmann Constant Using the Differential – Cylindrical Procedure*, arXiv:1501.02519 |
| PDF | `Project_B/corpus_pdfs/controls/feng-2015-…-1501.02519.pdf`, 41 pages, SHA-256 `da7a2977…e219cdf`. **Outside the repo**, and stays there |
| extraction | `pdftotext -layout <pdf> -` (poppler 24.02.0) |
| why that command | it reproduces `demo/paper.txt` **byte for byte** from `demo/paper.pdf` (41,121 bytes, SHA-256 `1294dc34…`, checked 2026-11-02). Plain `pdftotext` and `-raw` do not |
| cleaning | `.strip()` only — what `render.paper_text()` does to `paper.txt` |
| full extraction | 71,524 chars, 1,740 lines, SHA-256 `49239138…b3bfa7` |
| **cut** | **first 979 lines** (re-cut 2026-11-02 from the calibration; was 813), a paragraph boundary, ending "…decreasing gas pressures." (§2–3, the apparatus and first measurements) |
| truncated text | 38,609 chars, SHA-256 `7d80ae1f…` (full hash in `manifest.json`; the 813-line cut, `e666212f…`, is kept in `cut_history`). No "[truncated]" marker: the block's text must stay identical across arms |
| prompt hashes (`sha256[:16]`) | arm0 `c6a5b5e6eac0def5` · armB `4045efb3e514b981` (both **unchanged**, pinned by a test) · armX **`a848cebd7806d2cc`** (the 813-line prompt, calibrated and replaced, was `2c56254bc6d40d86`) |
| prompt chars | arm0 3,687 · armB 45,222 · armX 42,975 |
| keyword scan (`\bterm`, case-insensitive) | path, plan, graph, robot, search, heuristic, obstacle, map, navigat, lidar: **0 of each except one `path`** in armX's text ("…through a separate and symmetric path", a gas line) (D\*'s `paper.txt`: 73, 33, 5, 66, 8, 4, 32, 34, 5, 0) |

⚠️ **The text is not committed** (licence unclear, plan R10). It is regenerated
by `python demo/controls/build_feng2015.py`, which refuses to write anything
whose PDF, full-extraction or truncated hash differs from `manifest.json`.

⚠️ **Residual format difference, disclosed (R7).** Feng is single-column and
double-spaced, so its layout text is blank-line-heavy; D\*'s is two-column and
`-layout` interleaves the columns line by line, so it is gap-heavy (21.8 % of
D\*'s characters are excess whitespace, 10.5 % of Feng's). Feng is not
scrambled to match.

## The calibration (2026-11-02) and the one re-cut

Approved by Barna ("yes to 1-4"). Two calls through `calibrate.py`, CLI
**2.1.302**, the exact prompts and the flags `generate.py` uses. ⚠️ **Not
samples**: transcripts in `demo/transcripts/calibration/`, no `k` file, in no
denominator, decided before the calls.

| call | prompt | turns | turn-1 input tokens | all turns | cost |
|---|---|---|---|---|---|
| armB | `4045efb3e514b981` | 1 | **26,464** | 26,464 | $1.80 |
| armX, cut 813 | `2c56254bc6d40d86` | 4 (Bash, Write, Bash) | **28,265** (2 + 20,197 created + 8,066 cached) | 218,664 | $2.01 |

Turn-1 counts come from the per-request usage in the CLI's own session logs;
`--output-format json` sums every turn.

⚠️ **The raw difference (+1,801, +6.8 %) is NOT the prompt difference.** The
armX session loaded the account's **claude.ai MCP connectors** (Claude Docs,
Gmail, …) and the armB session did not: 7,504 extra characters of CLI
overhead (MCP tool names, MCP server instructions, three tool definitions) in
armX's first turn only. Everything else in the two requests is identical. That
overhead is estimated at **~3,300–3,500 Claude tokens** (proxy count × the
Claude/proxy ratio fitted on the armB call's own non-paper text, 1.44–1.51,
which matches the paper block's 1.47–1.48). Corrected, **the 813-line cut was
about −6 % (−5.8 % to −6.3 %) short of armB**, outside ±0.5 %.

**Re-cut once, to 979 lines** (`recut_from_calibration.py`): the measured armX
block is the second anchor, on Feng's own text, and the cut moves to the
paragraph boundary nearest zero.

| cut | kind | armX vs armB, legacy | Qwen | mean |
|---|---|---|---|---|
| 928 | paragraph | −1.75 % | −1.11 % | −1.43 % |
| 973 | sentence | −0.19 % | +0.82 % | +0.31 % |
| **979** | **paragraph** | **−0.01 %** | **+0.99 %** | **+0.49 %** |
| 1001 | paragraph | +1.28 % | +2.49 % | +1.88 % |

**Estimate for 979: +0.5 %, about 26,594 tokens against armB's 26,464, with
proxy spread −0.0 % to +1.0 %.** The 979-line prompt has **not been
measured**, so ±0.5 % is estimated, not demonstrated. The uncertainty is now
about ±1 %, down from −17 %…+10 %, and it comes mainly from the overhead
correction, which a proxy has to estimate. Under the MANIFEST's rule the
paragraph boundary wins, because one lands inside the tolerance. The sentence
cut at 973 is better centred (+0.31 %) and is the alternative if Barna
prefers it.

⚠️ **The connector race was a hazard for the campaign, and it is now
blocked.** Pilot sessions loaded no MCP connectors (checked: 0 of 16 session
logs). Under 2.1.302 one of the two calibration calls did, which gave that call
~3.4k extra tokens plus **Gmail and Docs tools**, and that could differ by arm
and by call. See the next section.

## `--strict-mcp-config` (2026-11-02, Barna's decision) — DISCLOSE IN THE PREPRINT

`generate/backends.py`'s `ClaudeCliBackend` now passes `--strict-mcp-config`
and **no** `--mcp-config`, so no MCP server and no claude.ai connector loads in
any generation or calibration call (`demo/generate.py`, `calibrate.py`, and
every other `ClaudeCliBackend` caller). Checked on CLI 2.1.302: the flag is
accepted, and the cut-979 session log contains **no** MCP content (its
environment block is 11,272 chars, the same as the connector-free armB call's).
The flag is not part of the prompt: the arm0, armB and armX prompt hashes do not
move.

⚠️ **For the write-up:** the 16 pilot calls (CLI 2.1.278) ran **without** this
flag (and loaded no connectors); the campaign runs **with** it. Together with
the CLI version change, this is a difference in apparatus between the pilot and
the confirmatory samples.

## Verification of cut 979 (2026-11-02, one approved call)

| call | prompt | flag | turn-1 input tokens | cost |
|---|---|---|---|---|
| armB (from above) | `4045efb3e514b981` | none, no connectors loaded | **26,464** | $1.80 |
| **armX, cut 979** | **`a848cebd7806d2cc`** | `--strict-mcp-config` | **26,872** | ~$1.19 (estimated) |

**armX − armB = +408 tokens, +1.54 %.** The proxy estimate was +0.5 %, so the
proxies were off by about 1 point, inside the ±1 % stated for them. Barna,
2026-11-02: *"cut point is enough to be close, it doesnt need to match exactly,
just make it approximately same size."* So the tolerance is now **±3 %** (it was
±0.5 %), **cut 979 is kept**, and the test checks the measured count against
it.

⚠️ The call was **interrupted after its first turn**: the calling shell's
10-minute limit killed it while turn 2 was in flight. Turn 1 is the whole
measurement, so nothing was lost, and its usage comes from the CLI's session
log. No json payload or response exists. The cost is turn 1 at the rates the
two earlier payloads imply, plus a small unknown charge for the partial turn 2.
**Calibration total, all three calls: about $5.00.**

## The original offline estimate (superseded by the calibration above)

No token-counting key was available and no paid call was made, so **no Claude
count of the armX prompt exists**. The estimate (`estimate_tokens.py`):

1. **Anchor** on the one exact Claude measurement: the pilot's single-turn CLI
   calls report `input + cache_creation + cache_read` = **12,549** for arm0 and
   **25,315** for armB (±1 between calls; CLI overhead is the same for every
   arm). So the block costs **12,766** Claude tokens.
2. **Transfer** through proxy tokenizers: Claude(armX block) ≈ 12,766 ×
   proxy(armX block) / proxy(armB block).
3. **Cut** at the paragraph boundary minimising the worst error across the
   voting proxies.

| proxy | armX at cut 813 | vs armB |
|---|---|---|
| Anthropic legacy Claude tokenizer (public `tokenizer.json`, pre-Claude-3) | 25,480 | +0.65 % |
| Qwen2.5 | 25,351 | +0.14 % |
| **point estimate (mean of the two)** | **25,415** | **+0.40 %** (tolerance ±0.5 % = ±127) |
| GPT-2 BPE (sensitivity only) | 21,084 | −16.7 % |
| characters (sensitivity only) | 22,777 | −10.0 % |

The neighbouring paragraph boundaries are −3.1 % (751 lines) and +2.3 %
(852 lines) on the voting proxies, so **paragraph granularity is itself
about ±2 %** and only this boundary lands inside the tolerance.

### How uncertain this is

**Much more than ±0.5 %, and the direction is not identified.** The two
voting proxies agree with each other within 0.5 %, but that is agreement
between two whitespace-merging tokenizers, not with Claude:

- Claude's tokenizer spends **1.47×** the legacy tokenizer's count on armB's
  block. A ratio that large says the current tokenizer differs substantially
  from both proxies, and the transfer assumes the ratio is the **same** on
  Feng's text as on D\*'s.
- The texts differ most in **whitespace**, which is where tokenizers differ
  most: the legacy proxy spends 3.4 % of D\*'s tokens on whitespace and 9.6 %
  of Feng's; GPT-2 spends 50 % and 34 %. A tokenizer pricing long space runs
  like GPT-2 puts armX **~17 % short**; one pricing blank lines more heavily
  than the legacy proxy puts it **long**.
- **The CLI has moved since the anchor was taken**: the pilot ran on Claude
  Code **2.1.278**, the campaign will run on **2.1.302**. Both anchors
  (12,549 / 25,315) include the CLI's own overhead (~11.6k tokens of system
  prompt and tool definitions), which may differ between versions. The
  **block** estimate is a difference taken within one version, so the
  overhead cancels there; the **absolute** 25,315 does not carry over. Any
  check against an absolute count must use counts taken **under the same CLI
  version** (see the calibration step below). If the tokenizer itself changed
  with the model serving, the 12,766 anchor moves too, and only a fresh count
  can tell.
- **Stated range: roughly −17 % to +10 %**, point estimate +0.4 %, plus an
  unquantified CLI-version term on any absolute comparison. The ±0.5 %
  target is therefore **not demonstrated**, and
  the test then ran on the estimate. It now runs on the measured count
  (`test_armx_tokens_match_armb_within_tolerance`).

## What is left

- Per R3, check the first real armX transcripts' **turn-1** input tokens
  against the **fresh armB samples from the same campaign and CLI version**
  (both now under `--strict-mcp-config`), not the pilot's 25,315 from 2.1.278
  and not the json totals. Stop and re-truncate if they deviate by more than
  ±3 %, the tolerance now in force.
- ⚠️ Run long calls **in the background**: a foreground shell limit is what cut
  the cut-979 call short.

## Reproduce

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P demo/controls/build_feng2015.py           # regenerate feng2015.txt, verify hashes
$P demo/render.py                            # three-way byte-identity outside the block
$P -m pytest demo/test_demo.py -q            # 12 tests

# The estimate. Needs `tokenizers` (not in env_isaacsim; ~/miniforge3/bin/python has it).
# legacy Claude tokenizer: huggingface.co/Xenova/claude-tokenizer, tokenizer.json,
#   sha256 c241737df24b4e7f7c9af4fdcee29a0ca903dcb288a8b753bc346a3092911767
PYTHONNOUSERSITE=1 ~/miniforge3/bin/python demo/controls/estimate_tokens.py \
    --tokenizer claude-legacy=<claude tokenizer.json> \
    --tokenizer qwen2.5=<Qwen2.5-7B-Instruct tokenizer.json> \
    --tokenizer gpt2=<openai/whisper-tiny tokenizer.json> \
    --vote claude-legacy,qwen2.5

# The calibration (PAID, approved once, 2026-11-02) and the re-cut (free)
$P demo/controls/calibrate.py --arm armB --arm armX --dry-run
PYTHONNOUSERSITE=1 ~/miniforge3/bin/python demo/controls/recut_from_calibration.py \
    --tokenizer claude-legacy=<claude tokenizer.json> \
    --tokenizer qwen2.5=<Qwen2.5-7B-Instruct tokenizer.json>
```
