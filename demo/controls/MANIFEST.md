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
| **cut** | **first 813 lines**, a paragraph boundary, ending "…for the lumped mean value of the NIM's TPW reference cells." (§2, mid-way through the apparatus description) |
| truncated text | 32,600 chars, SHA-256 `e666212f…4c85437a`. No "[truncated]" marker: the block's text must stay identical across arms |
| prompt hashes (`sha256[:16]`) | arm0 `c6a5b5e6eac0def5` · armB `4045efb3e514b981` (both **unchanged**, pinned by a test) · armX `2c56254bc6d40d86` |
| prompt chars | arm0 3,687 · armB 45,222 · armX 36,966 |
| keyword scan (`\bterm`, case-insensitive) | path, plan, graph, robot, search, heuristic, obstacle, map, navigat, lidar: **0 of each** in armX's text (D\*'s `paper.txt`: 73, 33, 5, 66, 8, 4, 32, 34, 5, 0) |

⚠️ **The text is not committed** (licence unclear, plan R10). It is regenerated
by `python demo/controls/build_feng2015.py`, which refuses to write anything
whose PDF, full-extraction or truncated hash differs from `manifest.json`.

⚠️ **Residual format difference, disclosed (R7).** Feng is single-column and
double-spaced, so its layout text is blank-line-heavy; D\*'s is two-column and
`-layout` interleaves the columns line by line, so it is gap-heavy (21.8 % of
D\*'s characters are excess whitespace, 10.5 % of Feng's). Feng is not
scrambled to match.

## The token match — an OFFLINE ESTIMATE, not a count

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
  `test_armx_tokens_within_half_a_percent_of_armb` currently passes on the
  **estimate** — it switches to the measured number once
  `claude_input_tokens.armX_measured` is filled in.

## ⚠️ Needs Barna's approval: the calibration call (plan §2.2.4 fallback, R3)

Only a Claude count can close this. In order of preference:

1. **Token-counting endpoint** (`messages.count_tokens`, free, needs an API
   key only Barna handles): count `render("armB")` and `render("armX")` as one
   user message each. The difference is exactly the document difference.
2. **One calibration call through the CLI**, with the **exact** armX prompt
   and the **same flags** as `generate.py`. Its usage gives the exact input
   count. Its transcript goes to `demo/transcripts/calibration/`, it **never**
   yields a `k` file, and it is **excluded from every denominator in advance**
   (write that into `PREREG-v2.md` before the call). ~$1.5 (a full controller
   is generated and discarded). ⚠️ **Under CLI 2.1.302 the pilot's 25,315 is
   not a valid target**: compare against an armB (or arm0 + 12,766) count
   taken under the **same** version, i.e. one armB calibration call as well
   (~$3 total), or compare armX − arm0 against armB − arm0 if arm0 is cheaper
   to re-measure. Record the CLI version beside every count.
3. If the measured count is outside ±0.5 %: re-cut using the measured
   Claude/proxy ratio on Feng (now a **second anchor**, which also identifies
   the whitespace cost), update `cut_lines` and the hashes here, and verify
   with one more count. If no paragraph boundary lands inside ±0.5 %, the
   choice between a **sentence** boundary (`.`-terminated line) and accepting
   the nearest paragraph is Barna's; record it either way.

Then: record the count as `claude_input_tokens.armX_measured`, and, per R3,
check the first real armX transcripts' input tokens against armB's count
**from the same campaign and CLI version** (the fresh armB samples, not the
pilot's 25,315 from 2.1.278) — stop and re-truncate if they deviate by more
than 1 %.

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
```
