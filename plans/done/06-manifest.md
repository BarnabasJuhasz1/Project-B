# 06 — Manifest

**Epic:** [00](../00-epic-reproducible-episode.md) · **Depends on:** 01 · **Needs Isaac Sim:** no

## Why

"Every reported result names that snapshot" (project description §4). A result without
provenance cannot be re-run, and `scenes/lab42_v1.usd` is an 18 MB binary crate tracked in git
that has already been rewritten once by a rebuild — there is no way to tell from the file whether
it still matches `build_scene.py`. Hashing the bytes actually loaded closes that gap.

## Build

`core/manifest.py`, one `capture(...)` returning a JSON-serialisable dict:

| field | source |
|---|---|
| `git_commit`, `git_dirty` | `git rev-parse HEAD`, `git status --porcelain` |
| `usd_path`, `usd_sha256` | the scene file actually opened |
| `isaacsim_version` | `importlib.metadata.version("isaacsim")` — read the dist, do **not** import it |
| `python_version`, `platform` | stdlib |
| `gpu_name`, `driver_version` | `nvidia-smi --query-gpu=name,driver_version` |
| `container_image` | env var, else `None` when running outside the container |
| `scenario_id`, `seed`, `controller`, `library_snapshot` | arguments; `library_snapshot` is `None` until Week 3 |
| `started_utc`, `sim_seconds`, `wall_seconds` | filled by the runner |

Every external probe degrades to `None` rather than raising: no git repo, no `nvidia-smi`, no
container. The manifest must never be the reason a run fails.

## Done when

`core/tests/test_manifest.py` passes:

- Every key present, and the whole dict survives `json.dumps`.
- `usd_sha256` matches `sha256sum scenes/lab42_v1.usd`.
- With `nvidia-smi` forced to fail, `capture()` still returns and `gpu_name is None`.

## Outcome

Done 2026-08-22. `core/manifest.py`, `core/tests/test_manifest.py`. Whole suite:
**93 passed in 13.8 s**.

Captured on this machine:

```
git_commit  6619ce97…   git_dirty true    git_branch main
usd_sha256  01d083b1e44837d968107964c8c9f5c341372bfefd755d8512145f19452a28b1
isaacsim_version 6.0.1.0   python 3.12.13
gpu_name "NVIDIA GeForce RTX 4060 Laptop GPU"   driver 580.173.02
container_image null   in_container false
```

`usd_sha256` verified against `sha256sum scenes/lab42_v1.usd`.

### Differed from the plan

- **`in_container` added** alongside `container_image`, from `/.dockerenv`. Runs outside the pinned
  image are for authoring only, and this is the single field that says a number must not be
  reported. `container_image` comes from `PROJECT_B_CONTAINER_IMAGE`, which the container
  entrypoint will need to set.
- **`git_dirty` is `None`, not `False`, when there is no repo.** "Clean" would be a lie about a
  state that was never observed, and this field is meant to be the reason someone distrusts a
  result.
- `git_branch` added — free, and useful when a branch is what distinguishes two runs.
- `_isaacsim_version()` falls back from the `isaacsim` dist to `isaacsim-core`.

### Discovered

`test_isaac_version_is_read_not_imported` initially did a substring search for `"import isaacsim"`
and failed on the module's own docstring, which uses those exact words to explain why it does not
do it. Now an AST walk over the actual `Import`/`ImportFrom` nodes. Worth remembering: **the
sim-free guards must parse, not grep**, or comments explaining a rule will trip the rule.

The manifest currently reports `git_dirty: true` — `plans/`, `core/`, `controllers/`,
`scripts/recorder.py` and `scripts/testing/` are all untracked or modified. Nothing has been
committed, per the working agreement. Any number produced before that is committed carries
`git_dirty: true` and is not reportable.
