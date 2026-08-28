"""
generate/backends.py

Where the model call happens, and the only place in the pipeline that is not a
pure function.

Three backends, and the reason there are three is that they answer three
different questions:

  ClaudeCliBackend   what runs on this machine today
  AnthropicBackend   what a container run would use. NOT EXERCISED — see below.
  ReplayBackend      what makes a library regenerable without a model at all

**THE DETERMINISM STORY CHANGED, AND IT IS WORTH READING BEFORE TRUSTING IT.**

Chunk 15's plan asks that "the same card, same prompt, same model id and
temperature 0 produces a byte-identical candidate, or the metadata explains why
not". This is the metadata explaining why not.

`temperature` was REMOVED from the Messages API on Claude Opus 5, Fable 5,
Sonnet 5, Opus 4.8 and Opus 4.7 — passing it is a 400, not a no-op. Only older
models (Opus 4.6, Sonnet 4.5, Haiku 4.5) still accept a sampling temperature.
So temperature-0 determinism is not something this project can have on a current
model, and pinning generation to an older one to buy it would be trading the
generator's capability for a reproducibility property we can get another way.

The other way is the transcript. **Every call writes one**, keyed by the hash of
the exact prompt text, recording the model id, the backend, the full response and
the sampling parameters actually in force. `ReplayBackend` reads those back, so:

- the library is regenerable byte-for-byte, from committed files, with no model,
  no key and no network — which is the reproducibility requirement in §6, and a
  stronger one than temperature 0 would have given us; and
- what the model actually returned is auditable after the fact, which matters
  more here than usual, since the artefact under study IS the model's output.

Generation is therefore reproducible; it is not *repeatable*. Re-running a
prompt against a live model may produce a different node, and the transcripts are
what let anyone tell the two apart. Chunk 17 has to state which one the reported
numbers rest on.

**WHY THE CLI BACKEND IS NOT A HACK.** `claude -p` is a real call to a real
model with no API key, and it starts a fresh session per call — so conditions A,
B and D cannot leak into one another through a shared context, which is the same
class of contamination as chunk 11's seal and is not otherwise structurally
prevented. It is run with `--setting-sources ''` from a scratch directory, so
this repo's CLAUDE.md is NOT in the model's context: a generation call that could
read the project's own description of its four conditions would be measuring the
description rather than the corpus. Verified: the probe asked what project it was
looking at and got "NO PROJECT CONTEXT".

What it does still carry is Claude Code's own tool definitions, which cannot be
switched off from the command line. That is a deviation worth stating and not
worth fixing: it is **identical across A, B and D**, so it cannot differentiate
the conditions, and B−A and B−D are differences.

**EXECUTING WHAT COMES BACK.** The response is model-written Python and
`generate/admit.py` executes it. There is no sandbox on this machine. The import
allowlist there is the only barrier, it is a static AST check, and it is a real
barrier against accident rather than a barrier against intent. Said plainly here
because the pipeline's whole job is to run generated code.
"""

import hashlib
import json
import os
import subprocess
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
TRANSCRIPTS = Path(__file__).resolve().parent / "transcripts"

# ---------------------------------------------------------------------------
# Everything above this line is code; below it is data.
# ---------------------------------------------------------------------------

# The generation model. One place, so that a library's model id is a fact about
# the run and not about whichever default was in force that day.
DEFAULT_MODEL = "claude-opus-5"

# Wall clock for one generation call. A node plus a micro-world is a few hundred
# lines of thinking; two minutes is not enough and ten is a hang.
CLI_TIMEOUT_S = 600

# Models that still accept a sampling temperature. Everything else rejects the
# parameter outright — see the module docstring.
TEMPERATURE_MODELS = frozenset({
    "claude-opus-4-6", "claude-sonnet-4-6",
    "claude-sonnet-4-5", "claude-haiku-4-5",
})


class BackendError(RuntimeError):
    """The model could not be reached, or returned nothing usable. Distinct from
    a REJECTED candidate: a rejection is a result, this is a failure to get
    one."""


def prompt_hash(text):
    """Identity of a prompt, and the key a transcript is filed under.

    The full rendered text, card substituted and all — not the template. Two
    cards through the same template are two different prompts, and a library
    that could not tell them apart could not attribute a node to a card.
    """
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]


class Response:
    """One model call: what went in, what came out, and under what sampling."""

    def __init__(self, text, model_id, backend, temperature=None,
                 sampling_note="", raw=None):
        self.text = text
        self.model_id = model_id
        self.backend = backend
        self.temperature = temperature
        self.sampling_note = sampling_note
        self.raw = raw or {}

    def as_dict(self):
        return {
            "text": self.text,
            "model_id": self.model_id,
            "backend": self.backend,
            "temperature": self.temperature,
            "sampling_note": self.sampling_note,
            "raw": self.raw,
        }


def _sampling(model_id, temperature):
    """What sampling is actually in force, and the honest note about it.

    Returned rather than assumed, because `temperature=None` in node metadata is
    ambiguous on its own — it could mean "the default" or "nobody recorded it",
    and G1's regenerability claim turns on which.
    """
    if model_id in TEMPERATURE_MODELS:
        return temperature, ""
    return None, (
        f"{model_id} does not accept a temperature parameter (removed from the "
        f"Messages API on this model family); sampling is the model default and "
        f"generation is reproducible from the transcript, not repeatable from "
        f"the prompt")


class Backend:
    """What the generator needs of a model. Two methods, deliberately: the
    generator must be able to record WHICH backend produced a node without
    inspecting its type."""

    name = "backend"

    def complete(self, prompt):
        raise NotImplementedError

    def describe(self):
        return {"backend": self.name}


class ClaudeCliBackend(Backend):
    """Shell out to the `claude` CLI. Works on this machine with no API key.

    Run from a scratch directory with `--setting-sources ''` so that no CLAUDE.md
    and no project settings reach the model — see the module docstring for why
    that matters more here than it looks.
    """

    name = "claude-cli"

    def __init__(self, model_id=DEFAULT_MODEL, timeout=CLI_TIMEOUT_S,
                 executable="claude"):
        self.model_id = model_id
        self.timeout = timeout
        self.executable = executable

    def complete(self, prompt):
        # A scratch cwd, thrown away after the call: CLAUDE.md discovery walks
        # UP from the working directory, so running in-tree would put this
        # project's own description of its four conditions into the context of a
        # call whose whole purpose is to not have seen it.
        with tempfile.TemporaryDirectory(prefix="gen-clean-") as clean:
            # The prompt goes on stdin, not argv. It is ~12 KB and grows with
            # the card; argv is a blunter instrument than it looks once a
            # prompt carries quotes, backticks and a YAML block.
            argv = [
                self.executable, "-p",
                "--model", self.model_id,
                "--setting-sources", "",
                "--system-prompt", SYSTEM_PROMPT,
                "--output-format", "json",
            ]
            try:
                proc = subprocess.run(argv, cwd=clean, input=prompt,
                                      capture_output=True, text=True,
                                      timeout=self.timeout)
            except FileNotFoundError as exc:
                raise BackendError(
                    f"{self.executable!r} is not on PATH. The CLI backend is "
                    f"what runs without an API key; install it or use "
                    f"ReplayBackend against a committed transcript.") from exc
            except subprocess.TimeoutExpired as exc:
                raise BackendError(
                    f"generation timed out after {self.timeout}s") from exc

        if proc.returncode != 0:
            # Both streams: the CLI reports some failures on stdout and leaves
            # stderr empty, and a BackendError that says nothing is a debugging
            # session rather than a message.
            raise BackendError(
                f"claude exited {proc.returncode}\n"
                f"  stderr: {proc.stderr.strip()[:600] or '(empty)'}\n"
                f"  stdout: {proc.stdout.strip()[:600] or '(empty)'}")
        try:
            payload = json.loads(proc.stdout)
        except json.JSONDecodeError as exc:
            raise BackendError(
                f"claude did not return JSON: {proc.stdout[:400]!r}") from exc
        if payload.get("is_error"):
            raise BackendError(f"claude reported an error: {payload!r}")

        text = payload.get("result") or ""
        if not text.strip():
            raise BackendError("the model returned an empty response")

        temperature, note = _sampling(self.model_id, None)
        return Response(
            text=text, model_id=self.model_id, backend=self.name,
            temperature=temperature,
            sampling_note=note or "sampling not exposed by the CLI",
            # Cost and token counts, kept because a per-condition cost is
            # something the write-up will be asked for and cannot be
            # reconstructed later.
            raw={"usage": payload.get("usage"),
                 "total_cost_usd": payload.get("total_cost_usd"),
                 "session_id": payload.get("session_id")},
        )

    def describe(self):
        return {"backend": self.name, "model_id": self.model_id}


class AnthropicBackend(Backend):
    """The Anthropic Python SDK. **NOT EXERCISED — no SDK and no key here.**

    Written because Week 5's reported numbers come from inside
    `nvcr.io/nvidia/isaac-sim:6.0.1`, where the `claude` CLI is not installed and
    an API key is the only way to reach a model. Kept small for that reason: the
    first run of it will find whatever is wrong with it, and there is less to be
    wrong with.

        ~/miniforge3/envs/env_isaacsim/bin/pip install anthropic
        export ANTHROPIC_API_KEY=...

    `temperature` is passed ONLY for the models that still accept it; on Opus 5
    and its family the parameter is a 400. See the module docstring.
    """

    name = "anthropic-sdk"

    def __init__(self, model_id=DEFAULT_MODEL, temperature=0.0,
                 max_tokens=16000):
        self.model_id = model_id
        self.temperature = temperature
        self.max_tokens = max_tokens

    def complete(self, prompt):
        try:
            import anthropic
        except ImportError as exc:
            raise BackendError(
                "the anthropic SDK is not installed in this interpreter. "
                "`pip install anthropic`, or use ClaudeCliBackend, or replay a "
                "committed transcript.") from exc

        temperature, note = _sampling(self.model_id, self.temperature)
        kwargs = dict(
            model=self.model_id,
            max_tokens=self.max_tokens,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user", "content": prompt}],
        )
        if temperature is not None:
            kwargs["temperature"] = temperature

        client = anthropic.Anthropic()
        try:
            message = client.messages.create(**kwargs)
        except Exception as exc:                 # noqa: BLE001 — reported, not swallowed
            raise BackendError(f"the Messages API call failed: {exc}") from exc

        # A refusal is an HTTP 200 with no usable content, so it has to be
        # checked before the content blocks are read.
        if getattr(message, "stop_reason", None) == "refusal":
            raise BackendError(
                f"the model declined to answer: "
                f"{getattr(message, 'stop_details', None)!r}")

        text = "".join(block.text for block in message.content
                       if getattr(block, "type", None) == "text")
        if not text.strip():
            raise BackendError("the model returned no text blocks")

        return Response(
            text=text, model_id=self.model_id, backend=self.name,
            temperature=temperature, sampling_note=note,
            raw={"stop_reason": getattr(message, "stop_reason", None),
                 "usage": getattr(message, "usage", None)
                 and message.usage.model_dump()},
        )

    def describe(self):
        return {"backend": self.name, "model_id": self.model_id,
                "temperature": self.temperature}


class ReplayBackend(Backend):
    """Return a committed transcript instead of calling a model.

    This is what makes a library regenerable, and it is the backend every test
    uses: a test that called a model would be slow, costly, non-deterministic and
    dependent on a network, and would be testing the model rather than the
    pipeline.
    """

    name = "replay"

    def __init__(self, directory=None):
        self.directory = Path(directory or TRANSCRIPTS)

    def path_for(self, prompt):
        return self.directory / f"{prompt_hash(prompt)}.json"

    def complete(self, prompt):
        path = self.path_for(prompt)
        if not path.exists():
            raise BackendError(
                f"no transcript for prompt {prompt_hash(prompt)} in "
                f"{self.directory}. Replay reproduces a recorded generation; it "
                f"cannot invent one. Run once against a live backend first, or "
                f"check whether the prompt text drifted — the hash is over the "
                f"rendered prompt, so editing a prompt file orphans every "
                f"transcript taken against it.")
        data = json.loads(path.read_text(encoding="utf-8"))
        response = data["response"]
        return Response(
            text=response["text"], model_id=response["model_id"],
            backend=response["backend"], temperature=response["temperature"],
            sampling_note=response.get("sampling_note", ""),
            raw=response.get("raw"),
        )

    def describe(self):
        return {"backend": self.name, "directory": str(self.directory)}


def write_transcript(prompt, response, meta, directory=None):
    """Record one generation call. Filed under the prompt hash.

    The prompt text is stored in full alongside the response. It costs a few KB
    and it is the difference between a transcript you can audit and a hash you
    have to trust.
    """
    directory = Path(directory or TRANSCRIPTS)
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{prompt_hash(prompt)}.json"
    payload = {
        "prompt_hash": prompt_hash(prompt),
        "prompt": prompt,
        "response": response.as_dict(),
        "meta": meta,
    }
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
    return path


def make(name, model_id=DEFAULT_MODEL, **kwargs):
    """Resolve a backend by name, the way `controllers.make` resolves one."""
    if name == "claude-cli":
        return ClaudeCliBackend(model_id=model_id, **kwargs)
    if name == "anthropic-sdk":
        return AnthropicBackend(model_id=model_id, **kwargs)
    if name == "replay":
        return ReplayBackend(**kwargs)
    raise KeyError(f"no backend {name!r}. Known: claude-cli, anthropic-sdk, replay")


# The system prompt is deliberately almost empty. Everything that steers the
# model is in generate/prompts/, where it is versioned and where the diff between
# conditions is visible; a rule hidden here would be an uncontrolled difference
# between conditions if it were ever varied, and invisible in the diff if it were
# not.
SYSTEM_PROMPT = (
    "You are writing Python for a robotics research codebase. Follow the "
    "specification you are given exactly. Return the code in a single fenced "
    "python block."
)
