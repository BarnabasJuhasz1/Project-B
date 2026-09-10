# Fixture transcripts — hand-built, no model was ever called

⚠️ **Nothing in this directory is a record of a model call.** These two files
exist so `generate/tests/test_replay.py` can exercise chunk 42's **two-call
chain** — card call, then node call — with `ReplayBackend` and no network.

`generate/transcripts/` is the real record. This is a test fixture and is kept
separate for exactly that reason: a hand-written response sitting among real
ones would eventually be counted as one.

| file | what it is |
|---|---|
| `88c49cee31e01477.json` | the CARD call for `d / navigate_to_goal`. Response hand-written: one card in the frozen six-field schema, which `cards.store.card_from_dict` — the real loader — accepts. |
| `58ebd5976070bbcf.json` | the NODE call rendered from that card. Response reuses the **committed, admitted** `generate/staging/b__limit_speed_dwa__d6ae93648c7b4f25.py`, so the code is genuinely model-written even though this pairing is not. |

The card's `prediction` was written to match that module's `PREDICTION`, so the
chain admits end to end.

**Both hashes are over the real rendered prompts**, so editing `card_d.md`,
`node.md` or `cardgen.NO_SOURCE` orphans these fixtures and the replay test
fails — which is the intended behaviour, and the same rule the real transcripts
live under. Rebuild them with the snippet in chunk 42's plan file.

The one finding they produced: the no-source sentinel used to contain a colon,
so `citation: (no source: ...)` was a YAML scanner error and every card the
D arm was *told* to write would have been refused by the loader. Found here,
before a single model call was spent on it.
