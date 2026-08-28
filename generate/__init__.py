"""
generate/

SIM-FREE. Condition A, B and D's generation path: prompts, the model call, the
staged candidate, and the automatic admission decision.

**NEVER IMPORTS `core.scenarios_held_out`**, and neither may anything it
produces. Chunk 11's AST guard checks this package's own source;
`generate/admit.py`'s import allowlist checks the code this package generates,
which the guard cannot see because it does not exist when the guard runs. G3
measures held-out scenarios never seen during generation, and a generation path
that can read them measures nothing.

    generate/
      prompts/        one file per condition. THE INDEPENDENT VARIABLE.
      backends.py     the model call, and the transcript that makes it replayable
      generator.py    card + contract spec -> a staged candidate
      admit.py        staged candidate -> the harness -> library or rejection
      staging/        every candidate produced, admitted or not, with its reason
      transcripts/    every model call, keyed by prompt hash
"""
