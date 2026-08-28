"""
compose/

⚠️ **READ `compose/FINDINGS.md` FIRST.** Five findings, two of which bind chunk
17: a composition can succeed and produce a tree that does not work, and the
admitted condition-B node has two effects the harness could not detect.

Task specification + retrieved port contracts -> a behaviour tree.

**NON-NEGOTIABLE #6: composition sees only port contracts and a task
specification. Never the papers.** That separation is what makes G3 a distinct
claim. If composition could read a card's Mechanism or Derivation it would be
doing generation again, and "solved by recombination alone, no new primitives"
would mean nothing.

The separation is enforced three ways, in decreasing order of how much I trust
them:

1. **The composition prompt's vocabulary is a list of node ids.** The model
   returns a JSON tree naming nodes from the retrieved set; it never returns
   Python. A plan naming an id that was not retrieved is rejected. So "no new
   primitives" is mechanical rather than a rule the model is asked to respect —
   the model has no channel through which to write one.
2. **The prompt is rendered from `LibraryEntry.identity()`**, which carries
   ports, kind, effect, precondition and tags, and does not carry mechanism,
   derivation or source. A card field cannot reach the prompt because nothing
   on the render path can see one.
3. **A test greps the rendered prompt against the card store** and fails on any
   Mechanism or Derivation sentence. Belt and braces, because 1 and 2 are
   properties of code that someone could later change without noticing.

**NEVER IMPORTS `core.scenarios_held_out`** — chunk 11's AST guard covers this
package, tests included. The one test that must import the split to prove a
TaskSpec leaks nothing therefore lives in `core/tests/test_compose_leak.py`:
weakening the guard to make room for a test would be weakening the seal in order
to test the seal.
"""
