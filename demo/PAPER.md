# `demo/paper.*` — the paper armB was handed, by hash

Stentz, A. (1994). *Optimal and Efficient Path Planning for Partially-Known
Environments.* Proc. IEEE ICRA, 3310–3317.

Neither file is in git (licence; `Mindfire_Experiment_Plan.md` R10). Both are
listed in the repo's `.gitignore`. Place them at these paths with these hashes:

    dedfc448b455e380f1a195acd43457004a893d6d41fc92b6b7ef4b9c3fa3508e  demo/paper.pdf
    1294dc34b73f9b2a0b72c7cad6bff9e8b1dd3172e0049d841192674ff0f9843a  demo/paper.txt

⚠️ `paper.txt` is what reaches the model (`render.paper_text()`), not the PDF.
armB's prompt hash `4045efb3e514b981` is reproducible only from a `paper.txt`
with exactly this SHA-256; any re-extraction that differs by a byte changes it.

Check:

    sha256sum demo/paper.pdf demo/paper.txt
