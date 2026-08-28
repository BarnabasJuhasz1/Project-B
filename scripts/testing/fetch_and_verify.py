"""
fetch_and_verify.py

Closes the loop on CORPUS-BRIEF.md §9: never write a span from memory.

Reads a manifest of candidate papers, downloads whatever is legitimately available,
extracts text page by page, and dumps the sentences that mention each paper's anchor
terms together with their page numbers. You then copy the real sentence into the table.

It does not bypass anything. If a fetch does not return a PDF, the row is marked
`paywalled` and no file is written.

    pip install requests pdfplumber        # pdfplumber is the FALLBACK extractor
    apt install poppler-utils              # pdftotext: the default, and much better
    python fetch_and_verify.py --manifest manifest.csv --out corpus/

Outputs:
    corpus/<filename>.pdf          one per successful download
    corpus/spans_candidates.md     candidate sentences, per paper, with page numbers
    corpus/access_report.csv       manifest + resolved access status
"""

import argparse
import csv
import os
import re
import shutil
import subprocess
import sys
import time

import requests

try:
    import pdfplumber
except ImportError:
    sys.exit("pip install pdfplumber")


UA = ("Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/124.0 Safari/537.36")

# Be polite. These are mostly university and society servers.
DELAY_SECONDS = 3.0
TIMEOUT = 60


def download(url, dest):
    """Return (ok, note). Only writes dest if the response really is a PDF."""
    try:
        r = requests.get(url, headers={"User-Agent": UA}, timeout=TIMEOUT,
                         allow_redirects=True)
    except requests.RequestException as e:
        return False, f"request failed: {type(e).__name__}"

    if r.status_code != 200:
        return False, f"http {r.status_code}"

    ctype = r.headers.get("content-type", "").lower()
    looks_pdf = r.content[:5] == b"%PDF-"

    if not looks_pdf:
        if "html" in ctype:
            return False, "returned HTML, not a PDF (paywall, captcha or landing page)"
        return False, f"unexpected content-type: {ctype or 'unknown'}"

    with open(dest, "wb") as f:
        f.write(r.content)
    return True, f"{len(r.content) // 1024} KB"


SENTENCE_END = re.compile(r"(?<=[.!?])\s+(?=[A-Z(])")

# Publisher download stamps, removed before sentences are cut.
#
# Not cosmetic: PNAS stamps the DOWNLOADER'S IP ADDRESS onto every page, so a
# span copied straight out of the candidate list would carry a personal
# identifier into a committed card. The stamp also splices itself into the
# middle of real sentences, which corrupts spans that are otherwise perfect.
WATERMARK = re.compile(
    # Most specific first: alternation is ordered, and a lazy prefix will
    # otherwise match nothing and leave the IP tail behind.
    r"Downloaded from \S+ by [^\s]+ on \w+ \d{1,2}, \d{4}"
    r"(?: from IP address [0-9a-fA-F.:]+\.?)?"
    r"|Downloaded from \S+(?: by \S+)?(?: on [^.]{0,40}?\d{4})?"
    r"(?: from IP address [0-9a-fA-F.:]+\.?)?"
    r"|from IP address [0-9a-fA-F.:]+\.?"
    r"|This content is licensed under[^.]{0,120}\.",
    re.IGNORECASE)


def _pages_pdftotext(pdf_path):
    """Poppler's extractor, one string per page.

    NOT pdfplumber, and the difference is not cosmetic. Older TeX-produced
    papers place every glyph with its own positioning operator, and pdfplumber
    then reports a space between each one - the DWA paper came out as
    "t h e c o n s t r u c t i o n o f l o c a l w o r l d m o d e l s".
    Every multi-word anchor misses, so the run reports success and yields
    nothing usable. Measured on that PDF: pdftotext is clean, pypdfium2 drops
    some inter-word spaces ("themethod"), pdfplumber is unusable.

    pdftotext separates pages with a form feed, so page numbers survive.
    """
    out = subprocess.run(["pdftotext", "-q", str(pdf_path), "-"],
                         capture_output=True, timeout=120)
    if out.returncode != 0:
        raise RuntimeError(f"pdftotext exited {out.returncode}")
    return out.stdout.decode("utf-8", "replace").split("\f")


def _pages_pdfplumber(pdf_path):
    """Fallback when poppler is not installed. See _pages_pdftotext for why
    this is the fallback and not the default."""
    with pdfplumber.open(pdf_path) as pdf:
        return [(page.extract_text() or "") for page in pdf.pages]


HAVE_PDFTOTEXT = shutil.which("pdftotext") is not None


def sentences_with_pages(pdf_path):
    """Yield (page_number, sentence). Page numbers are 1-indexed in printed order."""
    pages = (_pages_pdftotext(pdf_path) if HAVE_PDFTOTEXT
             else _pages_pdfplumber(pdf_path))

    for i, text in enumerate(pages, start=1):
        # Rejoin words broken across line ends, then collapse whitespace.
        text = re.sub(r"-\n(\w)", r"\1", text)
        text = re.sub(r"\s+", " ", text).strip()
        text = WATERMARK.sub(" ", text)
        if not text:
            continue
        for sent in SENTENCE_END.split(text):
            sent = sent.strip()
            if 40 <= len(sent) <= 400:
                yield i, sent


def find_candidates(pdf_path, anchors, max_hits=8):
    """Sentences containing any anchor term, or a number plus a unit-ish token."""
    anchors = [a.strip().lower() for a in anchors if a.strip()]
    hits, seen = [], set()

    for page, sent in sentences_with_pages(pdf_path):
        low = sent.lower()
        matched = [a for a in anchors if a in low]
        if not matched:
            continue
        key = sent[:80]
        if key in seen:
            continue
        seen.add(key)
        # Prefer sentences carrying a concrete quantity: §4 wants a threshold,
        # exponent, rule or measured relationship, not a claim of importance.
        quantitative = bool(re.search(r"\d+(\.\d+)?\s*(%|m/s|m\b|s\b|deg|°|×|x\b)", sent)) \
            or bool(re.search(r"\b(approximately|exponent|threshold|ratio|proportional|"
                              r"inversely|scales? with|greater than|less than)\b", low))
        hits.append((page, sent, matched, quantitative))

    # Quantitative sentences first, then by page order.
    hits.sort(key=lambda h: (not h[3], h[0]))
    return hits[:max_hits]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", default="manifest.csv")
    ap.add_argument("--out", default="corpus/")
    ap.add_argument("--skip-download", action="store_true",
                    help="re-run span extraction on already-downloaded PDFs")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    with open(args.manifest, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    report, spans_md = [], []

    for row in rows:
        fname = row["filename"]
        url = row["pdf_url"].strip()
        dest = os.path.join(args.out, fname)

        # A PDF already on disk is extractable however it got there, and that
        # includes "someone downloaded it by hand in a browser". Testing the
        # FILE before the URL is what makes a manual download a first-class
        # access state instead of a row skipped for having no URL to try -
        # which matters because the papers a fetcher cannot reach are exactly
        # the ones that get fetched by hand.
        have_url = bool(url) and url.lower() != "none"
        if os.path.exists(dest) and (args.skip_download or not have_url):
            status = "open access" if have_url else "manual"
            note = "cached" if have_url else "obtained by hand"
        elif not have_url:
            status, note = "paywalled", "no open URL known"
        else:
            ok, note = download(url, dest)
            status = "open access" if ok else "paywalled"
            time.sleep(DELAY_SECONDS)

        print(f"[{status:12}] {fname}  ({note})")
        report.append({**row, "access": status, "note": note})

        if status not in ("open access", "manual"):
            continue

        try:
            hits = find_candidates(dest, row.get("anchors", "").split("|"))
        except Exception as e:
            print(f"               extraction failed: {e}")
            continue

        spans_md.append(f"\n## {fname}\n")
        spans_md.append(f"*{row.get('citation','')}*  \n")
        spans_md.append(f"Bucket {row.get('bucket','?')}. "
                        f"Look in: {row.get('look_in','')}\n")
        if not hits:
            spans_md.append("\n**No anchor matches.** Widen the anchors or read it by hand.\n")
        for page, sent, matched, quant in hits:
            flag = "**[quantitative]** " if quant else ""
            spans_md.append(f"\n- p.{page} {flag}`{', '.join(matched)}`  \n  > {sent}\n")

    with open(os.path.join(args.out, "spans_candidates.md"), "w", encoding="utf-8") as f:
        f.write("# Candidate spans\n\n"
                "Copy the sentence that actually states the mechanism into the "
                "`quotable_span` column, with its page number. If none of these do, "
                "open the PDF. Do not paraphrase into the span column.\n")
        f.writelines(spans_md)

    if report:
        with open(os.path.join(args.out, "access_report.csv"), "w",
                  newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=list(report[0].keys()))
            w.writeheader()
            w.writerows(report)

    n_ok = sum(1 for r in report
               if r["access"] in ("open access", "manual"))
    print(f"\n{n_ok}/{len(report)} downloaded. "
          f"See {args.out}spans_candidates.md")


if __name__ == "__main__":
    main()
