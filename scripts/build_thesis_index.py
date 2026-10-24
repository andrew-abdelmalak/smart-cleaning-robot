"""Build site/thesis-index.json, the passage index behind "Ask the thesis".

Reads the thesis LaTeX into passages that keep its headings, lists, tables,
and emphasis (scripts/thesis_latex.py), places each on its page of the PDF,
then embeds each passage with the same model the site runs in the browser
(all-MiniLM-L6-v2, 8-bit ONNX), so a question embedded on the page can be
compared directly with the stored vectors.

Needs poppler's pdftotext and the `search` dependency group:

    uv sync --group search
    uv run scripts/build_thesis_index.py
"""

import base64
import json
import re
import subprocess
import unicodedata
from pathlib import Path

import numpy as np
import thesis_latex as latex

ROOT = Path(__file__).resolve().parent.parent
PDF = ROOT / "docs/thesis/thesis.pdf"
OUT = ROOT / "site/thesis-index.json"

MODEL = "Xenova/all-MiniLM-L6-v2"
REVISION = "751bff37182d3f1213fa05d7196b954e230abad9"
ONNX_FILE = "onnx/model_quantized.onnx"  # transformers.js dtype "q8"

# Printed page numbers start at PDF page 14 (Chapter 1 is page 1).
PAGE_OFFSET = 13
ABSTRACT_PAGE = 5
BODY_PAGES = range(14, 64)  # Chapters 1 to 5
APPENDIX_PAGES = range(67, 70)  # Bill of materials
COVERAGE = 0.9  # share of the PDF text the passages must hold; measured 0.93


def page_text(page: int) -> list[str]:
    out = subprocess.run(
        ["pdftotext", "-f", str(page), "-l", str(page), str(PDF), "-"],
        check=True,
        capture_output=True,
        encoding="utf-8",
    ).stdout
    lines = [line.strip() for line in out.splitlines() if line.strip()]
    # Drop the printed page number at the foot of the page.
    if lines and re.fullmatch(r"\d+|[ivx]+", lines[-1]):
        lines.pop()
    # A word hyphenated across the page break comes out as the page's last
    # line with the page number glued to its first half ("com47"); restore
    # the hyphen so sections() joins it to the rest of the word.
    number = str(page - PAGE_OFFSET)
    if lines:
        head = lines[-1].removesuffix(number)
        if head != lines[-1] and head[-1:].islower():
            lines[-1] = head + "-"
    return lines


def normalize(text: str) -> str:
    """Letters and digits only, so LaTeX and PDF text compare despite layout."""
    return re.sub(r"[^a-z0-9]", "", unicodedata.normalize("NFKC", text).lower())


def pdf_pages() -> dict[int, tuple[str, str]]:
    """Each page's text normalized as one string, and as space-separated words."""
    pages = {}
    for page in [ABSTRACT_PAGE, *BODY_PAGES, *APPENDIX_PAGES]:
        text = " ".join(page_text(page))
        words = " ".join(w for w in map(normalize, text.split()) if w)
        pages[page] = (normalize(text), f" {words} ")
    return pages


def _key(block: dict) -> str:
    """The start of a block as it appears in the PDF text."""
    if "table" in block:
        return normalize(latex.plain(block["table"]["rows"][0][0]["t"]))[:30]
    return normalize(latex.block_text(block))[:40]


def _check(passage: dict, text: str, words: str) -> list[str]:
    """Parts of a passage the PDF does not show on its pages."""
    missing = []
    for block in passage["blocks"]:
        if "table" in block:
            t = block["table"]
            cells = [c for row in [t["head"], *t["rows"]] for c in row]
            missing += [
                latex.plain(c["t"])
                for c in cells
                if normalize(latex.plain(c["t"])) not in text
            ]
            continue
        tokens = [w for w in map(normalize, latex.block_text(block).split()) if w]
        shingles = ["".join(tokens[k : k + 3]) for k in range(max(1, len(tokens) - 2))]
        found = sum(s in text for s in shingles)
        # Numbers (figure and table numbers, values) must each match a whole
        # word of the PDF, so 5.1 cannot pass inside 2551.
        numbers = [w for w in tokens if re.search(r"\d", w) and f" {w} " not in words]
        if found < 0.9 * len(shingles) or numbers:
            missing.append(f"{latex.block_text(block)[:80]} {numbers}")
    return missing


def passages() -> list[dict]:
    """Passages from the LaTeX, each placed on the PDF page where it starts.

    Every passage is checked against the PDF's own text, so a parsing mistake
    (a dropped word, a wrong figure number) stops the build.
    """
    pages = pdf_pages()
    order = list(pages)
    out, at, problems = [], 0, []
    for passage in latex.passages():
        keys = [_key(b) for b in passage["blocks"]]
        where = next(
            (
                k
                for k in range(at, len(order))
                for key in keys
                if key in pages[order[k]][0]
            ),
            None,
        )
        if where is None:
            raise ValueError(
                f"not found in the PDF: {latex.passage_text(passage)[:80]}"
            )
        at = where
        pdf_page = order[where]
        # A passage can run on past full-page figures, so look a few pages on.
        window = order[where : where + 5]
        text = "".join(pages[p][0] for p in window)
        words = "".join(pages[p][1] for p in window)
        problems += [f"p.{pdf_page}: {m}" for m in _check(passage, text, words)]
        page = "v" if pdf_page == ABSTRACT_PAGE else str(pdf_page - PAGE_OFFSET)
        out.append({**passage, "page": page, "pdfPage": pdf_page})
    if problems:
        raise ValueError("passages differ from the PDF:\n" + "\n".join(problems))
    # And the other way round: the passages hold most of the PDF's text. The
    # rest is figure captions, headings, and page furniture the index leaves out.
    held = sum(len(normalize(latex.passage_text(p))) for p in out)
    total = sum(len(text) for text, _ in pages.values())
    if held < COVERAGE * total:
        raise ValueError(f"passages hold {held / total:.0%} of the PDF text")
    return out


class Embedder:
    """Mean-pooled, normalized MiniLM embeddings, computed as transformers.js does."""

    def __init__(self) -> None:
        import onnxruntime
        from huggingface_hub import hf_hub_download
        from tokenizers import Tokenizer

        def fetch(name: str) -> str:
            return hf_hub_download(MODEL, name, revision=REVISION)

        self.tokenizer = Tokenizer.from_file(fetch("tokenizer.json"))
        self.tokenizer.enable_truncation(256)
        self.tokenizer.enable_padding()
        self.session = onnxruntime.InferenceSession(fetch(ONNX_FILE))

    def __call__(self, texts: list[str]) -> np.ndarray:
        enc = self.tokenizer.encode_batch(texts)
        ids = np.array([e.ids for e in enc], dtype=np.int64)
        mask = np.array([e.attention_mask for e in enc], dtype=np.int64)
        feeds = {
            "input_ids": ids,
            "attention_mask": mask,
            "token_type_ids": np.zeros_like(ids),
        }
        (hidden,) = self.session.run(["last_hidden_state"], feeds)
        summed = (hidden * mask[..., None]).sum(axis=1)
        vectors = summed / mask.sum(axis=1, keepdims=True)
        return vectors / np.linalg.norm(vectors, axis=1, keepdims=True)


def quantize(vectors: np.ndarray) -> str:
    """Store each unit vector as signed bytes (value * 127), base64 encoded."""
    data = np.round(vectors * 127).astype(np.int8).tobytes()
    return base64.b64encode(data).decode()


def load_vectors(index: dict) -> np.ndarray:
    raw = np.frombuffer(base64.b64decode(index["vectors"]), dtype=np.int8)
    shape = (len(index["passages"]), index["dim"])
    return raw.reshape(shape).astype(np.float32) / 127


# Ranking. site/ask.js runs the same steps in the browser; keep them in step.
STOPWORDS = frozenset(
    [
        "a",
        "an",
        "the",
        "of",
        "to",
        "and",
        "or",
        "is",
        "are",
        "was",
        "were",
        "how",
        "what",
        "why",
        "which",
        "does",
        "do",
        "did",
        "you",
        "it",
        "this",
        "that",
        "in",
        "on",
        "for",
        "from",
        "with",
        "by",
        "be",
        "as",
        "at",
    ]
)
KEYWORD_WEIGHT = 0.3


def words(text: str) -> list[str]:
    return [w for w in re.findall(r"[a-z0-9]+", text.lower()) if w not in STOPWORDS]


def keyword_scores(question: str, items: list[dict]) -> np.ndarray:
    """BM25 score of each passage (with its section title) for the question."""
    docs = [
        words(f"{p['title']} {p['heading']} {latex.passage_text(p)}") for p in items
    ]
    avg_len = sum(map(len, docs)) / len(docs)
    terms = set(words(question))
    df = {t: sum(t in d for d in docs) for t in terms}
    scores = np.zeros(len(docs))
    for i, doc in enumerate(docs):
        norm = 1.2 * (0.25 + 0.75 * len(doc) / avg_len)
        for t in terms:
            tf = doc.count(t)
            if tf:
                idf = np.log(1 + (len(docs) - df[t] + 0.5) / (df[t] + 0.5))
                scores[i] += idf * tf * 2.2 / (tf + norm)
    return scores


def rank(
    question: str, query: np.ndarray | None, items: list[dict], vectors: np.ndarray
) -> list[int]:
    """Passage indices, best first: meaning plus a share of keyword match."""
    keywords = keyword_scores(question, items)
    if keywords.max() > 0:
        keywords /= keywords.max()
    if query is None:  # The model has not loaded: keywords alone.
        return list(np.argsort(-keywords, kind="stable"))
    meaning = vectors @ query
    blended = (1 - KEYWORD_WEIGHT) * meaning
    blended += KEYWORD_WEIGHT * keywords * meaning.max()
    return list(np.argsort(-blended, kind="stable"))


def embed_text(passage: dict) -> str:
    heading = f"{passage['heading']}. " if passage["heading"] else ""
    return heading + latex.passage_text(passage)


def main() -> None:
    items = passages()
    embed = Embedder()
    # Each passage vector leans toward its section title, the author's own
    # summary of what the passage is about.
    vectors = embed([f"{p['title']}: {embed_text(p)}" for p in items])
    vectors += embed([p["title"] for p in items])
    vectors /= np.linalg.norm(vectors, axis=1, keepdims=True)
    index = {
        "model": MODEL,
        "revision": REVISION,
        "dtype": "q8",
        "dim": int(vectors.shape[1]),
        "passages": items,
        "vectors": quantize(vectors),
    }
    text = json.dumps(index, ensure_ascii=False, separators=(",", ":"))
    OUT.write_text(text + "\n", encoding="utf-8")
    print(f"wrote {OUT.relative_to(ROOT)}: {len(items)} passages")
    seen = []
    for p in items:
        key = (p["section"], p["title"], p["page"])
        if key[:2] not in [s[:2] for s in seen]:
            seen.append(key)
    for section, title, page in seen:
        print(f"  {section:6} p.{page:3} {title}")


if __name__ == "__main__":
    main()
