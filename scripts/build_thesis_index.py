"""Build site/thesis-index.json, the passage index behind "Ask the thesis".

Splits the thesis PDF into short passages tagged with their section and page,
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
from pathlib import Path

import numpy as np

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
TARGET_WORDS = 90

CHAPTER = re.compile(r"^(?:Chapter (\d)|Appendix (A))$")
# pdftotext puts a section number and its title on separate lines.
SECTION_NUMBER = re.compile(r"^([1-5A])\.\d{1,2}(?:\.\d{1,2})?$")
SECTION_TITLE = re.compile(r"^[A-Z][^.]{2,70}$")
# Chapter titles wrap across lines in the PDF, so they come from the LaTeX.
CHAPTER_TITLES = {
    "1": "Introduction",
    "2": "Theoretical Background and Literature Review",
    "3": "Mechanical Design and Assembly",
    "4": "Electrical and Control System Design and Testing",
    "5": "Conclusion and Recommendations for Future Work",
    "A": "Bill of Materials and Cost Analysis",
}


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


def _section_key(number: str) -> tuple[int, ...]:
    return tuple(0 if part == "A" else int(part) for part in number.split("."))


def _add_section(found: list[dict], number: str, title: str) -> None:
    """Start a section, failing loudly if the numbering goes backwards.

    A misread table cell or cross-reference would otherwise become a bogus
    section without any error.
    """
    last = found[-1]["section"]
    same_chapter = last.split(".")[0] == number.split(".")[0]
    if last and same_chapter and _section_key(number) <= _section_key(last):
        raise ValueError(f"section {number} {title!r} follows {last}: misread?")
    found.append({"section": number, "title": title, "lines": []})


def sections() -> list[dict]:
    """Group the text into numbered sections, remembering each one's first page."""
    abstract = page_text(ABSTRACT_PAGE)[1:]
    found = [
        {
            "section": "",
            "title": "Abstract",
            "lines": [(ABSTRACT_PAGE, line) for line in abstract],
        }
    ]
    chapter = ""
    for page in [*BODY_PAGES, *APPENDIX_PAGES]:
        lines = page_text(page)
        i = 0
        while i < len(lines):
            line = lines[i]
            heading = CHAPTER.match(line)
            number = SECTION_NUMBER.match(line)
            if heading:
                chapter = heading[1] or heading[2]
                title = CHAPTER_TITLES[chapter]
                _add_section(found, chapter, title)
                # Skip the title lines that follow, however they wrap.
                i += 1
                shown = ""
                while i < len(lines) and title.startswith(
                    f"{shown} {lines[i]}".strip()
                ):
                    shown = f"{shown} {lines[i]}".strip()
                    i += 1
                continue
            if number and number[1] == chapter:
                # A table that runs on from the previous section can land
                # between the number and its title; it stays with that section.
                ahead = lines[i + 1 : i + 7]
                at = next(
                    (k for k, s in enumerate(ahead) if SECTION_TITLE.match(s)), None
                )
                if at is not None:
                    found[-1]["lines"] += [(page, s) for s in ahead[:at]]
                    _add_section(found, line, ahead[at])
                    i += 2 + at
                    continue
            body = found[-1]["lines"]
            if body and body[-1][1].endswith("-") and line[:1].islower():
                start, first = body.pop()
                body.append((start, first[:-1] + line))
            else:
                body.append((page, line))
            i += 1
    return found


def passages() -> list[dict]:
    """Cut each section into passages of about TARGET_WORDS words on sentence ends."""
    out = []
    for sec in sections():
        words = [(page, word) for page, line in sec["lines"] for word in line.split()]
        start = 0
        while start < len(words):
            end = min(start + TARGET_WORDS, len(words))
            # Extend to the end of the sentence, within reason.
            while end < len(words) and end - start < TARGET_WORDS * 1.5:
                if words[end - 1][1].endswith((".", ":", "?")):
                    break
                end += 1
            chunk = words[start:end]
            previous = out[-1] if out else None
            if len(chunk) < 12 and previous and previous["title"] == sec["title"]:
                # A short tail joins the passage before it rather than vanish.
                previous["text"] += " " + " ".join(word for _, word in chunk)
            elif chunk:
                pdf_page = chunk[0][0]
                page = "v" if pdf_page == ABSTRACT_PAGE else str(pdf_page - PAGE_OFFSET)
                out.append(
                    {
                        "section": sec["section"],
                        "title": sec["title"],
                        "page": page,
                        "pdfPage": pdf_page,
                        "text": " ".join(word for _, word in chunk),
                    }
                )
            start = end
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
    docs = [words(f"{p['title']} {p['text']}") for p in items]
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


def main() -> None:
    items = passages()
    embed = Embedder()
    # Each passage vector leans toward its section title, the author's own
    # summary of what the passage is about.
    vectors = embed([f"{p['title']}: {p['text']}" for p in items])
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
