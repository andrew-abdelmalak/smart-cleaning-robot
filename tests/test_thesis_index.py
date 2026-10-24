"""Checks for site/thesis-index.json, the passage index behind "Ask the thesis".

The shape and parity checks always run. The retrieval check embeds real
questions with the same model the site uses, so it needs the `search`
dependency group and a one-time model download; it is skipped without them.
"""

import json
import re
from pathlib import Path

import build_thesis_index as build
import numpy as np
import pytest
import thesis_latex as latex

ROOT = Path(__file__).resolve().parent.parent

# Questions a visitor might ask, with the sections that answer them.
QUESTIONS = {
    "How much did the robot cost to build?": {"5.1.1", "A"},
    "What microcontroller does the robot use?": {"4.2", "4.3", "4.5.1"},
    "How does the robot avoid obstacles?": {"4.3", "4.5.1", "4.5.2", "4.6.1"},
    "Why did pure PID control fail?": {"4.5.2"},
    "How do you control the robot from a phone?": {"4.5.3"},
    "What are the limitations of the prototype?": {"5.2"},
    "What would you improve in the future?": {"5.4"},
    "How does the vacuum create suction?": {"3.3.2"},
    "What material is the chassis printed in?": {"3.2", "3.3.1"},
    "How long does the battery last?": {"4.4.1", "4.6", "4.6.1", "4.6.2"},
    "How do the wheel encoders work?": {"2.1", "3.3.3", "4.5.2"},
    "Which motor driver does it use?": {"4.2", "4.3", "4.4.2"},
    "How was the circuit board designed?": {"4.4.3"},
    "How is the code uploaded to the Pico?": {"4.5.4"},
    "What did you learn from this project?": {"5.3"},
    "How do commercial robots like the Roomba navigate?": {"2.1", "2.2.2"},
    "What is feed-forward control?": {"2.1", "4.5.2"},
    "What happens when a wheel gets stuck?": {"4.5.1", "4.5.2", "4.6.1"},
    "What problem does this thesis solve?": {"", "1.2", "2.3"},
    "Which pins are the motors connected to?": {"4.4.2"},
    "How is the robot powered?": {"4.2", "4.4.1"},
}


@pytest.fixture(scope="module")
def index() -> dict:
    return json.loads((ROOT / "site/thesis-index.json").read_text(encoding="utf-8"))


def test_index_shape(index: dict) -> None:
    passages = index["passages"]
    vectors = build.load_vectors(index)
    assert (index["model"], index["revision"], index["dtype"]) == (
        build.MODEL,
        build.REVISION,
        "q8",
    )
    assert vectors.shape == (len(passages), 384)
    # Unit vectors survive the 8-bit storage; a misaligned array would not.
    assert np.allclose(np.linalg.norm(vectors, axis=1), 1, atol=0.03)
    for p in passages:
        assert p["blocks"] and latex.passage_text(p).strip()
        printed = (
            "v"
            if p["pdfPage"] == build.ABSTRACT_PAGE
            else str(p["pdfPage"] - build.PAGE_OFFSET)
        )
        assert p["page"] == printed
    # Every section the questions expect exists in the index.
    sections = {p["section"] for p in passages}
    assert set().union(*QUESTIONS.values()) <= sections


def test_browser_ranking_uses_the_same_constants() -> None:
    # site/ask.js mirrors rank(); a change on one side only would make the
    # page rank differently from what the retrieval test measures.
    js = (ROOT / "site/ask.js").read_text(encoding="utf-8")
    weight = re.search(r"KEYWORD_WEIGHT = ([\d.]+);", js)
    assert weight and float(weight[1]) == build.KEYWORD_WEIGHT
    stop = re.search(r'\(\s*"([^"]+)" \+\s*"([^"]+)"\s*\)\.split', js)
    assert stop and set((stop[1] + stop[2]).split()) == build.STOPWORDS
    for constant in ("1.2", "0.25", "0.75", "2.2"):
        assert constant in js


def test_questions_find_their_section_in_top_three(index: dict) -> None:
    pytest.importorskip("onnxruntime")
    pytest.importorskip("tokenizers")
    try:
        embed = build.Embedder()
    except OSError as err:  # offline and the model is not cached
        pytest.skip(f"model unavailable: {err}")

    passages = index["passages"]
    vectors = build.load_vectors(index)
    queries = embed(list(QUESTIONS))
    misses = []
    for (question, expected), query in zip(QUESTIONS.items(), queries, strict=True):
        top = build.rank(question, query, passages, vectors)[:3]
        found = [passages[i]["section"] for i in top]
        if not expected & set(found):
            misses.append(f"{question} -> {found}, expected {sorted(expected)}")
    hit_rate = 1 - len(misses) / len(QUESTIONS)
    print(f"hit@3 = {hit_rate:.0%}", *misses, sep="\n  ")
    # 17 of 21 when the index was built; the floor catches regressions.
    assert hit_rate >= 0.75, "\n".join(misses)


def test_keyword_ranking_finds_exact_terms(index: dict) -> None:
    # The page falls back to keywords alone while the model loads.
    passages = index["passages"]
    top = build.rank("L298N motor driver", None, passages, build.load_vectors(index))
    assert any("L298N" in latex.passage_text(passages[i]) for i in top[:3])


def _runs(block: dict):
    """Every run in a block, nested lists and table cells included."""
    if "p" in block or "h" in block:
        yield from block.get("p") or block.get("h")
    elif "table" in block:
        t = block["table"]
        for cell in [*t["head"], *(c for row in t["rows"] for c in row)]:
            yield from cell["t"]
    else:
        for item in block.get("ul") or block.get("ol"):
            for inner in item:
                yield from _runs(inner)


def test_index_matches_the_latex(index: dict) -> None:
    # The index is built from docs/thesis; a parser change without a rebuild
    # (or a rebuild without the parser change) fails here.
    fresh = latex.passages()
    assert [p["blocks"] for p in index["passages"]] == [p["blocks"] for p in fresh]
    assert [p["heading"] for p in index["passages"]] == [p["heading"] for p in fresh]


def test_no_raw_latex_reaches_the_site(index: dict) -> None:
    for p in index["passages"]:
        for block in p["blocks"]:
            for run in _runs(block):
                text = run if isinstance(run, str) else run[1]
                assert not re.search(r"[\\{}~]", text), (p["section"], text)


def test_structure_survives(index: dict) -> None:
    by_heading = {p["heading"]: p for p in index["passages"] if p["heading"]}
    tuning = by_heading["PID Tuning Methodology"]["blocks"]
    steps = next(b["ol"] for b in tuning if "ol" in b)
    assert len(steps) == 3
    assert steps[0][0]["p"][0] == ["b", "Start with P only"]
    assert ["sub", "p"] in tuning[0]["p"]  # K with subscript p
    tables = [
        b["table"] for p in index["passages"] for b in p["blocks"] if "table" in b
    ]
    cost = next(t for t in tables if t["caption"].startswith("Table 5.1:"))
    assert [latex.plain(c["t"]) for c in cost["head"]][:2] == [
        "Robot",
        "Approx. Cost (USD)",
    ]
