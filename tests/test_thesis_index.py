"""Checks for site/thesis-index.json, the passage index behind "Ask the thesis".

The shape checks always run. The retrieval check embeds real questions with the
same model the site uses, so it needs the `search` dependency group and a
one-time model download; it is skipped without them.
"""

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "scripts"))

INDEX = json.loads((ROOT / "site/thesis-index.json").read_text(encoding="utf-8"))

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


def test_index_shape() -> None:
    from build_thesis_index import load_vectors

    passages = INDEX["passages"]
    vectors = load_vectors(INDEX)
    assert vectors.shape == (len(passages), 384)
    assert all(1 <= p["pdfPage"] <= 69 for p in passages)
    assert all(p["text"].strip() for p in passages)
    # Every section the questions expect exists in the index.
    sections = {p["section"] for p in passages}
    assert set().union(*QUESTIONS.values()) <= sections


def test_questions_find_their_section_in_top_three() -> None:
    pytest.importorskip("onnxruntime")
    pytest.importorskip("tokenizers")
    from build_thesis_index import Embedder, load_vectors, rank

    passages = INDEX["passages"]
    vectors = load_vectors(INDEX)
    queries = Embedder()(list(QUESTIONS))
    misses = []
    for (question, expected), query in zip(QUESTIONS.items(), queries, strict=True):
        top = rank(question, query, passages, vectors)[:3]
        found = [passages[i]["section"] for i in top]
        if not expected & set(found):
            misses.append(f"{question} -> {found}, expected {sorted(expected)}")
    hit_rate = 1 - len(misses) / len(QUESTIONS)
    print(f"hit@3 = {hit_rate:.0%}", *misses, sep="\n  ")
    # 17 of 21 when the index was built; the floor catches regressions.
    assert hit_rate >= 0.75, "\n".join(misses)


def test_keyword_ranking_finds_exact_terms() -> None:
    # The page falls back to keywords alone while the model loads.
    from build_thesis_index import load_vectors, rank

    passages = INDEX["passages"]
    top = rank("L298N motor driver", None, passages, load_vectors(INDEX))[:3]
    assert any("L298N" in passages[i]["text"] for i in top)
