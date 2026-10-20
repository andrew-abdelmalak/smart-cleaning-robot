// "Ask the thesis": semantic search over the thesis PDF, run in the browser.
//
// thesis-index.json holds the thesis cut into short passages, each with a
// vector from the all-MiniLM-L6-v2 model (built by
// scripts/build_thesis_index.py). A question gets a vector from the same
// model here, through transformers.js, and passages rank by meaning plus a
// share of keyword match. Results show at once from keywords alone, then
// re-rank when the model has loaded. The ranking mirrors rank() in the
// build script; keep the two in step.

const TRANSFORMERS = "https://cdn.jsdelivr.net/npm/@huggingface/transformers@4.3.0/dist/transformers.min.js";
const KEYWORD_WEIGHT = 0.3;
const STOPWORDS = new Set(
  ("a an the of to and or is are was were how what why which does do did you" +
    " it this that in on for from with by be as at").split(" "),
);

const form = document.getElementById("ask-form");
const input = document.getElementById("ask-input");
const status = document.getElementById("ask-status");
const results = document.getElementById("ask-results");

let index = null; // The parsed index (a promise while it loads), with token lists for keyword scoring.
let embed = null; // Promise of a function from question to vector.
let modelReady = false;
let latest = 0; // Drop answers to questions the visitor has since replaced.

const words = (text) => (text.toLowerCase().match(/[a-z0-9]+/g) || []).filter((w) => !STOPWORDS.has(w));

async function loadIndex() {
  const res = await fetch("thesis-index.json");
  if (!res.ok) throw new Error(`thesis-index.json: HTTP ${res.status}`);
  const data = await res.json();
  const bytes = Uint8Array.from(atob(data.vectors), (c) => c.charCodeAt(0));
  if (bytes.length !== data.passages.length * data.dim) throw new Error("thesis-index.json: vectors do not match passages");
  const docs = data.passages.map((p) => words(`${p.title} ${p.text}`));
  return {
    ...data,
    vectors: new Int8Array(bytes.buffer),
    docs,
    avgLen: docs.reduce((n, d) => n + d.length, 0) / docs.length,
  };
}

function loadModel(model) {
  const files = new Map();
  let shown = -10;
  const progress = (p) => {
    if (p.status !== "progress" || !p.total) return;
    files.set(p.file, [p.loaded, p.total]);
    let loaded = 0;
    let total = 0;
    for (const [l, t] of files.values()) {
      loaded += l;
      total += t;
    }
    // The model is about 23 MB and its runtime about 6 MB more. The status
    // line is announced to screen readers, so update it in 10 % steps only.
    const pct = Math.floor((100 * loaded) / total);
    if (pct < shown + 10) return;
    shown = pct;
    setStatus(`Loading the language model (about 30 MB, once): ${pct}%. Until then, results match keywords.`);
  };
  return import(TRANSFORMERS)
    .then(({ pipeline }) =>
      pipeline("feature-extraction", model.model, {
        revision: model.revision,
        dtype: model.dtype,
        progress_callback: progress,
      }),
    )
    .then((extract) => {
      modelReady = true;
      return async (question) => (await extract(question, { pooling: "mean", normalize: true })).data;
    });
}

function keywordScores(question) {
  const { docs, avgLen } = index;
  const terms = [...new Set(words(question))];
  const df = new Map(terms.map((t) => [t, docs.filter((d) => d.includes(t)).length]));
  return docs.map((doc) => {
    const norm = 1.2 * (0.25 + (0.75 * doc.length) / avgLen);
    let score = 0;
    for (const t of terms) {
      const tf = doc.filter((w) => w === t).length;
      if (!tf) continue;
      const idf = Math.log(1 + (docs.length - df.get(t) + 0.5) / (df.get(t) + 0.5));
      score += (idf * tf * 2.2) / (tf + norm);
    }
    return score;
  });
}

function rank(question, query) {
  const keywords = keywordScores(question);
  const top = Math.max(...keywords);
  const kw = keywords.map((k) => (top > 0 ? k / top : 0));
  let scores = kw;
  if (query) {
    const { vectors, dim } = index;
    const meaning = kw.map((_, i) => {
      let dot = 0;
      for (let j = 0; j < dim; j++) dot += vectors[i * dim + j] * query[j];
      return dot / 127;
    });
    const best = Math.max(...meaning);
    scores = meaning.map((m, i) => (1 - KEYWORD_WEIGHT) * m + KEYWORD_WEIGHT * kw[i] * best);
  }
  // Array sort is stable, so ties keep passage order, as in the build script.
  return scores
    .map((s, i) => [s, i])
    .sort((a, b) => b[0] - a[0])
    .filter(([s]) => query || s > 0)
    .slice(0, 3)
    .map(([, i]) => index.passages[i]);
}

function setStatus(text) {
  status.textContent = text;
}

let shownKey = "";

function show(passages) {
  // Re-ranking often keeps the same three passages; leave them, and any
  // keyboard focus on their links, in place.
  const key = passages.map((p) => `${p.pdfPage}:${p.text.slice(0, 40)}`).join("|");
  if (key === shownKey) return;
  shownKey = key;
  results.replaceChildren(
    ...passages.map((p) => {
      const li = document.createElement("li");
      const page = `page ${p.page}`;
      const where = document.createElement("p");
      where.className = "ask-where";
      where.textContent = `${p.section ? `${p.section} ` : ""}${p.title} · ${page}`;
      const text = document.createElement("p");
      text.className = "ask-text";
      text.textContent = p.text;
      const link = document.createElement("a");
      link.href = `thesis.pdf#page=${p.pdfPage}`;
      link.textContent = `Open ${page} of the PDF`;
      li.append(where, text, link);
      return li;
    }),
  );
}

async function ask(question) {
  question = question.trim();
  if (!question) return;
  const id = ++latest;
  index ??= loadIndex();
  try {
    index = await index;
  } catch (err) {
    console.error(err);
    index = null; // The next question tries again.
    setStatus("The search index did not load. Check your connection, or read the PDF above.");
    return;
  }
  if (id !== latest) return;

  let quick = [];
  const model = (embed ??= loadModel(index));
  if (!modelReady) {
    quick = rank(question, null);
    show(quick);
    setStatus(quick.length ? "Keyword matches while the language model loads." : "No keyword matches. The language model is loading.");
  }
  let query;
  try {
    query = await (await model)(question);
  } catch (err) {
    console.error(err);
    if (embed === model) embed = null; // The next question tries again.
    if (id !== latest) return;
    setStatus(
      quick.length
        ? "The language model did not load, so these are keyword matches only."
        : "The language model did not load, and no passage matches those words. Try other words.",
    );
    return;
  }
  if (id !== latest) return;
  show(rank(question, query));
  setStatus("The three passages closest in meaning to your question.");
}

form.addEventListener("submit", (e) => {
  e.preventDefault();
  ask(input.value);
});

document.querySelectorAll("[data-ask]").forEach((button) => {
  button.addEventListener("click", () => {
    input.value = button.textContent;
    ask(input.value);
  });
});
