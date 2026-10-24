// "Ask the thesis": semantic search over the thesis PDF, run in the browser.
//
// thesis-index.json holds the thesis cut into short passages, read from its
// LaTeX so they keep their headings, lists, tables, and emphasis, each with a
// vector from the all-MiniLM-L6-v2 model (built by
// scripts/build_thesis_index.py). A question gets a vector from the same
// model here, through transformers.js, and passages rank by meaning plus a
// share of keyword match. The model starts loading in the background once
// the page has settled; results show at once from keywords alone, then
// re-rank when the model has loaded. The ranking mirrors rank() in the
// build script; keep the two in step.

// transformers.js and its ONNX runtime are served by the site itself;
// scripts/vendor.json pins their versions and hashes.
const TRANSFORMERS = "./vendor/transformers/transformers.min.js";
const ORT = (file) => new URL(`vendor/ort/ort-wasm-simd-threaded.asyncify.${file}`, import.meta.url).href;
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
let asked = false; // Progress shows only once the visitor has asked something.

// A passage is a list of blocks: {p}, {h}, {ul}, {ol, start}, or {table};
// text runs are strings or [style, text] pairs. These mirror block_text()
// and passage_text() in scripts/thesis_latex.py; keep them in step.
const runsText = (runs) => runs.map((r) => (typeof r === "string" ? r : r[1])).join("");
function blockText(b) {
  if (b.p || b.h) return runsText(b.p || b.h);
  if (b.table) {
    const cells = [...b.table.head, ...b.table.rows.flat()];
    return [b.table.caption, ...cells.map((c) => runsText(c.t))].join(" ");
  }
  return (b.ul || b.ol).flatMap((item) => item.map(blockText)).join(" ");
}
const passageText = (p) => p.blocks.map(blockText).join(" ");

const words = (text) => (text.toLowerCase().match(/[a-z0-9]+/g) || []).filter((w) => !STOPWORDS.has(w));

async function loadIndex() {
  const res = await fetch("thesis-index.json");
  if (!res.ok) throw new Error(`thesis-index.json: HTTP ${res.status}`);
  const data = await res.json();
  const bytes = Uint8Array.from(atob(data.vectors), (c) => c.charCodeAt(0));
  if (bytes.length !== data.passages.length * data.dim) throw new Error("thesis-index.json: vectors do not match passages");
  const docs = data.passages.map((p) => words(`${p.title} ${p.heading} ${passageText(p)}`));
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
    // line is announced to screen readers, so update it in 10% steps only.
    const pct = Math.floor((100 * loaded) / total);
    if (!asked || pct < shown + 10) return;
    shown = pct;
    setStatus(`Loading the language model (about 30 MB, once): ${pct}%. Until then, results match keywords.`);
  };
  return import(TRANSFORMERS)
    .then(({ pipeline, env }) => {
      env.backends.onnx.wasm.wasmPaths = { mjs: ORT("mjs"), wasm: ORT("wasm") };
      return pipeline("feature-extraction", model.model, {
        revision: model.revision,
        dtype: model.dtype,
        progress_callback: progress,
      });
    })
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

// The last search, kept for this tab. Back from the PDF viewer restores the
// page from the browser's cache when it can; when the browser reloads it
// instead, this puts the question and its results back as the page loads,
// so the browser's scroll restore lands on them.
const SAVED = "ask-the-thesis";
function remember(question, passages) {
  try {
    sessionStorage.setItem(SAVED, JSON.stringify({ question, passages, status: status.textContent }));
  } catch {
    // Storage can be full or blocked; the search still works.
  }
}
function restore() {
  let saved = null;
  try {
    saved = JSON.parse(sessionStorage.getItem(SAVED));
  } catch {
    return;
  }
  // Results saved before the index kept its structure have no blocks.
  if (!saved?.passages?.length || !saved.passages.every((p) => p.blocks)) return;
  input.value = saved.question;
  show(saved.passages);
  setStatus(saved.status);
}

// Open the passage in the PDF.js viewer at its page, with up to 14 of its
// words highlighted. Only words of letters, apostrophes, and hyphens count:
// at digits or symbols the cleaned index text can differ from the PDF's own.
// The phrase is the first run of at least four such words, so a passage that
// opens with a figure number or a table cell still highlights a sentence.
function viewerPhrase(text) {
  const clean = (w) => /^[A-Za-z][A-Za-z'’-]*[,.;:]?$/.test(w);
  const runs = [[]];
  for (const w of text.split(" ")) {
    if (clean(w)) runs.at(-1).push(w);
    else if (runs.at(-1).length) runs.push([]);
  }
  const run = runs.find((r) => r.length >= 4) ?? runs.reduce((a, b) => (b.length > a.length ? b : a));
  // End at a full stop: the next sentence may be a caption placed elsewhere on the page.
  const stop = run.findIndex((w, i) => i >= 3 && w.endsWith("."));
  const words = run.slice(0, stop >= 0 ? Math.min(stop + 1, 14) : 14);
  return (words.join(" ") || text.split(" ")[0]).replace(/[,.;:]$/, "");
}
// The phrase comes from the passage's first block alone: in the PDF a list
// bullet or a table's layout sits between blocks, and the search would miss.
// For a table it is the first cell, which the build places on this page.
function firstText(blocks) {
  const b = blocks[0];
  if (b.p || b.h) return runsText(b.p || b.h);
  if (b.table) return runsText(b.table.rows[0][0].t);
  return firstText((b.ul || b.ol)[0]);
}
function viewerLink(p) {
  const search = encodeURIComponent(viewerPhrase(firstText(p.blocks)));
  return `pdfjs/web/viewer.html?file=../../thesis.pdf#page=${Number(p.pdfPage)}&search=${search}&phrase=true`;
}

// Results are built from elements and text nodes only; nothing from the
// index is ever parsed as HTML.
const TAGS = { b: "strong", i: "em", code: "code", sub: "sub", sup: "sup" };
function inline(el, runs) {
  for (const run of runs) {
    if (typeof run === "string") {
      el.append(run);
    } else {
      const span = document.createElement(TAGS[run[0]]);
      span.textContent = run[1];
      el.append(span);
    }
  }
  return el;
}
const make = (tag, className) => Object.assign(document.createElement(tag), className ? { className } : {});

function render(block) {
  if (block.p) return inline(make("p", "ask-text"), block.p);
  if (block.h) return inline(make("p", "ask-label"), block.h);
  if (block.table) {
    const { caption, head, rows } = block.table;
    const table = make("table");
    table.createCaption().textContent = caption;
    const cell = (tag, c) => {
      const el = inline(make(tag), c.t);
      if (c.span > 1) el.colSpan = c.span;
      return el;
    };
    table.createTHead().insertRow().append(...head.map((c) => cell("th", c)));
    const body = table.createTBody();
    for (const row of rows) body.insertRow().append(...row.map((c) => cell("td", c)));
    const wrap = make("div", "ask-table");
    wrap.append(table);
    return wrap;
  }
  const list = make(block.ol ? "ol" : "ul", "ask-list");
  if (block.ol && block.start > 1) list.start = block.start;
  for (const item of block.ol || block.ul) {
    const li = make("li");
    // An item that is one paragraph reads as a line, not a boxed paragraph.
    if (item.length === 1 && item[0].p) inline(li, item[0].p);
    else li.append(...item.map(render));
    list.append(li);
  }
  return list;
}

let shownKey = "";

function show(passages) {
  // Re-ranking often keeps the same three passages; leave them, and any
  // keyboard focus on their links, in place.
  const key = passages.map((p) => `${p.pdfPage}:${passageText(p).slice(0, 40)}`).join("|");
  if (key === shownKey) return;
  shownKey = key;
  results.replaceChildren(
    ...passages.map((p) => {
      const li = make("li");
      const page = `page ${p.page}`;
      const where = make("p", "ask-where");
      where.textContent = `${p.section ? `${p.section} ` : ""}${p.title} · ${page}`;
      li.append(where);
      if (p.heading) {
        const heading = make("h4", "ask-heading");
        heading.textContent = p.heading;
        li.append(heading);
      }
      li.append(...p.blocks.map(render));
      const link = make("a");
      link.href = viewerLink(p);
      link.textContent = `Open ${page} of the PDF`;
      li.append(link);
      return li;
    }),
  );
}

async function ask(question) {
  question = question.trim();
  if (!question) return;
  const id = ++latest;
  asked = true;
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
    remember(question, quick);
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
  const passages = rank(question, query);
  show(passages);
  setStatus("The three passages closest in meaning to your question.");
  remember(question, passages);
}

// Load the index and the model in the background, so the first question
// gets meaning-based results at once. page.js signals when the rest of the
// page has loaded, so the model never competes with it; focusing the
// question box starts it at once. With Save-Data on or on 2G, only a
// question starts it.
let preloading = false;
function preload() {
  if (preloading) return;
  preloading = true;
  index ??= loadIndex();
  index
    .then((data) => {
      index = data;
      embed ??= loadModel(data);
      return embed;
    })
    .catch((err) => {
      console.error(err);
      // The visitor's first question tries again.
      if (index instanceof Promise) index = null;
      embed = null;
    });
}
const net = navigator.connection;
if (!(net && (net.saveData || /2g/.test(net.effectiveType)))) {
  document.addEventListener("page-preloaded", preload, { once: true });
  input.addEventListener("focus", preload, { once: true });
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

restore();
