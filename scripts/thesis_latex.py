"""Read the thesis LaTeX into structured passages for "Ask the thesis".

The PDF's extracted text loses headings, lists, tables, bold labels, and
subscripts, so search results read as one run-on block. This module reads
docs/thesis instead and keeps that structure. It numbers sections, figures,
tables, and citations as LaTeX does, leaves figures out, and fails loudly on
any command it does not know, so nothing raw reaches the site. The LaTeX
itself is never changed.

A passage is a dict with "section" (number of the deepest numbered heading),
"title" (that heading's title), "heading" (an unnumbered sub-heading, or ""),
and "blocks": paragraphs ({"p": runs}), minor headings ({"h": runs}), lists
({"ul" or "ol": items, "start": n}, each item a list of blocks), and tables
({"table": {"caption", "head", "rows"}}, each cell {"t": runs, "span": n}).
Runs are strings, or [style, text] pairs with style "b", "i", "code", "sub",
or "sup".
"""

import re
from pathlib import Path

THESIS = Path(__file__).resolve().parent.parent / "docs/thesis"
CHAPTERS = [
    "chapter1_introduction",
    "chapter2_background",
    "chapter3_mechanical",
    "chapter4_electrical_control",
    "chapter5_conclusion",
]
APPENDIX = "appendix_bom"

# A passage gathers whole blocks until it holds at least MIN_WORDS words and
# stops before MAX_WORDS. Blocks never split, except lists by item and
# tables by row.
MIN_WORDS = 60
MAX_WORDS = 130
TABLE_ROWS = 10

MATH = {
    r"\times": "×",
    r"\varnothing": "∅",
    r"\Omega": "Ω",
    r"\sim": "∼",
    r"\to": " → ",
    r"\approx": " ≈ ",
}
TEXT_SYMBOLS = {
    r"\%": "%",
    r"\&": "&",
    r"\_": "_",
    r"\#": "#",
    r"\$": "$",
    r"\ ": " ",
    r"\,": " ",  # thin space, as in 16\,cm
}
# Commands that only change layout, with the number of {} arguments to drop.
LAYOUT = {
    "centering": 0,
    "small": 0,
    "sloppy": 0,
    "fussy": 0,
    "hfill": 0,
    "clearpage": 0,
    "noindent": 0,
    "phantomsection": 0,
    "hline": 0,
    "label": 1,
    "vspace": 1,
    "cellcolor": 1,
    "includegraphics": 1,
    "addcontentsline": 3,
}
STYLES = {"textbf": "b", "textit": "i", "emph": "i", "texttt": "code"}
HEADINGS = ("chapter", "section", "subsection", "subsubsection")


# ---------------------------------------------------------------------------
# Scanning helpers
# ---------------------------------------------------------------------------


def strip_comments(src: str) -> str:
    """Drop comments with their line ends, as LaTeX does, so no blank line appears."""
    return re.sub(r"(?<!\\)%.*\n?", "", src)


def group(s: str, i: int, open_: str = "{", close: str = "}") -> tuple[str, int]:
    """Return the balanced group at s[i] (after spaces) and the index after it."""
    while i < len(s) and s[i] in " \n":
        i += 1
    if i >= len(s) or s[i] != open_:
        raise ValueError(f"expected {open_!r} at {s[i : i + 40]!r}")
    depth = 0
    j = i
    while j < len(s):
        if s[j] == "\\":
            j += 2  # an escaped character never opens or closes a group
            continue
        if s[j] == open_:
            depth += 1
        elif s[j] == close:
            depth -= 1
            if depth == 0:
                return s[i + 1 : j], j + 1
        j += 1
    raise ValueError(f"unbalanced group at {s[i : i + 40]!r}")


def optional(s: str, i: int) -> tuple[str | None, int]:
    j = i
    while j < len(s) and s[j] == " ":
        j += 1
    if j < len(s) and s[j] == "[":
        return group(s, j, "[", "]")
    return None, i


def environment(s: str, i: int, name: str) -> tuple[str, int]:
    """Body of the environment whose \\begin{name} starts at s[i], nesting aware."""
    begin, end = f"\\begin{{{name}}}", f"\\end{{{name}}}"
    depth, j = 0, i
    while True:
        b, e = s.find(begin, j), s.find(end, j)
        if e < 0:
            raise ValueError(f"no {end}")
        if 0 <= b < e:
            depth, j = depth + 1, b + len(begin)
        else:
            depth, j = depth - 1, e + len(end)
            if depth == 0:
                return s[i + len(begin) : e], j


def split_top(s: str, sep: str) -> list[str]:
    """Split on sep where it is outside braces and environments."""
    parts, depth, env, start, i = [], 0, 0, 0, 0
    while i < len(s):
        if s.startswith("\\begin{", i):
            env += 1
        elif s.startswith("\\end{", i):
            env -= 1
        if depth == 0 and env == 0 and s.startswith(sep, i):
            parts.append(s[start:i])
            i += len(sep)
            start = i
            continue
        if s[i] == "\\":
            i += 2  # an escaped character never opens or closes a group
            continue
        if s[i] == "{":
            depth += 1
        elif s[i] == "}":
            depth -= 1
        i += 1
    parts.append(s[start:])
    return parts


def macros() -> dict[str, str]:
    """Argument-free \\newcommand values from main.tex, such as \\totalVoltage."""
    src = strip_comments((THESIS / "main.tex").read_text(encoding="utf-8"))
    return {
        m[1]: group(src, m.end() - 1)[0]
        for m in re.finditer(r"\\newcommand\{\\(\w+)\}\{", src)
    }


def citations() -> dict[str, int]:
    src = (THESIS / "sections/references.tex").read_text(encoding="utf-8")
    return {key: n for n, key in enumerate(re.findall(r"\\bibitem\{([^}]+)\}", src), 1)}


# ---------------------------------------------------------------------------
# Inline text
# ---------------------------------------------------------------------------


class Inline:
    """Turns inline LaTeX into runs, resolving references and citations."""

    def __init__(
        self, labels: dict[str, str], cites: dict[str, int], defs: dict[str, str]
    ) -> None:
        self.labels = labels
        self.cites = cites
        self.defs = defs

    def runs(self, s: str) -> list:
        out: list = []
        self._walk(s, "", out)
        return tidy(out)

    def _walk(self, s: str, style: str, out: list) -> None:
        text: list[str] = []

        def flush() -> None:
            if text:
                out.append([style, "".join(text)])
                text.clear()

        i = 0
        while i < len(s):
            c = s[i]
            if c == "\\":
                two = s[i : i + 2]
                if two in TEXT_SYMBOLS:
                    text.append(TEXT_SYMBOLS[two])
                    i += 2
                    continue
                if two == "\\\\":
                    text.append(" ")
                    i += 2
                    continue
                m = re.match(r"\\([A-Za-z]+\*?)", s[i:])
                if not m:
                    raise ValueError(f"unknown escape {s[i : i + 20]!r}")
                name, i = m[1], i + m.end()
                if name in self.defs:
                    if s.startswith("{}", i):
                        i += 2
                    flush()
                    self._walk(self.defs[name], style, out)
                elif name in STYLES:
                    arg, i = group(s, i)
                    flush()
                    self._walk(arg, STYLES[name], out)
                elif name in ("textcolor", "href"):
                    _, i = group(s, i)
                    arg, i = group(s, i)
                    flush()
                    self._walk(arg, style, out)
                elif name == "hyperref":
                    _, i = optional(s, i)
                    arg, i = group(s, i)
                    flush()
                    self._walk(arg, style, out)
                elif name == "texorpdfstring":
                    _, i = group(s, i)
                    arg, i = group(s, i)
                    text.append(arg)
                elif name == "ref":
                    key, i = group(s, i)
                    text.append(self.labels[key])
                elif name == "cite":
                    keys, i = group(s, i)
                    nums = ", ".join(
                        str(self.cites[k.strip()]) for k in keys.split(",")
                    )
                    text.append(f"[{nums}]")
                elif name == "newline":
                    text.append(" ")
                elif name in LAYOUT:
                    if name == "includegraphics":
                        _, i = optional(s, i)
                    for _ in range(LAYOUT[name]):
                        _, i = group(s, i)
                else:
                    raise ValueError(
                        f"unknown command \\{name} near {s[max(0, i - 40) : i + 20]!r}"
                    )
            elif c == "$":
                end = s.index("$", i + 1)
                flush()
                self._math(s[i + 1 : end], style, out)
                i = end + 1
            elif c in "{}":
                i += 1
            elif c == "~":
                text.append(" ")
                i += 1
            elif s.startswith("``", i) or s.startswith("''", i):
                text.append("“" if c == "`" else "”")
                i += 2
            elif s.startswith("---", i):
                text.append("—")
                i += 3
            elif s.startswith("--", i):
                text.append("–")
                i += 2
            else:
                text.append(c)
                i += 1
        flush()

    def _math(self, s: str, style: str, out: list) -> None:
        i = 0
        while i < len(s):
            m = re.match(r"\\[A-Za-z]+", s[i:])
            if m:
                i += m.end()
                if m[0] == r"\newline":
                    continue
                if m[0] not in MATH:
                    raise ValueError(f"unknown math {m[0]}")
                out.append([style, MATH[m[0]]])
            elif s[i] in "_^":
                kind = "sub" if s[i] == "_" else "sup"
                if s[i + 1] == "{":
                    arg, i = group(s, i + 1)
                else:
                    arg, i = s[i + 1], i + 2
                out.append([kind, arg])
            elif s[i] == " ":
                i += 1
            elif s[i] == ",":
                out.append([style, ", "])
                i += 1
            elif s[i] in "\\{}":
                raise ValueError(f"unknown math {s[i : i + 12]!r}")
            else:
                out.append([style, s[i]])
                i += 1


def tidy(runs: list) -> list:
    """Merge runs of one style, collapse spaces, and make unstyled runs strings."""
    merged: list[list[str]] = []
    for style, text in runs:
        text = re.sub(r"\s+", " ", text)
        if merged and merged[-1][0] == style:
            merged[-1][1] += text
        else:
            merged.append([style, text])
    if merged:
        merged[0][1] = merged[0][1].lstrip()
        merged[-1][1] = merged[-1][1].rstrip()
    return [text if not style else [style, text] for style, text in merged if text]


def plain(runs: list) -> str:
    return "".join(r if isinstance(r, str) else r[1] for r in runs)


def block_text(block: dict) -> str:
    """The words of a block, as the site's keyword search sees them."""
    if "p" in block:
        return plain(block["p"])
    if "h" in block:
        return plain(block["h"])
    if "table" in block:
        t = block["table"]
        cells = [*t["head"], *(c for row in t["rows"] for c in row)]
        return " ".join([t["caption"], *(plain(c["t"]) for c in cells)])
    items = block.get("ul") or block.get("ol")
    return " ".join(block_text(b) for item in items for b in item)


def passage_text(passage: dict) -> str:
    return " ".join(block_text(b) for b in passage["blocks"])


def words_in(block: dict) -> int:
    return len(block_text(block).split())


def drop_empty_columns(head: list[dict], rows: list[list[dict]]) -> tuple:
    """Leave out columns that only held pictures, which parse to nothing."""
    width = max(sum(c["span"] for c in r) for r in [head, *rows])

    def columns(row: list[dict]) -> list[tuple[int, dict]]:
        out, col = [], 0
        for cell in row:
            out.append((col, cell))
            col += cell["span"]
        return out

    filled = set()
    for row in rows:
        for col, cell in columns(row):
            if cell["span"] == 1 and plain(cell["t"]):
                filled.add(col)
    keep = {c for c in range(width) if c in filled}

    def narrow(row: list[dict]) -> list[dict]:
        out = []
        for col, cell in columns(row):
            span = sum(1 for c in range(col, col + cell["span"]) if c in keep)
            if span:
                out.append({**cell, "span": span})
        return out

    def text(rs: list[list[dict]]) -> list[str]:
        return [plain(c["t"]) for r in rs for c in r if plain(c["t"])]

    head2, rows2 = narrow(head), [narrow(r) for r in rows]
    # Only a dropped column's header ("Photo") may go; body text may not.
    if text(rows) != text(rows2):
        raise ValueError("dropping empty columns would drop text")
    return head2, rows2


# ---------------------------------------------------------------------------
# Document structure
# ---------------------------------------------------------------------------


def sources() -> list[tuple[str, str]]:
    """(chapter number, LaTeX) for the abstract, the chapters, and the appendix."""
    front = strip_comments(
        (THESIS / "sections/frontmatter.tex").read_text(encoding="utf-8")
    )
    start = front.index(r"\chapter*{Abstract}")
    abstract = front[start : front.index(r"\clearpage", start)]

    def read(name: str) -> str:
        return strip_comments(
            (THESIS / f"sections/{name}.tex").read_text(encoding="utf-8")
        )

    return [
        ("", abstract),
        *((str(n), read(c)) for n, c in enumerate(CHAPTERS, 1)),
        ("A", read(APPENDIX)),
    ]


class Reader:
    """Walks the LaTeX once to number everything, then again to build blocks."""

    def __init__(self) -> None:
        self.labels: dict[str, str] = {}
        self.inline: Inline | None = None
        self.table_numbers: list[str] = []  # in order; table() takes each in turn

    # Numbering --------------------------------------------------------------

    def number(self) -> None:
        for chapter, src in sources():
            counters = [0, 0, 0]
            figure = table = sub = 0
            last = chapter
            for m in re.finditer(
                r"\\(chapter|section|subsection|subsubsection)(\*?)\{|\\begin\{(figure|sidewaysfigure|table|longtable|subfigure)\}|\\label\{([^}]+)\}|(\\end\{subfigure\})",
                src,
            ):
                kind, starred, env, label, sub_end = m[1], m[2], m[3], m[4], m[5]
                if kind and not starred and kind != "chapter":
                    level = HEADINGS.index(kind) - 1
                    if level < 2:  # subsubsections are not numbered
                        counters[level] += 1
                        counters[level + 1 :] = [0] * (2 - level)
                        last = ".".join([chapter, *map(str, counters[: level + 1])])
                elif kind == "chapter":
                    last = chapter
                elif env in ("figure", "sidewaysfigure"):
                    figure += 1
                    sub = 0
                    last = f"{chapter}.{figure}"
                elif env == "subfigure":
                    sub += 1
                    last = f"{chapter}.{figure}{'abcdefgh'[sub - 1]}"
                elif env in ("table", "longtable"):
                    table += 1
                    last = f"{chapter}.{table}"
                    self.table_numbers.append(last)
                elif sub_end:
                    last = f"{chapter}.{figure}"  # the figure's own label follows
                elif label:
                    self.labels[label] = last
        self.inline = Inline(self.labels, citations(), macros())

    # Blocks -----------------------------------------------------------------

    def runs(self, s: str) -> list:
        return self.inline.runs(s)

    def blocks(self, s: str) -> list[dict]:
        """Block-level content: paragraphs, lists, and tables; figures are left out."""
        out: list[dict] = []
        para: list[str] = []

        def end_para() -> None:
            text = " ".join(para).strip()
            para.clear()
            if not text:
                return
            for chunk in re.split(r"\n\s*\n", text):
                runs = self.runs(chunk)
                if not runs:
                    continue
                # A paragraph of bold text alone is a run-in heading.
                kind = (
                    "h"
                    if len(runs) == 1
                    and isinstance(runs[0], list)
                    and runs[0][0] == "b"
                    else "p"
                )
                out.append({kind: runs})

        i = 0
        while i < len(s):
            m = re.search(
                r"\\begin\{(\w+)\}|\\(?:begingroup|endgroup)|\\renewcommand", s[i:]
            )
            if not m:
                para.append(s[i:])
                break
            para.append(s[i : i + m.start()])
            at = i + m.start()
            if m[0] in (r"\begingroup", r"\endgroup"):
                i = at + len(m[0])
                continue
            if m[0] == r"\renewcommand":
                _, j = group(s, at + len(m[0]))
                _, i = group(s, j)
                continue
            env = m[1]
            body, i = environment(s, at, env)
            if env in ("itemize", "enumerate"):
                end_para()
                out.append(self.list_block(body, env))
            elif env in ("table", "longtable"):
                end_para()
                out.append({"table": self.table(body, env)})
            elif env == "center":
                end_para()
                out += self.blocks(body)
            elif env in ("figure", "sidewaysfigure"):
                end_para()
            else:
                raise ValueError(f"unknown environment {env}")
        end_para()
        return out

    def list_block(self, body: str, env: str) -> dict:
        items = []
        for part in split_top(body, r"\item")[1:]:
            label, _ = optional(part, 0)
            if label is not None:
                raise ValueError(f"item label [{label}] is not supported")
            items.append(self.blocks(part))
        return {"ol": items, "start": 1} if env == "enumerate" else {"ul": items}

    def table(self, body: str, env: str) -> dict:
        caption = ""
        m = re.search(r"\\caption\{", body)
        if m:
            text, end = group(body, m.end() - 1)
            caption = plain(self.runs(text))
            body = body[: m.start()] + body[end:]
        foot = ""
        if env == "table":
            _, j = optional(body, 0)  # placement, such as [htbp]
            body = body[j:]
            if body.count(r"\begin{tabular}") != 1:
                raise ValueError(f"table {caption!r} needs exactly one tabular")
            start = body.index(r"\begin{tabular}")
            inner, end = environment(body, start, "tabular")
            if plain(self.runs(body[:start] + body[end:])).strip():
                raise ValueError(f"text beside the tabular in table {caption!r}")
            body = inner
            _, j = group(body, 0)  # column spec
            head_part, *rows = split_top(body[j:], "\\\\")
        else:
            _, j = group(body, 0)  # column spec
            first, rest = body[j:].split(r"\endfirsthead", 1)
            # The repeated header and "Continued on next page" are layout;
            # the last footer holds the totals and stays.
            rest = rest.split(r"\endhead", 1)[1]
            footer, rest = rest.split(r"\endlastfoot", 1)
            foot = footer.split(r"\endfoot", 1)[-1]
            head_part = [p for p in split_top(first, "\\\\") if "&" in p][-1]
            rows = split_top(rest, "\\\\")
        head = self.cells(head_part)
        rows = [c for r in [*rows, *split_top(foot, "\\\\")] if (c := self.cells(r))]
        number = self.table_numbers.pop(0)
        head, rows = drop_empty_columns(head, rows)
        return {"caption": f"Table {number}: {caption}", "head": head, "rows": rows}

    def cells(self, row: str) -> list[dict]:
        """A table row's cells, or [] for a row with no text (rules, spacing)."""
        out = []
        for cell in split_top(re.sub(r"\\hline|\\cline\{[^}]*\}", "", row), "&"):
            span = 1
            m = re.match(r"\s*\\multicolumn\{(\d+)\}", cell)
            if m:
                span = int(m[1])
                _, j = group(cell, m.end())  # column format
                cell, _ = group(cell, j)
            out.append({"t": self.runs(cell), "span": span})
        return out if any(c["t"] for c in out) else []


# ---------------------------------------------------------------------------
# Passages
# ---------------------------------------------------------------------------


def document() -> list[dict]:
    """Each heading's own content, as {"section", "title", "heading", "blocks"}."""
    reader = Reader()
    reader.number()
    units = []
    heading_at = re.compile(r"\\(chapter|section|subsection|subsubsection)(\*?)\{")
    for chapter, src in sources():
        counters = [0, 0]
        unit = {"section": chapter, "title": "", "heading": ""}
        pos = 0
        while True:
            m = heading_at.search(src, pos)
            blocks = reader.blocks(src[pos : m.start() if m else len(src)])
            if blocks:
                units.append({**unit, "blocks": blocks})
            if not m:
                break
            raw, pos = group(src, m.end() - 1)
            title = plain(reader.runs(raw))
            kind, starred = m[1], m[2]
            if kind == "chapter":
                unit = {"section": chapter, "title": title, "heading": ""}
            elif kind == "subsubsection" or starred:
                unit = {**unit, "heading": title}
            else:
                level = 0 if kind == "section" else 1
                counters[level] += 1
                if level == 0:
                    counters[1] = 0
                number = ".".join([chapter, *map(str, counters[: level + 1])])
                unit = {"section": number, "title": title, "heading": ""}
    if reader.table_numbers:
        raise ValueError(f"tables numbered but not read: {reader.table_numbers}")
    return units


def sentences(runs: list) -> list[list]:
    """A paragraph's runs, split after each sentence that ends in plain text."""
    out: list[list] = [[]]
    for run in runs:
        if not isinstance(run, str):
            out[-1].append(run)
            continue
        parts = re.split(r"(?<=[.!?”])\s+(?=[A-Z“(])", run)
        for k, part in enumerate(parts):
            if k:
                out.append([])
            if part:
                out[-1].append(part)
    return [s for s in out if s]


def pieces(block: dict) -> list[dict]:
    """A block, or a long paragraph, list, or table cut into parts that fit."""
    if "p" in block and words_in(block) > MAX_WORDS:
        out: list[list] = [[]]
        for sentence in sentences(block["p"]):
            size = len(plain(sentence).split())
            if out[-1] and len(plain(out[-1]).split()) + size > MAX_WORDS * 0.75:
                out.append([])
            if out[-1]:
                out[-1].append(" ")
            out[-1] += sentence
        return [
            {"p": tidy([["", r] if isinstance(r, str) else r for r in o])} for o in out
        ]
    kind = "ol" if "ol" in block else "ul" if "ul" in block else None
    if kind and words_in(block) > MAX_WORDS:
        out, items, start = [], [], block.get("start", 1)
        for item in block[kind]:
            size = sum(words_in(b) for b in item)
            held = sum(words_in(b) for i in items for b in i)
            if items and held + size > MAX_WORDS:
                out.append({kind: items, **({"start": start} if kind == "ol" else {})})
                start += len(items)
                items = []
            items.append(item)
        out.append({kind: items, **({"start": start} if kind == "ol" else {})})
        return out
    if "table" in block and len(block["table"]["rows"]) > TABLE_ROWS:
        t = block["table"]
        return [
            {"table": {**t, "rows": t["rows"][k : k + TABLE_ROWS]}}
            for k in range(0, len(t["rows"]), TABLE_ROWS)
        ]
    return [block]


def passages() -> list[dict]:
    """Whole blocks grouped into passages of MIN_WORDS to MAX_WORDS words."""
    out = []
    for unit in document():
        groups: list[list[dict]] = []
        current: list[dict] = []
        for piece in (p for b in unit["blocks"] for p in pieces(b)):
            held = sum(map(words_in, current))
            if current and held >= MIN_WORDS and held + words_in(piece) > MAX_WORDS:
                # A run-in heading stays with the text it introduces.
                carry = [current.pop()] if "h" in current[-1] else []
                groups.append(current)
                current = carry
            last = current[-1] if current else {}
            if (
                "table" in piece
                and last.get("table", {}).get("caption") == piece["table"]["caption"]
            ):
                # Two parts of one table in one passage are one table again.
                rows = last["table"]["rows"] + piece["table"]["rows"]
                current[-1] = {"table": {**last["table"], "rows": rows}}
                continue
            current.append(piece)
        if current:
            if groups and sum(map(words_in, current)) < 25:
                groups[-1] += current  # a short tail joins the passage before it
            else:
                groups.append(current)
        meta = {k: unit[k] for k in ("section", "title", "heading")}
        out += [{**meta, "blocks": g} for g in groups]
    return out
