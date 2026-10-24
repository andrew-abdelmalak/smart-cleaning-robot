// Clip autoplay and the image viewer, shared by the whole page.
// Play the silent clips only while they are on screen, and never
// automatically when the visitor prefers reduced motion.
// A clip the visitor paused stays paused.
if (!matchMedia("(prefers-reduced-motion: reduce)").matches) {
  const onScreen = new Set();
  const io = new IntersectionObserver((entries) => {
    for (const e of entries) {
      if (e.isIntersecting) {
        onScreen.add(e.target);
        if (!e.target.dataset.userPaused) e.target.play().catch(() => {});
      } else {
        onScreen.delete(e.target);
        e.target.pause();
      }
    }
  }, { threshold: 0.4 });
  document.querySelectorAll("video[data-autoplay]").forEach((v) => {
    v.addEventListener("pause", () => {
      if (onScreen.has(v)) v.dataset.userPaused = "true";
    });
    v.addEventListener("play", () => delete v.dataset.userPaused);
    io.observe(v);
  });
}

// The project video plays muted on its own; this button plays it from the
// start with its music, or mutes it again. The label says what a press does.
const film = document.getElementById("film");
const filmSound = document.getElementById("film-sound");
const filmLabel = () => {
  filmSound.querySelector("span").textContent = film.muted || film.paused ? "Play with sound" : "Mute";
};
filmSound.addEventListener("click", () => {
  if (film.muted || film.paused) {
    film.muted = false;
    film.currentTime = 0;
    film.play().catch(() => {
      film.muted = true; // the browser refused sound; keep the silent loop
      film.play().catch(() => {});
    });
  } else {
    film.muted = true;
  }
});
for (const event of ["volumechange", "play", "pause"]) film.addEventListener(event, filmLabel);
// Sound never plays out of sight: leaving the screen or the tab stops and
// mutes the film, so back in view it plays silently again.
const silence = () => {
  if (film.muted) return;
  film.pause();
  film.muted = true;
};
new IntersectionObserver(([e]) => e.isIntersecting || silence()).observe(film);
document.addEventListener("visibilitychange", () => document.hidden && silence());

// After the room's firmware runs, load the rest of the page in the
// background, in reading order, so each part is ready when the visitor gets
// to it: the videos, then the PDF viewer and the thesis it opens. Then
// ask.js loads its search model. With Save-Data on or on 2G or 3G, each
// part loads only when it is used, and a hidden tab waits until it is shown.
const net = navigator.connection;
const lowData = Boolean(net && (net.saveData || /[23]g/.test(net.effectiveType)));
const PDF_FILES = [
  "pdfjs/web/viewer.html?file=../../thesis.pdf", // the URL every thesis link opens
  "pdfjs/web/viewer.css",
  "pdfjs/web/viewer.mjs",
  "pdfjs/build/pdf.mjs",
  "pdfjs/build/pdf.worker.mjs",
  "pdfjs/web/locale/locale.json",
  "pdfjs/web/locale/en-US/viewer.ftl",
  "thesis.pdf",
];

const shown = () =>
  document.hidden ? new Promise((r) => document.addEventListener("visibilitychange", r, { once: true })).then(shown) : null;

// Read a file into the browser's cache without keeping it in memory.
async function warm(url) {
  await shown();
  const res = await fetch(url, { priority: "low" });
  const reader = res.body.getReader();
  while (!(await reader.read()).done);
}

let preloadStarted = false;
async function preloadPage() {
  if (preloadStarted) return;
  preloadStarted = true;
  await new Promise((r) => (window.requestIdleCallback ? requestIdleCallback(r, { timeout: 4000 }) : setTimeout(r, 500)));
  if (!lowData) {
    // One file at a time, so the next section is always ready first.
    // A video the visitor has already reached is loading on its own.
    const videos = [...document.querySelectorAll('video[preload="none"]')]
      .filter((v) => v.readyState === 0 && v.networkState !== HTMLMediaElement.NETWORK_LOADING)
      .map((v) => v.getAttribute("src"));
    for (const url of [...videos, ...PDF_FILES]) await warm(url).catch(() => {});
  }
  window.pagePreloaded = true; // for a script that starts listening late
  document.dispatchEvent(new Event("page-preloaded"));
}
document.addEventListener("firmware-settled", preloadPage, { once: true });
setTimeout(preloadPage, 20000); // if the room never starts

// Back to top appears once the hero, with the room, has scrolled away. It is
// a plain link to the top bar, so it also works without JavaScript; hidden, it
// is out of the tab order.
const toTop = document.getElementById("to-top");
new IntersectionObserver(([e]) => toTop.classList.toggle("is-shown", !e.isIntersecting)).observe(
  document.querySelector(".hero"),
);

// Diagrams and photos open in a viewer; Close, Esc, or a click outside closes it.
const viewer = document.getElementById("viewer");
const viewerImg = document.createElement("img");
viewerImg.alt = "";
const viewerBody = document.getElementById("viewer-body");
viewerBody.append(viewerImg);
document.querySelectorAll("a[data-view]").forEach((a) => {
  a.addEventListener("click", (e) => {
    // Ctrl, Cmd, or Shift click opens the image in a new tab as usual.
    if (!viewer.showModal || e.ctrlKey || e.metaKey || e.shiftKey) return;
    e.preventDefault();
    const img = a.querySelector("img");
    // A diagram's visible caption describes it; a photo's alt text does.
    const caption = a.closest("figure")?.querySelector("figcaption");
    const diagram = a.href.endsWith(".svg");
    const text = diagram && caption ? caption.textContent : img.alt;
    viewerImg.src = a.href;
    // The caption shows the description; the image keeps a diagram's short
    // name, and a photo, whose caption is its alt text, is not read twice.
    viewerImg.alt = diagram ? img.alt : "";
    viewer.classList.toggle("is-diagram", diagram);
    document.getElementById("viewer-caption").textContent = text;
    viewer.showModal();
    viewerBody.scrollTo(0, 0); // every image opens at its start, not where the last one was left
  });
});
document.getElementById("viewer-close").addEventListener("click", () => viewer.close());
viewer.addEventListener("click", (e) => {
  if (e.target === viewer) viewer.close();
});
