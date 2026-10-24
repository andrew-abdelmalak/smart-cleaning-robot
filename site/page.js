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
        e.target.muted = true; // back in view, it plays silently again
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

// Diagrams and photos open in a viewer; Close, Esc, or a click outside closes it.
const viewer = document.getElementById("viewer");
const viewerImg = document.createElement("img");
viewerImg.alt = "";
document.getElementById("viewer-body").append(viewerImg);
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
    viewerImg.alt = text;
    viewer.classList.toggle("is-diagram", diagram);
    document.getElementById("viewer-caption").textContent = text;
    viewer.showModal();
  });
});
document.getElementById("viewer-close").addEventListener("click", () => viewer.close());
viewer.addEventListener("click", (e) => {
  if (e.target === viewer) viewer.close();
});
