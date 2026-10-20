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
    viewerImg.src = a.href;
    viewerImg.alt = img.alt;
    viewer.classList.toggle("is-diagram", a.href.endsWith(".svg"));
    document.getElementById("viewer-caption").textContent = img.alt;
    viewer.showModal();
  });
});
document.getElementById("viewer-close").addEventListener("click", () => viewer.close());
viewer.addEventListener("click", (e) => {
  if (e.target === viewer) viewer.close();
});
