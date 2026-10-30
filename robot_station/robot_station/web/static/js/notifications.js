/* One non-modal notification, outside the header's safety controls. */
(() => {
  "use strict";
  const region = document.getElementById("commandNotice");
  const card = region.querySelector(".toast-card");
  const message = document.getElementById("toastMessage");
  const icon = document.getElementById("toastIcon");
  const close = document.getElementById("btnDismissNotice");
  const symbols = {
    success: '<path d="m4 10 4 4 8-8"/>',
    info: '<circle cx="10" cy="10" r="7.5"/><path d="M10 9v4m0-7h.01"/>',
    error: '<circle cx="10" cy="10" r="7.5"/><path d="M10 6v4m0 3h.01"/>'
  };
  const seen = new Map();
  let current = "", timer = 0, due = 0, remaining = 0, frame = 0, returnFocus = null;
  let paused = false;

  function layout() {
    frame = 0;
    if (region.hidden) return;
    const header = document.querySelector(".top").getBoundingClientRect();
    const nav = document.getElementById("navTabs").getBoundingClientRect();
    const task = document.getElementById("globalTask");
    const taskBottom = task.hidden ? 0 : task.getBoundingClientRect().bottom;
    // One predictable anchor on every page, regardless of message width or free space.
    region.style.top = Math.max(header.bottom, nav.bottom, taskBottom, 0) + 8 + "px";
    region.dataset.position = "center";
    requestAnimationFrame(syncPause);
  }
  function scheduleLayout() {
    if (!region.hidden && !frame) frame = requestAnimationFrame(layout);
  }
  function dismiss() {
    clearTimeout(timer);
    region.hidden = true;
    current = "";
    remaining = 0;
    paused = false;
    if (document.activeElement === close && returnFocus?.isConnected) returnFocus.focus({preventScroll:true});
  }
  function startTimer() {
    clearTimeout(timer);
    paused = false;
    due = performance.now() + remaining;
    timer = setTimeout(dismiss, remaining);
  }
  function pauseTimer() {
    if (paused || region.hidden) return;
    remaining = Math.max(1, due - performance.now());
    clearTimeout(timer); paused = true;
  }
  function syncPause() {
    if (region.hidden) return;
    const hold = card.matches(":hover") || card.contains(document.activeElement);
    if (hold && !paused) pauseTimer();
    else if (!hold && paused) startTimer();
  }
  function show(text, {tone = "success", pending = false, duration, force = false} = {}) {
    text = String(text || "").trim();
    if (!text) return;
    if (!["success", "info", "error"].includes(tone)) tone = "info";
    const signature = tone + ":" + pending + ":" + text;
    const now = performance.now(), lastSeen = seen.get(signature);
    seen.set(signature, now);
    if (seen.size > 64) seen.delete(seen.keys().next().value);
    // Duplicates never prolong the timer or immediately reappear after dismissal.
    if (!force && (signature === current || (region.hidden && lastSeen !== undefined && now - lastSeen < 2000))) return;
    current = signature;
    card.dataset.tone = tone;
    card.classList.toggle("toast-pending", pending);
    icon.innerHTML = pending ? "" : '<svg viewBox="0 0 20 20" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">' + symbols[tone] + '</svg>';
    region.setAttribute("role", tone === "error" ? "alert" : "status");
    region.setAttribute("aria-live", tone === "error" ? "assertive" : "polite");
    message.textContent = text;
    message.title = text;
    region.hidden = false;
    remaining = Number.isFinite(duration) && duration > 0 ? duration : pending ? 10000 : tone === "error" ? 7000 : 3500;
    layout();
    if (card.matches(":hover") || card.contains(document.activeElement)) {
      clearTimeout(timer); due = performance.now() + remaining; paused = true;
    }
    else startTimer();
  }
  close.addEventListener("click", dismiss);
  close.addEventListener("focus", event => {
    returnFocus = event.relatedTarget;
    syncPause();
  });
  close.addEventListener("blur", syncPause);
  card.addEventListener("mouseenter", syncPause);
  card.addEventListener("mouseleave", syncPause);
  window.addEventListener("resize", scheduleLayout);
  window.addEventListener("scroll", scheduleLayout, {passive:true});
  window.addEventListener("station:page", scheduleLayout);
  if (window.ResizeObserver) {
    const observer = new ResizeObserver(scheduleLayout);
    observer.observe(document.querySelector(".top"));
    observer.observe(document.getElementById("navTabs"));
    observer.observe(document.getElementById("globalTask"));
  }
  window.StationToast = {show, dismiss};
})();
