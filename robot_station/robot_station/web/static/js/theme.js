/* Apply the saved appearance before styles paint; theme changes are local UI only. */
(() => {
  "use strict";
  const key = "fafu-station-theme";
  const root = document.documentElement;
  let theme = "dark";
  try { if (localStorage.getItem(key) === "light") theme = "light"; } catch (_) {}
  root.dataset.theme = theme;

  function paintButton() {
    const button = document.getElementById("btnTheme");
    if (!button) return;
    const light = theme === "light";
    button.setAttribute("aria-pressed", String(light));
    button.title = light ? "切换到夜间主题" : "切换到日间主题";
    document.getElementById("themeLabel").textContent = light ? "日间" : "夜间";
  }
  function set(next, persist = true) {
    if (next !== "light" && next !== "dark") return;
    const changed = theme !== next;
    theme = next;
    root.dataset.theme = theme;
    if (persist) { try { localStorage.setItem(key, theme); } catch (_) {} }
    paintButton();
    if (changed) window.dispatchEvent(new CustomEvent("station:theme", {detail:{theme}}));
  }
  window.StationTheme = {get:() => theme, set};
  document.addEventListener("DOMContentLoaded", () => {
    paintButton();
    document.getElementById("btnTheme").addEventListener("click", () => set(theme === "light" ? "dark" : "light"));
  });
  window.addEventListener("storage", event => {
    if (event.key === key) set(event.newValue === "light" ? "light" : "dark", false);
  });
})();
