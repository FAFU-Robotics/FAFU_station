/* Plain decimal input: no scientific notation, coercion of blanks, or partial parses. */
(() => {
  "use strict";
  const decimal = /^-?(?:\d+(?:\.\d*)?|\.\d+)$/;
  const partial = /^-?\d*(?:\.\d*)?$/;
  function clear(input) {input.setCustomValidity("");input.removeAttribute("aria-invalid");}
  function read(input, {label = "数值", min = -Infinity, max = Infinity, integer = false} = {}) {
    const raw = input.value.trim(), value = Number(raw);
    let error = "";
    if (!raw || !decimal.test(raw) || !Number.isFinite(value)) error = label+"请输入普通十进制数字";
    else if (integer && !Number.isInteger(value)) error = label+"必须为整数";
    else if (value < min || value > max) error = label+"须在 "+min+"–"+max+" 之间";
    if (error) {input.setCustomValidity(error);input.setAttribute("aria-invalid","true");throw new Error(error);}
    clear(input); return value;
  }
  function bind(input) {
    input.type = "text"; input.inputMode = "decimal"; input.autocomplete = "off";
    input.dataset.decimal = "true";
    input.addEventListener("input", () => clear(input));
  }
  const numeric = input => input instanceof HTMLInputElement && (input.type === "number" || input.dataset.decimal === "true");
  function insertionAllowed(input, text) {
    if (/[^0-9.\-]/.test(text)) return false;
    if (Number(input.getAttribute("min")) >= 0 && input.hasAttribute("min") && text.includes("-")) return false;
    if (input.type === "number") return partial.test(text);
    const start = input.selectionStart ?? input.value.length, end = input.selectionEnd ?? start;
    return partial.test(input.value.slice(0,start)+text+input.value.slice(end));
  }
  document.addEventListener("keydown", event => {
    if (!numeric(event.target) || event.ctrlKey || event.metaKey || event.altKey || event.isComposing || event.key.length !== 1) return;
    if (!insertionAllowed(event.target,event.key)) event.preventDefault();
  }, true);
  document.addEventListener("beforeinput", event => {
    if (numeric(event.target) && !event.isComposing && typeof event.data === "string" && !insertionAllowed(event.target,event.data)) event.preventDefault();
  }, true);
  document.addEventListener("paste", event => {
    const input = event.target;
    if (!numeric(input) || !event.clipboardData) return;
    const text = event.clipboardData.getData("text").trim();
    event.preventDefault();
    if (!decimal.test(text) || !insertionAllowed(input,text)) return;
    if (input.type === "number") input.value = text;
    else input.setRangeText(text,input.selectionStart,input.selectionEnd,"end");
    input.dispatchEvent(new Event("input",{bubbles:true}));
  }, true);
  window.StationNumbers = {read,bind,clear};
})();
