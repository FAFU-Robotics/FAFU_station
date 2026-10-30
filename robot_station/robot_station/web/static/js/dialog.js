/* Shared confirmation dialog; cancellation is the default action. */
(() => {
  "use strict";
  const dialog = document.getElementById("confirmDialog");
  let pending = null;
  function finish(confirmed) {
    const resolve = pending;
    pending = null;
    if (dialog.open) dialog.close(confirmed ? "confirm" : "cancel");
    if (resolve) resolve(confirmed);
  }
  dialog.querySelector("form").addEventListener("submit", event => {
    event.preventDefault();
    finish(event.submitter?.value === "confirm");
  });
  dialog.addEventListener("cancel", event => {event.preventDefault();finish(false);});
  dialog.addEventListener("close", () => {if (!dialog.open) finish(dialog.returnValue === "confirm");});
  window.StationDialog = {
    confirm({title = "确认操作", message, confirmText = "确认", destructive = false}) {
      if (pending) return Promise.resolve(false);
      document.getElementById("confirmTitle").textContent = title;
      document.getElementById("confirmMessage").textContent = message;
      const accept = document.getElementById("confirmAccept");
      accept.textContent = confirmText;
      accept.className = destructive ? "destructive" : "primary";
      dialog.returnValue = "cancel";
      return new Promise(resolve => {
        pending = resolve;
        dialog.showModal();
        document.getElementById("confirmCancel").focus();
      });
    }
  };
})();
