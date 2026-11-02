// place.js: pick-and-place goal on the robot image. Loaded AFTER the main script of index.html and uses its globals
// (S, CFG, topic, send, log, $, renderSel, armMoving, pickRunning, gotoMoving, pubPick, pubSel, select, rcamObjectAt).
// Flow: select an object (a ring appears around it) -> click where to put it (a dashed circle shows the goal, green = valid) ->
// left thumbs up (or the Confirm button) runs it, left thumbs down cancels both. No goal set: confirm drops the object in the bin as before.
(() => {
  const PC = {camFx: 554.4, camZ: 1.25, ringScale: 1.7, stateStaleMs: 3000, goalTtlS: 30};   // camera constants as in the world file
  const pubPlaceClick = topic("/place/click", "std_msgs/String");
  const pubPlaceClear = topic("/place/clear", "std_msgs/String");
  const PL = {s: null, t: 0, lastTe: 0};
  const placeOn = () => $("placeOn").checked && !$("gotoOn").checked;
  const selFresh = () => !!S.sel && Date.now() - S.selT < CFG.selectTtlS * 1000;
  const busy = () => armMoving() || pickRunning() || gotoMoving();
  const stateFresh = () => !!PL.s && Date.now() - PL.t < PC.stateStaleMs && PL.s.object === S.sel;
  const goalState = () => (stateFresh() && PL.s.goal) ? PL.s : null;
  const diam = o => Math.max(o.wx, o.wy) * PC.camFx / (PC.camZ - o.top_z) * PC.ringScale;   // ring diameter in image pixels

  function logPlace(s) {
    const f = (s.fails || []).join("; "), w = (s.warns || []).join("; ");
    if (!s.goal) { log("place goal: " + (f || "cleared")); return; }
    log("place goal " + (s.ok ? "OK" : "REFUSED") + ": x=" + s.goal.x + " y=" + s.goal.y + (s.goal.snap ? " (on top of " + s.goal.snap + ")" : "") +
        (s.what ? ", " + s.what : "") + (s.z_release ? ", release z=" + s.z_release : "") + (f ? " | " + f : "") + (w ? " | warn: " + w : ""));
  }
  topic("/place/state", "std_msgs/String", {queue_length: 1}).subscribe(m => {
    let s; try { s = JSON.parse(m.data); } catch (x) { return; }
    PL.s = s; PL.t = Date.now();
    if (s.te && s.te !== PL.lastTe) { PL.lastTe = s.te; logPlace(s); }
    render();
  });

  function mk(id, cls) {
    let e = $(id);
    if (!e) { e = document.createElement("div"); e.id = id; e.className = cls; $("ovl").appendChild(e); }
    return e;
  }
  function put(e, u, v, dpx) {
    const im = $("rcam"), iw = im.naturalWidth || 640, ih = im.naturalHeight || 480;
    e.style.left = (u / iw * 100) + "%"; e.style.top = (v / ih * 100) + "%"; e.style.width = (dpx / iw * 100) + "%"; e.style.display = "block";
  }

  function render() {
    const fresh = selFresh(), o = fresh ? S.det[S.sel] : null, seen = !!o && Date.now() - S.detT < CFG.detStaleMs;
    const ring = mk("selring", "ring sel"), gm = mk("goalmk", "ring goal"), g = goalState();
    if (seen) put(ring, o.px[0], o.px[1], diam(o)); else ring.style.display = "none";
    if (g && seen) { put(gm, g.goal.u, g.goal.v, diam(o)); gm.className = "ring goal " + (!g.ok ? "no" : (g.warns.length ? "warn" : "ok")); } else gm.style.display = "none";

    const fs = stateFresh() ? PL.s : null;
    $("pobj").textContent = fresh ? S.sel : "-";
    $("pgoal").textContent = g ? "x=" + g.goal.x + " y=" + g.goal.y + (g.goal.snap ? " (on " + g.goal.snap + ")" : "") : "-";
    $("pwhat").textContent = g && g.what ? g.what : "-";
    $("pz").textContent = g && g.z_release ? g.z_release + " m (fingertip)" : "-";
    $("pchk").textContent = g ? (g.ok ? (g.warns.length ? "OK with warnings" : "OK") : "REFUSED") : (fs && fs.te && (fs.fails || []).length ? "REFUSED" : "-");
    $("pmsg").textContent = fs ? ((fs.fails || []).concat((fs.warns || []).map(w => "warn: " + w))).join(" | ") : "";

    let t;
    if ($("gotoOn").checked) t = "Go-to mode: click a spot to add a waypoint (max 2), then Go.";
    else if (!fresh) t = "Click near an object to select it: a ring appears around it.";
    else if (!placeOn()) t = "Selected " + S.sel + ". Left thumbs up or Confirm drops it in the bin. Left thumbs down cancels.";
    else if (!seen) t = "Selected " + S.sel + " but it is not visible (is the arm at home?). Left thumbs down cancels.";
    else if (fs && fs.pending) t = "Checking the goal...";
    else if (!g && fs && fs.te && (fs.fails || []).length) t = "Goal check failed: " + fs.fails.join("; ") + ". Click another spot, or left thumbs down to cancel.";
    else if (!g) t = "Selected " + S.sel + ". Click where to put it, or left thumbs up to drop it in the bin. Left thumbs down cancels.";
    else if (!g.ok) t = "Goal refused: " + g.fails.join("; ") + ". Click another spot, or left thumbs down to cancel.";
    else t = "Goal: " + g.what + ", released 5 cm above the surface (fingertip z " + g.z_release + " m)" + (g.warns.length ? " - warning: " + g.warns.join("; ") : "") + ". Left thumbs up runs it, left thumbs down cancels.";
    $("rcap").textContent = t;
  }
  setInterval(render, 300);

  // a click on the robot image sets the goal while an object is selected (otherwise the original handler selects an object)
  $("rcam").parentElement.addEventListener("click", ev => {
    if (ev.target !== $("rcam") || !placeOn() || !selFresh()) return;
    ev.stopPropagation();
    if (busy()) { log("place click ignored: robot busy"); return; }
    const im = $("rcam"), r = im.getBoundingClientRect(), w = im.naturalWidth || 640, h = im.naturalHeight || 480;
    const u = Math.round((ev.clientX - r.left) * w / r.width * 10) / 10, v = Math.round((ev.clientY - r.top) * h / r.height * 10) / 10;
    send(pubPlaceClick, JSON.stringify({object: S.sel, u: u, v: v}));
    log("place click at pixel (" + Math.round(u) + "," + Math.round(v) + ") for " + S.sel);
  }, true);

  // Confirm button: with a checked goal it sends the goal too; with no goal the original handler drops the object in the bin
  $("confirm").parentElement.addEventListener("click", ev => {
    if (ev.target !== $("confirm")) return;
    const s = stateFresh() ? PL.s : null;      // a goal was clicked for the selected object (pending, refused or checked)
    if (!s || !s.te) return;                   // no goal clicked: the original handler drops the object in the bin
    ev.stopPropagation();
    if (!selFresh()) { log("confirm ignored: nothing selected in the last " + CFG.selectTtlS + " s"); return; }
    if (busy()) { log("confirm ignored: robot busy"); return; }
    if (s.pending) { log("confirm ignored: the goal is still being checked"); return; }
    const g = s;
    if (!g.goal || !g.ok) { log("confirm ignored: place goal refused (" + ((g.fails || []).join("; ") || "no reason given") + ")"); return; }
    if (Date.now() / 1000 - g.te > PC.goalTtlS) { log("confirm ignored: place goal expired, click again"); return; }
    send(pubPick, JSON.stringify({object: S.sel, goal: {x: g.goal.x, y: g.goal.y}}));
    log("sent /pick_place_command " + S.sel + " -> x=" + g.goal.x + " y=" + g.goal.y + " (" + g.what + ")");
    send(pubSel, ""); send(pubPlaceClear, "clear"); S.sel = null; renderSel(); render();
  }, true);

  $("pClear").addEventListener("click", () => { send(pubPlaceClear, "clear"); log("sent /place/clear"); });

  // selecting (again) forgets the old goal; with a selection present, dwell on the robot image must not re-select
  const origSelect = window.select;
  window.select = name => { origSelect(name); send(pubPlaceClear, "clear"); };
  const origAt = window.rcamObjectAt;
  window.rcamObjectAt = (cx, cy) => (placeOn() && selFresh()) ? null : origAt(cx, cy);
})();
