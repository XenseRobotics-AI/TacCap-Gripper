// TacCap 运行地图 —— 渲染与交互（只读 window.MAP_DATA / window.MAP_SCENARIOS）
(function () {
  "use strict";
  const D = window.MAP_DATA;
  const S = window.MAP_SCENARIOS;
  const NS = "http://www.w3.org/2000/svg";
  const NODE_W = 180, NODE_H = 56, BAND_NODE_H = 64;

  const KIND_TEXT = { sync: "同步调用", async: "异步 / 回调", data: "数据流", manual: "人工操作", build: "构建 / 校验", branch: "失败与分支" };
  const CONF_TEXT = { confirmed: "已确认", inferred: "有依据的推断", unverified: "尚未确认" };
  const NODE_KIND_TEXT = { core: "本仓库代码", external: "进程外 / 外部", optional: "可选组件", manual: "人工", asset: "仓库资产" };

  const nodeById = new Map(D.nodes.map(n => [n.id, n]));
  const bandById = new Map(D.bands.map(b => [b.id, b]));
  const edgeById = new Map(D.edges.map(e => [e.id, e]));
  const objById = new Map(D.objects.map(o => [o.id, o]));

  const state = {
    collapsed: Object.fromEntries(D.bands.map(b => [b.id, !!b.collapsed])),
    scen: -1, step: 0,
    selected: null,
    allLabels: false,
    autoExpand: true,
    view: { x: 0, y: 0, k: 1 },
    tab: "scen"
  };

  // ---------------- self-check ----------------
  function selfCheck() {
    const errs = [];
    D.nodes.forEach(n => { if (!bandById.has(n.band)) errs.push(`节点 ${n.id} 的分层 ${n.band} 不存在`);
      (n.objects || []).forEach(o => { if (!objById.has(o)) errs.push(`节点 ${n.id} 引用了不存在的对象 ${o}`); }); });
    D.edges.forEach(e => { if (!nodeById.has(e.from)) errs.push(`连线 ${e.id} 的起点 ${e.from} 不存在`);
      if (!nodeById.has(e.to)) errs.push(`连线 ${e.id} 的终点 ${e.to} 不存在`); });
    const ids = new Set(); [...D.nodes, ...D.edges].forEach(x => { if (ids.has(x.id)) errs.push(`重复 id ${x.id}`); ids.add(x.id); });
    let steps = 0;
    S.forEach(sc => sc.steps.forEach((st, i) => { steps++;
      st.nodes.forEach(n => { if (!nodeById.has(n)) errs.push(`场景 ${sc.id} 第 ${i + 1} 步：节点 ${n} 不存在`); });
      st.edges.forEach(e => { if (!edgeById.has(e)) errs.push(`场景 ${sc.id} 第 ${i + 1} 步：连线 ${e} 不存在`); });
      if (!KIND_TEXT[st.kind]) errs.push(`场景 ${sc.id} 第 ${i + 1} 步：未知类型 ${st.kind}`); }));
    errs.forEach(e => console.error("[map self-check]", e));
    return { errs, summary: `${D.nodes.length} 个节点 · ${D.edges.length} 条连线 · ${S.length} 个场景 / ${steps} 步` };
  }
  const check = selfCheck();

  // ---------------- helpers ----------------
  const $ = sel => document.querySelector(sel);
  function el(tag, attrs, ...kids) {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs || {})) {
      if (k === "class") e.className = v; else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
      else if (v !== undefined && v !== null && v !== false) e.setAttribute(k, v);
    }
    kids.flat().forEach(c => { if (c != null) e.append(c.nodeType ? c : document.createTextNode(String(c))); });
    return e;
  }
  function sv(tag, attrs) {
    const e = document.createElementNS(NS, tag);
    for (const [k, v] of Object.entries(attrs || {})) e.setAttribute(k, v);
    return e;
  }
  // Rough text width so long labels can be squeezed into the node instead of spilling out.
  function estWidth(text, px, mono) {
    let w = 0; for (const ch of text) w += /[\u2e80-\uffff]/.test(ch) ? px : px * (mono ? 0.6 : 0.56);
    return w;
  }
  function fitText(t, text, px, mono, max) {
    t.textContent = text;
    if (estWidth(text, px, mono) > max) { t.setAttribute("textLength", max); t.setAttribute("lengthAdjust", "spacingAndGlyphs"); }
  }
  function chip(text, cls) { return el("span", { class: "chip " + (cls || "") }, text); }
  function confChip(c) { c = c || "confirmed"; return chip(CONF_TEXT[c], "conf-" + c); }

  function evidenceList(ev) {
    if (!ev || !ev.length) return null;
    return el("ul", { class: "evidence" }, ev.map(([path, loc, note]) => {
      const ref = loc ? `${path} · ${loc}` : path;
      const btn = el("button", { class: "ev-ref", type: "button", title: "复制路径" }, ref);
      btn.addEventListener("click", () => copy(path, btn));
      return el("li", {}, btn, note ? el("span", { class: "ev-note" }, note) : null);
    }));
  }
  function copy(text, btn) {
    const done = () => { btn.classList.add("copied"); setTimeout(() => btn.classList.remove("copied"), 900); };
    try { navigator.clipboard.writeText(text).then(done, () => selectText(btn)); } catch (_) { selectText(btn); }
  }
  function selectText(node) { const r = document.createRange(); r.selectNodeContents(node); const s = getSelection(); s.removeAllRanges(); s.addRange(r); }

  // ---------------- layout model ----------------
  function rep(id) { const n = nodeById.get(id); return state.collapsed[n.band] ? "band:" + n.band : id; }
  function bandRect(b) { return { x: 20, y: b.y + (b.h - BAND_NODE_H) / 2, w: D.canvas.w - 40, h: BAND_NODE_H }; }
  function rectOf(id) {
    if (id.startsWith("band:")) return bandRect(bandById.get(id.slice(5)));
    const n = nodeById.get(id); return { x: n.x, y: n.y, w: NODE_W, h: NODE_H };
  }
  function visibleEdges() {
    const agg = new Map();
    D.edges.forEach(e => {
      const a = rep(e.from), b = rep(e.to);
      if (a === b) return;
      const key = a + "|" + b + "|" + e.kind;
      const fn = nodeById.get(e.from), tn = nodeById.get(e.to);
      if (!agg.has(key)) agg.set(key, { key, from: a, to: b, kind: e.kind, ids: [], ax: [], bx: [], labels: [], conf: e.conf });
      const g = agg.get(key);
      g.ids.push(e.id); g.ax.push(fn.x + NODE_W / 2); g.bx.push(tn.x + NODE_W / 2); g.labels.push(e.label);
    });
    const list = [...agg.values()];
    const keys = new Set(list.map(g => g.from + "|" + g.to));
    list.forEach(g => {
      const avg = a => a.reduce((s, v) => s + v, 0) / a.length;
      g.axv = avg(g.ax); g.bxv = avg(g.bx);
      g.pair = keys.has(g.to + "|" + g.from);
      g.label = g.labels.length === 1 ? g.labels[0] : `${g.labels[0]} 等 ${g.labels.length} 条`;
    });
    return list;
  }
  function clip(r, px, py, tx, ty) {
    const cx = px, cy = py, dx = tx - cx, dy = ty - cy;
    if (dx === 0 && dy === 0) return [cx, cy];
    // ray from (cx,cy) leaving the rect
    let t = Infinity;
    if (dx > 0) t = Math.min(t, (r.x + r.w - cx) / dx); if (dx < 0) t = Math.min(t, (r.x - cx) / dx);
    if (dy > 0) t = Math.min(t, (r.y + r.h - cy) / dy); if (dy < 0) t = Math.min(t, (r.y - cy) / dy);
    return [cx + dx * t, cy + dy * t];
  }

  // ---------------- SVG rendering ----------------
  const svg = $("#map");
  const vp = sv("g", { id: "vp" });
  function buildDefs() {
    const defs = sv("defs");
    ["sync", "async", "data", "manual", "build"].forEach(k => {
      const m = sv("marker", { id: "arr-" + k, viewBox: "0 0 10 10", refX: "9", refY: "5", markerWidth: "7", markerHeight: "7", orient: "auto-start-reverse" });
      m.append(sv("path", { d: "M0,0 L10,5 L0,10 z", class: "arrow k-" + k }));
      defs.append(m);
    });
    const m = sv("marker", { id: "arr-hl", viewBox: "0 0 10 10", refX: "9", refY: "5", markerWidth: "7", markerHeight: "7", orient: "auto-start-reverse" });
    m.append(sv("path", { d: "M0,0 L10,5 L0,10 z", class: "arrow hl" }));
    defs.append(m);
    svg.append(defs);
  }

  function stepNodeIds(st) {
    const ids = new Set(st.nodes);
    st.edges.forEach(e => { const ed = edgeById.get(e); if (ed) { ids.add(ed.from); ids.add(ed.to); } });
    return [...ids];
  }
  function currentStep() { return state.scen >= 0 ? S[state.scen].steps[state.step] : null; }

  function render() {
    vp.textContent = "";
    const step = currentStep();
    const hlNodes = new Set(), hlEdges = new Set();
    if (step) {
      step.nodes.forEach(n => hlNodes.add(rep(n)));
      step.edges.forEach(e => { hlEdges.add(e); const ed = edgeById.get(e); hlNodes.add(rep(ed.from)); hlNodes.add(rep(ed.to)); });
    }
    svg.classList.toggle("focus", !!step);

    // bands
    D.bands.forEach(b => {
      const g = sv("g", { class: "band" + (state.collapsed[b.id] ? " is-collapsed" : "") });
      g.append(sv("rect", { x: 10, y: b.y, width: D.canvas.w - 20, height: b.h, rx: 10, class: "band-bg" }));
      if (!state.collapsed[b.id]) {
        const t = sv("text", { x: 24, y: b.y + 18, class: "band-title" });
        t.textContent = "▾ " + b.title;
        t.addEventListener("click", ev => { ev.stopPropagation(); toggleBand(b.id); });
        t.setAttribute("tabindex", "0");
        t.addEventListener("keydown", ev => { if (ev.key === "Enter") toggleBand(b.id); });
        g.append(t);
      }
      vp.append(g);
    });

    // edges
    const eg = sv("g", { class: "edges" });
    const lg = sv("g", { class: "labels" });
    visibleEdges().forEach(g => {
      const ra = rectOf(g.from), rb = rectOf(g.to);
      const pa = [g.from.startsWith("band:") ? g.axv : ra.x + ra.w / 2, ra.y + ra.h / 2];
      const pb = [g.to.startsWith("band:") ? g.bxv : rb.x + rb.w / 2, rb.y + rb.h / 2];
      const [x1, y1] = clip(ra, pa[0], pa[1], pb[0], pb[1]);
      const [x2, y2] = clip(rb, pb[0], pb[1], pa[0], pa[1]);
      const mx = (x1 + x2) / 2, my = (y1 + y2) / 2;
      const len = Math.hypot(x2 - x1, y2 - y1) || 1;
      const bend = g.pair ? 16 : 0;
      const nx = -(y2 - y1) / len * bend, ny = (x2 - x1) / len * bend;
      const cx = mx + nx * 2, cy = my + ny * 2;
      const d = `M${x1},${y1} Q${cx},${cy} ${x2},${y2}`;
      const isHl = g.ids.some(id => hlEdges.has(id));
      const touchesSel = state.selected && (g.from === rep(state.selected) || g.to === rep(state.selected));
      const p = sv("path", { d, class: `edge k-${g.kind}${isHl ? " hl" : ""}${touchesSel ? " sel" : ""}${g.conf === "unverified" ? " unverified" : ""}`,
        "marker-end": `url(#${isHl ? "arr-hl" : "arr-" + g.kind})` });
      const title = sv("title"); title.textContent = `${KIND_TEXT[g.kind]}：${g.labels.join("；")}`; p.append(title);
      eg.append(p);
      if (isHl || touchesSel || state.allLabels) {
        const lx = 0.25 * x1 + 0.5 * cx + 0.25 * x2, ly = 0.25 * y1 + 0.5 * cy + 0.25 * y2;
        const t = sv("text", { x: lx, y: ly + 4, class: `elabel k-${g.kind}${isHl ? " hl" : ""}`, "text-anchor": "middle" });
        t.textContent = g.label;
        lg.append(t);
      }
    });
    vp.append(eg);

    // nodes
    const ng = sv("g", { class: "nodes" });
    D.bands.forEach(b => {
      if (!state.collapsed[b.id]) return;
      const r = bandRect(b), id = "band:" + b.id;
      const g = sv("g", { class: "node bandnode" + (hlNodes.has(id) ? " hl" : "") + (state.selected === id ? " selected" : ""), tabindex: "0", role: "button" });
      g.append(sv("rect", { x: r.x, y: r.y, width: r.w, height: r.h, rx: 8 }));
      const t = sv("text", { x: r.x + 18, y: r.y + 26, class: "n-title" }); t.textContent = "▸ " + b.title; g.append(t);
      const kids = D.nodes.filter(n => n.band === b.id).map(n => n.title).join(" · ");
      const s = sv("text", { x: r.x + 18, y: r.y + 47, class: "n-sub" }); fitText(s, kids, 11, true, r.w - 36); g.append(s);
      const tip = sv("title"); tip.textContent = "点击展开：" + b.desc; g.append(tip);
      g.dataset.id = id;
      ng.append(g);
    });
    D.nodes.forEach(n => {
      if (state.collapsed[n.band]) return;
      const g = sv("g", { class: `node nk-${n.kind}` + (hlNodes.has(n.id) ? " hl" : "") + (state.selected === n.id ? " selected" : ""), tabindex: "0", role: "button" });
      g.append(sv("rect", { x: n.x, y: n.y, width: NODE_W, height: NODE_H, rx: 7 }));
      const t = sv("text", { x: n.x + 12, y: n.y + 23, class: "n-title" }); fitText(t, n.title, 13.5, false, NODE_W - 30); g.append(t);
      const s = sv("text", { x: n.x + 12, y: n.y + 42, class: "n-sub" }); fitText(s, n.sub, 11, true, NODE_W - 22); g.append(s);
      if (n.conf && n.conf !== "confirmed") g.append(sv("circle", { cx: n.x + NODE_W - 12, cy: n.y + 12, r: 5, class: "cdot conf-" + n.conf }));
      g.dataset.id = n.id;
      ng.append(g);
    });
    vp.append(ng);
    vp.append(lg);
    applyView();
  }

  // ---------------- view: pan / zoom ----------------
  function applyView() { vp.setAttribute("transform", `translate(${state.view.x},${state.view.y}) scale(${state.view.k})`); }
  function svgSize() { const r = svg.getBoundingClientRect(); return { w: r.width, h: r.height }; }
  const reduceMotion = window.matchMedia && matchMedia("(prefers-reduced-motion: reduce)").matches;
  let anim = null;
  function animateTo(target) {
    if (anim) cancelAnimationFrame(anim);
    if (reduceMotion) { state.view = target; applyView(); return; }
    const from = { ...state.view }, t0 = performance.now(), dur = 380;
    const tick = now => {
      const t = Math.max(0, Math.min(1, (now - t0) / dur)), e = 1 - Math.pow(1 - t, 3);
      state.view = { x: from.x + (target.x - from.x) * e, y: from.y + (target.y - from.y) * e, k: from.k + (target.k - from.k) * e };
      applyView(); if (t < 1) anim = requestAnimationFrame(tick);
    };
    anim = requestAnimationFrame(tick);
  }
  function fitBox(b, maxK, animate) {
    const { w, h } = svgSize(); if (!w || !h) return;
    const pad = 28;
    const k = Math.max(0.15, Math.min(maxK || 2, (w - pad * 2) / b.w, (h - pad * 2) / b.h));
    const target = { k, x: (w - b.w * k) / 2 - b.x * k, y: (h - b.h * k) / 2 - b.y * k };
    animate ? animateTo(target) : (state.view = target, applyView());
  }
  function fitAll(animate) { fitBox({ x: 0, y: 0, w: D.canvas.w, h: D.canvas.h }, 1.5, animate); }
  function fitIds(ids) {
    if (!ids.length) return;
    let x1 = Infinity, y1 = Infinity, x2 = -Infinity, y2 = -Infinity;
    ids.forEach(id => { const r = rectOf(rep(id)); x1 = Math.min(x1, r.x); y1 = Math.min(y1, r.y); x2 = Math.max(x2, r.x + r.w); y2 = Math.max(y2, r.y + r.h); });
    fitBox({ x: x1 - 40, y: y1 - 40, w: x2 - x1 + 80, h: y2 - y1 + 80 }, 1.25, true);
  }
  function zoomAt(f, cx, cy) {
    const v = state.view, k = Math.max(0.15, Math.min(3, v.k * f));
    state.view = { k, x: cx - (cx - v.x) * (k / v.k), y: cy - (cy - v.y) * (k / v.k) };
    applyView();
  }
  svg.addEventListener("wheel", ev => {
    ev.preventDefault();
    const r = svg.getBoundingClientRect();
    zoomAt(Math.exp(-ev.deltaY * 0.0015), ev.clientX - r.left, ev.clientY - r.top);
  }, { passive: false });

  const ptrs = new Map(); let drag = null, pinch = null;
  svg.addEventListener("pointerdown", ev => {
    try { svg.setPointerCapture(ev.pointerId); } catch (_) { /* synthetic or already-released pointer */ }
    ptrs.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });
    if (ptrs.size === 1) drag = { x: ev.clientX, y: ev.clientY, vx: state.view.x, vy: state.view.y, moved: false, target: ev.target };
    if (ptrs.size === 2) { const [a, b] = [...ptrs.values()]; pinch = { d: Math.hypot(a.x - b.x, a.y - b.y), k: state.view.k }; drag = null; }
  });
  svg.addEventListener("pointermove", ev => {
    if (!ptrs.has(ev.pointerId)) return;
    ptrs.set(ev.pointerId, { x: ev.clientX, y: ev.clientY });
    if (pinch && ptrs.size === 2) {
      const [a, b] = [...ptrs.values()], r = svg.getBoundingClientRect();
      const d = Math.hypot(a.x - b.x, a.y - b.y);
      zoomAt((pinch.k * d / pinch.d) / state.view.k, (a.x + b.x) / 2 - r.left, (a.y + b.y) / 2 - r.top);
      return;
    }
    if (!drag) return;
    const dx = ev.clientX - drag.x, dy = ev.clientY - drag.y;
    if (!drag.moved && Math.hypot(dx, dy) > 4) { drag.moved = true; svg.classList.add("panning"); }
    if (drag.moved) { state.view.x = drag.vx + dx; state.view.y = drag.vy + dy; applyView(); }
  });
  function endPtr(ev) {
    ptrs.delete(ev.pointerId);
    if (ptrs.size < 2) pinch = null;
    if (drag && ptrs.size === 0) {
      if (!drag.moved) { const g = drag.target.closest && drag.target.closest(".node"); if (g) activate(g.dataset.id); }
      drag = null; svg.classList.remove("panning");
    }
  }
  svg.addEventListener("pointerup", endPtr);
  svg.addEventListener("pointercancel", endPtr);
  svg.addEventListener("keydown", ev => {
    if (ev.key !== "Enter" && ev.key !== " ") return;
    const g = ev.target.closest && ev.target.closest(".node"); if (g) { ev.preventDefault(); activate(g.dataset.id); }
  });

  // ---------------- actions ----------------
  function toggleBand(id, force) {
    state.collapsed[id] = force !== undefined ? force : !state.collapsed[id];
    render();
  }
  function activate(id) {
    if (id.startsWith("band:")) { const b = id.slice(5); state.selected = null; toggleBand(b, false); showBand(b); return; }
    state.selected = id; setTab("node"); renderNode(); render();
  }
  function setTab(t) {
    state.tab = t;
    document.querySelectorAll(".tab").forEach(b => b.setAttribute("aria-selected", b.dataset.tab === t ? "true" : "false"));
    document.querySelectorAll(".pane").forEach(p => { p.hidden = p.id !== "pane-" + t; });
  }

  // ---------------- panel: scenarios ----------------
  function renderScenarioPicker() {
    const box = $("#scen-pick"); box.textContent = "";
    S.forEach((sc, i) => box.append(el("button", { type: "button", class: "scen-btn" + (state.scen === i ? " on" : ""), onclick: () => startScenario(i) },
      el("span", { class: "scen-n" }, `${sc.steps.length} 步`), sc.title)));
  }
  function startScenario(i, step) {
    state.scen = i; state.step = step || 0; state.selected = null; setTab("scen"); goStep();
  }
  function goStep() {
    const st = currentStep();
    const ids = st ? stepNodeIds(st) : [];
    if (st && state.autoExpand) ids.forEach(n => { state.collapsed[nodeById.get(n).band] = false; });
    renderScenarioPicker(); renderScenario(); render();
    if (st) fitIds(ids);
  }
  function renderScenario() {
    const box = $("#scen-body"); box.textContent = "";
    if (state.scen < 0) {
      box.append(el("p", { class: "hint" }, "选一个场景，用“上一步 / 下一步”（或键盘 ← →）逐步演示。每一步会高亮涉及的模块和连线，地图自动对准它们。"));
      return;
    }
    const sc = S[state.scen], st = sc.steps[state.step];
    box.append(
      el("dl", { class: "tr" }, el("dt", {}, "触发"), el("dd", {}, sc.trigger), el("dt", {}, "结果"), el("dd", {}, sc.result)),
      el("div", { class: "stepper" },
        el("button", { type: "button", id: "prev", class: "btn", disabled: state.step === 0 ? "" : null, onclick: () => { state.step--; goStep(); } }, "← 上一步"),
        el("span", { class: "count" }, `${state.step + 1} / ${sc.steps.length}`),
        el("button", { type: "button", id: "next", class: "btn primary", disabled: state.step === sc.steps.length - 1 ? "" : null, onclick: () => { state.step++; goStep(); } }, "下一步 →")),
      el("article", { class: "step-card k-" + st.kind },
        el("div", { class: "step-meta" }, chip(KIND_TEXT[st.kind], "kind-" + st.kind), confChip(st.conf)),
        el("h3", {}, st.title),
        el("p", {}, st.text),
        evidenceList(st.evidence)),
      el("ol", { class: "steps" }, sc.steps.map((s, i) =>
        el("li", { class: (i === state.step ? "cur " : "") + "k-" + s.kind },
          el("button", { type: "button", onclick: () => { state.step = i; goStep(); } },
            el("span", { class: "dot" }), s.title)))),
      el("button", { type: "button", class: "btn ghost", onclick: exitScenario }, "退出演示")
    );
  }
  function exitScenario() { state.scen = -1; state.step = 0; renderScenarioPicker(); renderScenario(); render(); }

  // ---------------- panel: node / band ----------------
  function showBand(id) {
    const b = bandById.get(id); setTab("node");
    const box = $("#pane-node"); box.textContent = "";
    box.append(el("h2", {}, b.title), el("p", {}, b.desc),
      el("p", { class: "hint" }, "已展开。点击分层左上角的标题可以再折叠。"),
      el("ul", { class: "rel" }, D.nodes.filter(n => n.band === id).map(n =>
        el("li", {}, el("button", { type: "button", class: "link", onclick: () => activate(n.id) }, n.title), el("span", { class: "muted" }, " " + n.sub)))));
  }
  function renderNode() {
    const n = nodeById.get(state.selected); const box = $("#pane-node"); box.textContent = "";
    if (!n) { box.append(el("p", { class: "hint" }, "点击图中的任一模块查看职责、关联对象、上下游关系和代码位置。")); return; }
    const out = D.edges.filter(e => e.from === n.id), inn = D.edges.filter(e => e.to === n.id);
    const relItem = (e, other) => el("li", {}, chip(KIND_TEXT[e.kind], "kind-" + e.kind), " ",
      el("button", { type: "button", class: "link", onclick: () => activate(other) }, nodeById.get(other).title), el("span", { class: "muted" }, "：" + e.label));
    const inScen = [];
    S.forEach((sc, i) => { const j = sc.steps.findIndex(s => stepNodeIds(s).includes(n.id)); if (j >= 0) inScen.push([i, j, sc]); });
    box.append(
      el("div", { class: "node-head" }, el("h2", {}, n.title), el("code", {}, n.sub)),
      el("div", { class: "step-meta" }, chip(NODE_KIND_TEXT[n.kind] || n.kind, "nk"), chip(bandById.get(n.band).title, "nk"), confChip(n.conf)),
      el("h4", {}, "职责"), el("p", {}, n.desc),
      n.note ? el("p", { class: "need" }, el("strong", {}, "需要的证据："), n.note) : null,
      (n.objects && n.objects.length) ? el("div", {}, el("h4", {}, "关联业务对象"),
        el("div", { class: "chips" }, n.objects.map(o => el("button", { type: "button", class: "chip link-chip", onclick: () => { setTab("obj"); const t = document.getElementById("obj-" + o); if (t) { t.scrollIntoView({ block: "start" }); t.classList.add("flash"); setTimeout(() => t.classList.remove("flash"), 1200); } } }, objById.get(o).name)))) : null,
      out.length ? el("div", {}, el("h4", {}, `调用 / 输出（${out.length}）`), el("ul", { class: "rel" }, out.map(e => relItem(e, e.to)))) : null,
      inn.length ? el("div", {}, el("h4", {}, `被调用 / 输入（${inn.length}）`), el("ul", { class: "rel" }, inn.map(e => relItem(e, e.from)))) : null,
      inScen.length ? el("div", {}, el("h4", {}, "出现在场景"), el("ul", { class: "rel" }, inScen.map(([i, j, sc]) =>
        el("li", {}, el("button", { type: "button", class: "link", onclick: () => startScenario(i, j) }, sc.title), el("span", { class: "muted" }, ` · 从第 ${j + 1} 步开始`))))) : null,
      el("h4", {}, "代码位置"), evidenceList(n.evidence)
    );
  }

  // ---------------- panel: objects / trust ----------------
  function renderObjects() {
    const box = $("#pane-obj"); box.textContent = "";
    box.append(el("p", { class: "hint" }, "这些对象在多个模块之间流动。“状态”列出它在运行中可能处于的形态。"));
    D.objects.forEach(o => box.append(el("article", { class: "obj", id: "obj-" + o.id },
      el("h3", {}, o.name), el("code", { class: "where" }, o.where),
      el("p", {}, o.life), el("p", { class: "muted" }, o.rw),
      o.states.length ? el("div", { class: "chips" }, o.states.map(s => chip(s, "st"))) : null,
      evidenceList(o.evidence))));
  }
  function renderTrust() {
    const box = $("#pane-trust"); box.textContent = "";
    box.append(
      el("p", { class: "base" }, D.baseline),
      el("div", { class: "legend-conf" },
        el("div", {}, confChip("confirmed"), " 在代码或配置里读到；节点上不加标记"),
        el("div", {}, confChip("inferred"), " 有依据但不是直接读到；节点右上角黄点"),
        el("div", {}, confChip("unverified"), " 静态代码无法确认；节点右上角红点，连线为浅色")),
      el("h3", {}, "文档与实现不一致（以实现为准）"),
      el("ul", { class: "disc" }, D.discrepancies.map(d => el("li", {}, el("strong", {}, d.title),
        el("p", {}, el("span", { class: "tag" }, "文档"), d.doc), el("p", {}, el("span", { class: "tag code" }, "代码"), d.code)))),
      el("h3", {}, "推断与待核实"),
      el("ul", { class: "disc" }, D.open_questions.map(q => el("li", {}, el("div", { class: "step-meta" }, confChip(q.conf)), el("strong", {}, q.title),
        el("p", {}, q.text), el("p", { class: "need" }, el("strong", {}, "需要："), q.need)))),
      el("h3", {}, "数据自检"),
      check.errs.length
        ? el("ul", { class: "errs" }, check.errs.map(e => el("li", {}, e)))
        : el("p", { class: "ok" }, "通过：" + check.summary + "。所有连线端点与场景引用都存在。")
    );
  }

  // ---------------- legend & toolbar ----------------
  function renderLegend() {
    const box = $("#legend"); box.textContent = "";
    ["sync", "async", "data", "manual", "build"].forEach(k => {
      const s = sv("svg", { width: 34, height: 10, "aria-hidden": "true" });
      s.append(sv("line", { x1: 1, y1: 5, x2: 33, y2: 5, class: "edge k-" + k }));
      box.append(el("span", { class: "lg" }, s, KIND_TEXT[k]));
    });
    box.append(el("span", { class: "lg" }, el("i", { class: "sw sw-ext" }), "进程外 / 外部"),
               el("span", { class: "lg" }, el("i", { class: "sw sw-opt" }), "可选"));
  }
  $("#btn-reset").addEventListener("click", () => fitAll(true));
  $("#btn-in").addEventListener("click", () => { const { w, h } = svgSize(); zoomAt(1.25, w / 2, h / 2); });
  $("#btn-out").addEventListener("click", () => { const { w, h } = svgSize(); zoomAt(0.8, w / 2, h / 2); });
  $("#btn-expand").addEventListener("click", () => { D.bands.forEach(b => state.collapsed[b.id] = false); render(); fitAll(true); });
  $("#btn-collapse").addEventListener("click", () => { D.bands.forEach(b => state.collapsed[b.id] = true); state.selected = null; render(); fitAll(true); });
  $("#opt-labels").addEventListener("change", ev => { state.allLabels = ev.target.checked; render(); });
  $("#opt-expand").addEventListener("change", ev => { state.autoExpand = ev.target.checked; });
  document.querySelectorAll(".tab").forEach(b => b.addEventListener("click", () => setTab(b.dataset.tab)));
  document.addEventListener("keydown", ev => {
    if (ev.target.matches && ev.target.matches("input, textarea")) return;
    if (state.scen < 0) return;
    const n = S[state.scen].steps.length;
    if (ev.key === "ArrowRight" && state.step < n - 1) { state.step++; goStep(); }
    else if (ev.key === "ArrowLeft" && state.step > 0) { state.step--; goStep(); }
    else if (ev.key === "Escape") exitScenario();
  });
  window.addEventListener("resize", () => { if (!currentStep()) fitAll(false); });

  // ---------------- boot ----------------
  buildDefs(); svg.append(vp);
  renderLegend(); renderScenarioPicker(); renderScenario(); renderNode(); renderObjects(); renderTrust();
  render(); fitAll(false);
  if (check.errs.length) { $("#selfcheck").hidden = false; $("#selfcheck").textContent = `数据自检发现 ${check.errs.length} 个问题，见“可信度”页`; }
  window.__mapCheck = check;
})();
