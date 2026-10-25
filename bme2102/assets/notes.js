/* BME 2102 interactive notes: shared helpers (plain script, no modules, so pages also open from a local file) */
const $ = id => document.getElementById(id);
const DEG = Math.PI/180;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)');
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let C = {};
function readColors(){ for (const k of ['paper','sheet','ink','text','muted','rule','grid','chip','crank','rod','mass','vel','acc','force','ok','bad','steel','gl']) C[k] = css('--'+k); C.mono = css('--f'); C.head = css('--f'); }
function prep(cv){
  const dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W*dpr) || cv.height !== Math.round(H*dpr)){ cv.width = Math.round(W*dpr); cv.height = Math.round(H*dpr); }
  const ctx = cv.getContext('2d'); ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,W,H);
  return { ctx, W, H };
}
function alpha(col, a){
  if (col.startsWith('#')){ let h = col.slice(1); if (h.length === 3) h = h.split('').map(c=>c+c).join('');
    const n = parseInt(h,16); return `rgba(${n>>16&255},${n>>8&255},${n&255},${a})`; }
  return col;
}
function arrow(ctx, x0, y0, x1, y1, col, w=2){
  const dx = x1-x0, dy = y1-y0, L = Math.hypot(dx,dy); if (L < 2) return;
  const ux = dx/L, uy = dy/L, hl = Math.min(10, L*0.45);
  ctx.strokeStyle = col; ctx.fillStyle = col; ctx.lineWidth = w;
  ctx.beginPath(); ctx.moveTo(x0,y0); ctx.lineTo(x1-ux*hl*0.8, y1-uy*hl*0.8); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(x1,y1); ctx.lineTo(x1-ux*hl-uy*hl*0.45, y1-uy*hl+ux*hl*0.45); ctx.lineTo(x1-ux*hl+uy*hl*0.45, y1-uy*hl-ux*hl*0.45); ctx.closePath(); ctx.fill();
}
function richParts(txt){ const out = []; const re = /_([A-Za-z0-9]+)/g; let i = 0, m;
  while ((m = re.exec(txt))){ if (m.index > i) out.push([txt.slice(i, m.index), false]); out.push([m[1], true]); i = re.lastIndex; }
  if (i < txt.length) out.push([txt.slice(i), false]); return out; }
function richWidth(ctx, txt){ const f = ctx.font, sz = parseFloat((f.match(/(\d+(?:\.\d+)?)px/)||[0,12])[1]); let w = 0;
  for (const [t, sub] of richParts(txt)){ if (sub){ ctx.font = f.replace(/\d+(?:\.\d+)?px/, (sz*0.78).toFixed(1)+'px'); w += ctx.measureText(t).width; ctx.font = f; } else w += ctx.measureText(t).width; }
  return w; }
function rich(ctx, txt, x, y){ const f = ctx.font, sz = parseFloat((f.match(/(\d+(?:\.\d+)?)px/)||[0,12])[1]), al = ctx.textAlign, w = richWidth(ctx, txt);
  let cx = al === 'right' ? x - w : al === 'center' ? x - w/2 : x; ctx.textAlign = 'left';
  for (const [t, sub] of richParts(txt)){ if (sub){ ctx.font = f.replace(/\d+(?:\.\d+)?px/, (sz*0.78).toFixed(1)+'px'); ctx.fillText(t, cx, y + sz*0.28); cx += ctx.measureText(t).width; ctx.font = f; } else { ctx.fillText(t, cx, y); cx += ctx.measureText(t).width; } }
  ctx.textAlign = al; return w; }
function label(ctx, txt, x, y, col, align='left', size=11.5, bg=true){
  ctx.font = `500 ${size}px ${C.mono}`; ctx.textAlign = align; ctx.textBaseline = 'middle';
  if (bg){ const w = richWidth(ctx, txt), x0 = align==='left'?x:align==='right'?x-w:x-w/2;
    ctx.fillStyle = alpha(C.sheet.startsWith('#')?C.sheet:'#ffffff', 0.86); ctx.fillRect(x0-2, y-8, w+4, 16); }
  ctx.fillStyle = col; rich(ctx, txt, x, y);
}
function niceStep(span, target){ const raw = span/Math.max(target,1), p = Math.pow(10, Math.floor(Math.log10(raw))), f = raw/p;
  return (f < 1.5 ? 1 : f < 3 ? 2 : f < 7 ? 5 : 10)*p; }
function fmt(v, d=2){ if (!isFinite(v)) return 'n/a'; const a = Math.abs(v);
  if (a !== 0 && (a >= 1e6 || a < 1e-3)) return v.toExponential(2);
  return v.toLocaleString('en-US', { minimumFractionDigits:d, maximumFractionDigits:d }); }
function fmtTick(v, step){ const d = Math.max(0, -Math.floor(Math.log10(step) + 1e-9)); return Math.abs(v) >= 1e5 ? v.toExponential(1) : v.toFixed(Math.min(d,4)); }

/* reusable chart: o = {x, series:[{y,color,w,dash,label}], hlines, vlines, cursor, value, ylab, xlab, markers, fill} */
function chart(cv, o){
  const { ctx, W, H } = prep(cv);
  const pl = 56, pr = 12, pt = 26, pb = 30, w = W-pl-pr, h = H-pt-pb;
  const x = o.x, xmin = o.xmin != null ? o.xmin : x[0], xmax = o.xmax != null ? o.xmax : x[x.length-1];
  let ymin = Infinity, ymax = -Infinity;
  for (const s of o.series) for (let j = 0; j < s.y.length; j++){ const v = s.y[j]; if (isFinite(v)){ if (v < ymin) ymin = v; if (v > ymax) ymax = v; } }
  for (const hl of (o.hlines||[])){ ymin = Math.min(ymin, hl.v); ymax = Math.max(ymax, hl.v); }
  if (o.zero !== false){ ymin = Math.min(ymin, 0); ymax = Math.max(ymax, 0); }
  if (!(ymax > ymin)){ ymax = ymin + 1; }
  const padv = (ymax-ymin)*0.08; ymax += padv; if (ymin < 0) ymin -= padv;
  const X = v => pl + (v-xmin)/(xmax-xmin)*w, Y = v => pt + (ymax-v)/(ymax-ymin)*h;
  cv._map = { pl, w, xmin, xmax };
  ctx.font = `11px ${C.mono}`; ctx.textBaseline = 'middle'; ctx.lineWidth = 1;
  const ys = niceStep(ymax-ymin, 5);
  for (let v = Math.ceil(ymin/ys)*ys; v <= ymax; v += ys){
    ctx.strokeStyle = C.grid; ctx.beginPath(); ctx.moveTo(pl, Y(v)); ctx.lineTo(pl+w, Y(v)); ctx.stroke();
    ctx.fillStyle = C.muted; ctx.textAlign = 'right'; ctx.fillText(fmtTick(v, ys), pl-6, Y(v)); }
  const xs = niceStep(xmax-xmin, Math.max(3, Math.floor(w/70)));
  for (let v = Math.ceil(xmin/xs)*xs; v <= xmax+1e-9; v += xs){
    ctx.strokeStyle = C.grid; ctx.beginPath(); ctx.moveTo(X(v), pt); ctx.lineTo(X(v), pt+h); ctx.stroke();
    ctx.fillStyle = C.muted; ctx.textAlign = 'center'; ctx.fillText(fmtTick(v, xs), X(v), pt+h+11); }
  ctx.textAlign = 'right'; ctx.fillText(o.xlab || '', pl+w, pt+h+24);
  ctx.save(); ctx.translate(12, pt+h/2); ctx.rotate(-Math.PI/2); ctx.textAlign = 'center'; rich(ctx, o.ylab||'', 0, 0); ctx.restore();
  for (const sh of (o.shade||[])){ ctx.fillStyle = alpha(sh.color, 0.08); ctx.fillRect(X(sh.a), pt, Math.max(0, X(sh.b)-X(sh.a)), h);
    if (sh.label){ ctx.fillStyle = C.muted; ctx.textAlign='center'; ctx.font = `10px ${C.mono}`; ctx.fillText(sh.label, (X(sh.a)+X(Math.min(sh.b,xmax)))/2, pt+h-8); ctx.font = `11px ${C.mono}`; } }
  if (ymin < 0 && ymax > 0){ ctx.strokeStyle = C.muted; ctx.beginPath(); ctx.moveTo(pl, Y(0)); ctx.lineTo(pl+w, Y(0)); ctx.stroke(); }
  if (o.fill){ const f = o.fill; ctx.fillStyle = alpha(f.color, 0.2); ctx.beginPath(); ctx.moveTo(X(x[0]), Y(0)); for (let j = 0; j < x.length; j++) ctx.lineTo(X(x[j]), Y(f.y[j])); ctx.lineTo(X(x[x.length-1]), Y(0)); ctx.closePath(); ctx.fill(); }
  for (const hl of (o.hlines||[])){ ctx.strokeStyle = hl.color; ctx.lineWidth = 1.2; ctx.setLineDash(hl.dash||[5,4]); ctx.beginPath(); ctx.moveTo(pl, Y(hl.v)); ctx.lineTo(pl+w, Y(hl.v)); ctx.stroke(); ctx.setLineDash([]);
    if (hl.label){ ctx.textAlign = hl.right ? 'right' : 'left'; ctx.fillStyle = hl.color; rich(ctx, hl.label, hl.right ? pl+w-4 : pl+4, Y(hl.v) - 8); } }
  for (const vl of (o.vlines||[])){ if (vl.v < xmin || vl.v > xmax) continue; ctx.strokeStyle = vl.color; ctx.lineWidth = 1.2; ctx.setLineDash(vl.dash||[4,3]); ctx.beginPath(); ctx.moveTo(X(vl.v), pt); ctx.lineTo(X(vl.v), pt+h); ctx.stroke(); ctx.setLineDash([]);
    if (vl.label){ ctx.fillStyle = vl.color; ctx.textAlign = X(vl.v) > pl + w*0.7 ? 'right' : 'left'; rich(ctx, vl.label, X(vl.v) + (ctx.textAlign === 'left' ? 4 : -4), pt + 10 + (vl.dy||0)); } }
  ctx.save(); ctx.beginPath(); ctx.rect(pl, pt, w, h); ctx.clip();
  for (const s of o.series){
    ctx.strokeStyle = s.color; ctx.lineWidth = s.w || 1.8; ctx.setLineDash(s.dash || []);
    ctx.beginPath(); let on = false;
    for (let j = 0; j < x.length; j++){ const v = s.y[j]; if (!isFinite(v)){ on = false; continue; } const xx = X(x[j]), yy = Y(v); if (on) ctx.lineTo(xx, yy); else { ctx.moveTo(xx, yy); on = true; } }
    ctx.stroke(); ctx.setLineDash([]); }
  for (const mk of (o.markers||[])){ ctx.fillStyle = mk.color; ctx.beginPath(); ctx.arc(X(mk.x), Y(mk.y), 4, 0, 7); ctx.fill();
    if (mk.label){ ctx.textAlign = X(mk.x) > pl + w*0.6 ? 'right' : 'left'; rich(ctx, mk.label, X(mk.x) + (ctx.textAlign==='left'?7:-7), Y(mk.y) + (mk.dy||-10)); } }
  ctx.restore();
  if (o.cursor != null){
    const cx = X(o.cursor); ctx.strokeStyle = C.ink; ctx.lineWidth = 1; ctx.setLineDash([2,3]); ctx.beginPath(); ctx.moveTo(cx, pt); ctx.lineTo(cx, pt+h); ctx.stroke(); ctx.setLineDash([]);
    const s0 = o.series[o.cursorSeries||0], j = o.cursorIndex;
    if (s0 && j != null && isFinite(s0.y[j])){ ctx.fillStyle = s0.color; ctx.beginPath(); ctx.arc(cx, Y(s0.y[j]), 4, 0, 7); ctx.fill(); } }
  ctx.font = `11px ${C.mono}`; ctx.textBaseline = 'middle'; let lx = pl;
  for (const s of o.series){ if (!s.label) continue; ctx.strokeStyle = s.color; ctx.lineWidth = 2; ctx.setLineDash(s.dash||[]); ctx.beginPath(); ctx.moveTo(lx, 12); ctx.lineTo(lx+14, 12); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = C.muted; ctx.textAlign = 'left'; lx += 26 + rich(ctx, s.label, lx+18, 12); if (lx > W - 150) break; }
  if (o.value){ ctx.font = `600 12px ${C.mono}`; ctx.fillStyle = C.ink; ctx.textAlign = 'right'; rich(ctx, o.value, W-pr, 12); }
}

/* ---------- state ---------- */

/* ---------- redraw on theme or size change ---------- */
const REDRAW = new Set();
function onRedraw(fn){ REDRAW.add(fn); }
function redrawAll(){ readColors(); REDRAW.forEach(f => { try { f(); } catch (e) { if (!/IndexSize|negative/.test(String(e))) console.error(e); } }); }
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', redrawAll);
new MutationObserver(redrawAll).observe(document.documentElement, { attributes:true, attributeFilter:['data-theme'] });
if (document.fonts && document.fonts.ready) document.fonts.ready.then(redrawAll);
const RO = new ResizeObserver(entries => { readColors(); for (const e of entries){ const f = e.target._redraw; if (f && e.target.clientWidth > 20) f(); } });
function watchSize(el, fn){ el._redraw = fn; RO.observe(el); }

/* ---------- one animation loop for every figure; figures off screen sleep ---------- */
const ANIM = [];
const IO = new IntersectionObserver(es => es.forEach(e => { e.target._vis = e.isIntersecting; }), { rootMargin:'100px' });
function animate(el, fn){ const a = { el, fn, playing:!reduceMotion.matches }; el._vis = true; IO.observe(el); ANIM.push(a); return a; }
let _last = 0;
function _loop(ts){ const dt = _last ? Math.min(0.05, (ts - _last)/1000) : 0; _last = ts;
  for (const a of ANIM) if (a.playing && a.el._vis && a.el.clientWidth > 20) { try { a.fn(dt); } catch (e) { console.error(e); a.playing = false; } }
  requestAnimationFrame(_loop); }
requestAnimationFrame(_loop);
function playButton(btn, a, after){ const set = () => { btn.textContent = a.playing ? 'Pause' : 'Play'; btn.setAttribute('aria-pressed', a.playing ? 'false' : 'true'); };
  btn.addEventListener('click', () => { a.playing = !a.playing; set(); if (after) after(); }); set(); }

/* ---------- inputs ---------- */
// spec: [key, label, min, max, step, value, unit, hint(v)]
function ctlRow(box, spec, P, onChange){ const [k, lab, mn, mx, st, val, unit, hint] = spec; P[k] = val;
  const d = document.createElement('div'); d.className = 'ctl'; const id = box.id + '_' + k;
  d.innerHTML = `<label for="${id}_n"><span>${lab}${unit ? ` [${unit}]` : ''}</span><span class="hint" id="${id}_h"></span></label>
    <input type="range" id="${id}_r" min="${mn}" max="${mx}" step="${st}" value="${val}" aria-label="${lab}">
    <input type="number" id="${id}_n" min="${mn}" max="${mx}" step="${st}" value="${val}" aria-label="${lab} value">`;
  box.appendChild(d); const rr = d.querySelector('[type=range]'), nn = d.querySelector('[type=number]'), hh = d.querySelector('.hint');
  const upd = () => { if (hint) hh.textContent = hint(P[k], P); };
  rr.addEventListener('input', () => { nn.value = rr.value; P[k] = +rr.value; upd(); onChange(k); });
  nn.addEventListener('change', () => { let v = +nn.value; if (!isFinite(v)) v = P[k]; v = clamp(v, +nn.min, +nn.max); nn.value = v; rr.value = v; P[k] = v; upd(); onChange(k); });
  upd(); return { set(v){ P[k] = v; rr.value = v; nn.value = v; upd(); }, upd };
}
function controls(box, specs, P, onChange){ const H = {}; for (const s of specs) H[s[0]] = ctlRow(box, s, P, k => { for (const h of Object.values(H)) h.upd(); onChange(k); }); return H; }

/* ---------- worked examples: inputs prefilled with the numbers in the notes; working recomputes ---------- */
function example(el, o){
  const V = {}; o.inputs.forEach(([k, , v]) => V[k] = v);
  el.className = 'ex'; el.innerHTML = `<summary>${o.title} <span class="src">${o.src || ''}</span></summary><div class="exbody steps-hidden">
    <p class="exq">${o.q}</p><div class="exin">${o.inputs.map(([k, lab, v, unit]) => `<label>${lab}${unit ? ` [${unit}]` : ''}<input type="number" step="any" data-k="${k}" value="${v}"></label>`).join('')}</div>
    <ol class="steps"></ol><div class="btnrow" style="padding:10px 0 0;border:0"><button type="button" class="show">Show the working</button><button type="button" class="reset">Back to the numbers in the notes</button></div>${o.after || ''}</div>`;
  const body = el.querySelector('.exbody'), ol = el.querySelector('ol.steps');
  const run = () => { let s; try { s = o.solve(V); } catch (e) { s = [['These numbers do not give a real answer (check the inputs).', '', true]]; }
    ol.innerHTML = s.map(([t, r, ans]) => `<li class="${ans ? 'ans' : ''}">${t}${r !== '' && r != null ? ` <span class="r">${r}</span>` : ''}</li>`).join(''); if (o.onSolve) o.onSolve(V); };
  el.querySelectorAll('input[data-k]').forEach(i => i.addEventListener('input', () => { V[i.dataset.k] = +i.value; run(); }));
  el.querySelector('.show').addEventListener('click', e => { const h = body.classList.toggle('steps-hidden'); e.target.textContent = h ? 'Show the working' : 'Hide the working'; });
  el.querySelector('.reset').addEventListener('click', () => { o.inputs.forEach(([k, , v]) => { V[k] = v; el.querySelector(`[data-k="${k}"]`).value = v; }); run(); });
  run(); return V; }
const n2 = (v, d = 2) => fmt(v, d);

/* ---------- self-check questions ---------- */
function quiz(el, qs){
  el.className = 'quiz'; let right = 0, done = 0;
  el.innerHTML = qs.map((q, i) => `<div class="q" data-i="${i}"><p>${i + 1}. ${q.q}</p><div class="opts">${q.o.map((t, j) => `<button type="button" data-j="${j}">${t}</button>`).join('')}</div><div class="why" hidden></div></div>`).join('') + '<p class="score" aria-live="polite"></p>';
  el.querySelectorAll('.q').forEach(d => { const q = qs[+d.dataset.i];
    d.querySelectorAll('.opts button').forEach(b => b.addEventListener('click', () => { if (d.dataset.done) return; d.dataset.done = 1; done++;
      const j = +b.dataset.j; if (j === q.a) right++; b.classList.add(j === q.a ? 'right' : 'wrong'); d.querySelector(`[data-j="${q.a}"]`).classList.add('right');
      const w = d.querySelector('.why'); w.hidden = false; w.innerHTML = (j === q.a ? 'Correct. ' : 'Not quite. ') + q.why;
      el.querySelector('.score').textContent = `${right} of ${done} correct so far (${qs.length} questions).`; })); }); }

/* ---------- 3D viewer (three.js r147, global THREE) ---------- */
function hasGL(){ try { const c = document.createElement('canvas'); return !!(window.WebGLRenderingContext && (c.getContext('webgl') || c.getContext('experimental-webgl'))); } catch (e) { return false; } }
function view3d(el, o){
  if (!window.THREE || !hasGL()){ el.outerHTML = '<div class="nogl">The 3D model needs WebGL, which this browser has switched off. The drawings and graphs on this page still work.</div>'; return null; }
  const R = new THREE.WebGLRenderer({ canvas:el, antialias:true, alpha:false }); R.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  R.outputEncoding = THREE.sRGBEncoding;
  const scene = new THREE.Scene(), cam = new THREE.PerspectiveCamera(o.fov || 35, 1, 0.01, 200);
  cam.position.set(...(o.cam || [4, 3, 6])); const ctl = new THREE.OrbitControls(cam, el); ctl.target.set(...(o.target || [0, 0, 0])); ctl.enableDamping = true; ctl.dampingFactor = 0.12; ctl.update();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 0.75)); const d1 = new THREE.DirectionalLight(0xffffff, 0.9); d1.position.set(5, 8, 6); scene.add(d1);
  const d2 = new THREE.DirectionalLight(0xffffff, 0.35); d2.position.set(-6, 2, -4); scene.add(d2);
  const v = { R, scene, cam, ctl, root:new THREE.Group(), el }; scene.add(v.root);
  const bg = () => { readColors(); scene.background = new THREE.Color(C.gl || '#fbfcfd'); if (o.theme) o.theme(v); };
  const size = () => { const w = el.clientWidth, h = el.clientHeight; if (!w || !h) return; R.setSize(w, h, false); cam.aspect = w/h; cam.updateProjectionMatrix(); R.render(scene, cam); };
  bg(); if (o.build) o.build(v); watchSize(el, size); onRedraw(() => { bg(); R.render(scene, cam); }); size();
  v.anim = animate(el, dt => { if (o.update) o.update(dt, v); ctl.update(); R.render(scene, cam); });
  ctl.addEventListener('change', () => { if (!v.anim.playing) R.render(scene, cam); });
  v.render = () => R.render(scene, cam); return v; }
function mat(color, o = {}){ return new THREE.MeshStandardMaterial(Object.assign({ color, metalness:0.35, roughness:0.45 }, o)); }

/* ---------- drawing helpers shared by chapters 1 to 4 ---------- */
function fitView(cv, b, pad = 14){ const { ctx, W, H } = prep(cv); const sc = Math.min((W - 2*pad)/(b.xr - b.xl), (H - 2*pad)/(b.yt - b.yb));
  const ox = pad + (W - 2*pad - (b.xr - b.xl)*sc)/2 - b.xl*sc, oy = pad + (H - 2*pad - (b.yt - b.yb)*sc)/2 + b.yt*sc;
  return { ctx, W, H, sc, PX:x => ox + x*sc, PY:y => oy - y*sc, IX:px => (px - ox)/sc, IY:py => (oy - py)/sc, P:(p) => [ox + p[0]*sc, oy - p[1]*sc] }; }
function statsHTML(rows){ return rows.map(([k, v, cls]) => `<div class="stat${cls ? ' ' + cls : ''}"><div class="k">${k}</div><div class="v">${v}</div></div>`).join(''); }
function segGroup(el, cb){ const bs = [...el.querySelectorAll('button')];
  bs.forEach(b => b.addEventListener('click', () => { bs.forEach(x => x.setAttribute('aria-pressed', x === b ? 'true' : 'false')); cb(b.dataset.v, b); }));
  const on = bs.find(b => b.getAttribute('aria-pressed') === 'true') || bs[0]; on.setAttribute('aria-pressed', 'true'); return on.dataset.v; }
function dotAt(ctx, x, y, col, r = 4){ ctx.fillStyle = col; ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); }
function pinAt(ctx, x, y, r = 4.5){ ctx.fillStyle = C.sheet; ctx.strokeStyle = C.ink; ctx.lineWidth = 1.6; ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); ctx.stroke(); }
function barAt(ctx, x1, y1, x2, y2, col, w = 6){ ctx.strokeStyle = col; ctx.lineWidth = w; ctx.lineCap = 'round'; ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke(); ctx.lineCap = 'butt'; }
function groundAt(ctx, x, y, w = 34, ang = 0){ ctx.save(); ctx.translate(x, y); ctx.rotate(ang); ctx.strokeStyle = C.muted; ctx.lineWidth = 1.4;
  ctx.beginPath(); ctx.moveTo(-w/2, 0); ctx.lineTo(w/2, 0); for (let i = -w/2; i < w/2; i += 6){ ctx.moveTo(i + 6, 0); ctx.lineTo(i, 7); } ctx.stroke(); ctx.restore(); }
function pivotAt(ctx, x, y){ ctx.strokeStyle = C.muted; ctx.fillStyle = alpha(C.muted, 0.18); ctx.lineWidth = 1.4; ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x - 9, y + 13); ctx.lineTo(x + 9, y + 13); ctx.closePath(); ctx.fill(); ctx.stroke(); groundAt(ctx, x, y + 13, 30); pinAt(ctx, x, y); }
function dashLine(ctx, x1, y1, x2, y2, col, d = [5, 4], w = 1.2){ ctx.strokeStyle = col; ctx.lineWidth = w; ctx.setLineDash(d); ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke(); ctx.setLineDash([]); }
/* pointer dragging on a canvas: pick(x, y) returns a handle or null; move(handle, x, y) */
function dragOn(cv, pick, move, end){ let h = null;
  cv.addEventListener('pointerdown', e => { const r = cv.getBoundingClientRect(); h = pick(e.clientX - r.left, e.clientY - r.top); if (h != null){ cv.setPointerCapture(e.pointerId); e.preventDefault(); e.stopPropagation(); } });
  cv.addEventListener('pointermove', e => { const r = cv.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
    if (h != null){ move(h, x, y); return; } cv.style.cursor = pick(x, y) != null ? 'grab' : ''; });
  const up = () => { if (h != null && end) end(h); h = null; }; cv.addEventListener('pointerup', up); cv.addEventListener('pointercancel', up);
  cv.addEventListener('click', e => e.stopPropagation()); }
/* four-bar closure: ground pivots O2 (x2,y2) and O4 (x4,y4), crank a at angle t, coupler b, rocker c. Returns B or null. branch = +1/-1 */
function fourbar(O2, O4, a, b, c, t, branch = 1){ const A = [O2[0] + a*Math.cos(t), O2[1] + a*Math.sin(t)], dx = O4[0] - A[0], dy = O4[1] - A[1], d = Math.hypot(dx, dy);
  if (d > b + c || d < Math.abs(b - c) || d === 0) return null; const k = (b*b - c*c + d*d)/(2*d), h = Math.sqrt(Math.max(0, b*b - k*k));
  const B = [A[0] + k*dx/d - branch*h*dy/d, A[1] + k*dy/d + branch*h*dx/d]; return { A, B }; }
const vsub = (p, q) => [p[0] - q[0], p[1] - q[1]], vadd = (p, q) => [p[0] + q[0], p[1] + q[1]], vmul = (p, k) => [p[0]*k, p[1]*k], vlen = p => Math.hypot(p[0], p[1]), vperp = p => [-p[1], p[0]];

/* ---------- table of contents highlight ---------- */
document.addEventListener('DOMContentLoaded', () => { readColors();
  const links = [...document.querySelectorAll('nav.toc a[href^="#"]')], map = new Map(links.map(a => [a.getAttribute('href').slice(1), a]));
  const io = new IntersectionObserver(es => { es.forEach(e => { if (e.isIntersecting){ links.forEach(a => a.classList.remove('on')); const a = map.get(e.target.id); if (a) a.classList.add('on'); } }); }, { rootMargin:'-20% 0px -70% 0px' });
  map.forEach((a, id) => { const s = document.getElementById(id); if (s) io.observe(s); }); });
readColors();
