/* BME 2102 interactive notes, Chapter 5: toothed gearing */
(() => {
'use strict';
const inv = a => Math.tan(a) - a;
const TAU = Math.PI*2;

/* ---------- involute gear geometry (polar outline, same method as the Spur Gear Mesh Lab) ---------- */
function flankRadii(rb, ra, rf, psi, n = 14){
  const rs = Math.max(rb, rf); let rt = ra;
  if (psi(ra) <= 0){ let lo = rs, hi = ra; for (let i = 0; i < 50; i++){ const md = (lo + hi)/2; if (psi(md) > 0) lo = md; else hi = md; } rt = lo; }
  const list = []; for (let i = 0; i <= n; i++) list.push(rs + (rt - rs)*i/n); return { rs, rt, list }; }
function gearGeo(z, m, phiDeg, ha = 1, hf = 1.25, bl = 0, n = 14){
  const phi = phiDeg*DEG, r = m*z/2, rb = r*Math.cos(phi), ra = r + ha*m, rf = r - hf*m;
  const psi = rho => Math.PI/(2*z)*(1 - bl) + inv(phi) - inv(Math.acos(Math.min(1, rb/Math.max(rho, rb))));
  return { z, m, phi, r, rb, ra, rf, psi, fr:flankRadii(rb, ra, rf, psi, n), ha, hf }; }
function gearPath(g, c0, rootN = 6){
  const pts = [], step = TAU/g.z, { psi, fr, rb, rf } = g;
  for (let k = 0; k < g.z; k++){ const c = c0 + k*step;
    if (rf < rb) pts.push([rf, c - psi(rb)]);
    for (const r of fr.list) pts.push([r, c - psi(r)]);
    const tp = psi(fr.rt); for (let i = 1; i < 4; i++) pts.push([fr.rt, c - tp + 2*tp*i/4]);
    for (let i = fr.list.length - 1; i >= 0; i--){ const r = fr.list[i]; pts.push([r, c + psi(r)]); }
    if (rf < rb) pts.push([rf, c + psi(rb)]);
    const a0 = c + psi(fr.rs), a1 = c + step - psi(fr.rs);
    for (let i = 1; i < rootN; i++) pts.push([rf, a0 + (a1 - a0)*i/rootN]); }
  return pts; }
function toothPath(g, c){ const pts = [], { psi, fr, rb, rf } = g;
  if (rf < rb) pts.push([rf, c - psi(rb)]); for (const r of fr.list) pts.push([r, c - psi(r)]);
  for (let i = fr.list.length - 1; i >= 0; i--){ const r = fr.list[i]; pts.push([r, c + psi(r)]); } if (rf < rb) pts.push([rf, c + psi(rb)]); return pts; }
/* mesh of two external gears; pinion centre at (−r1w, 0), wheel at (+r2w, 0), pitch point at the origin */
function mesh(z1, z2, m, phiDeg, ha = 1, hf = 1.25, da = 0){
  const g1 = gearGeo(z1, m, phiDeg, ha, hf), g2 = gearGeo(z2, m, phiDeg, ha, hf), phi = phiDeg*DEG;
  const a = g1.r + g2.r, aw = a + da, alpha = Math.acos(Math.min(1, a*Math.cos(phi)/aw));
  const r1w = g1.rb/Math.cos(alpha), r2w = g2.rb/Math.cos(alpha), pb = Math.PI*m*Math.cos(phi);
  const MP = r1w*Math.sin(alpha), PN = r2w*Math.sin(alpha);
  const recess = Math.sqrt(g1.ra**2 - g1.rb**2) - MP, approach = Math.sqrt(g2.ra**2 - g2.rb**2) - PN;
  const j = TAU*r1w/z1 - 2*r1w*g1.psi(r1w) - 2*r2w*g2.psi(r2w);
  const contacts = u0 => { const k0 = Math.ceil((-approach - u0)/pb - 1e-12), L = []; for (let k = k0; u0 + k*pb <= recess + 1e-12; k++) L.push(u0 + k*pb); return L; };
  return { g1, g2, z1, z2, m, phi, a, aw, alpha, r1w, r2w, pb, MP, PN, recess, approach, path:approach + recess, eps:(approach + recess)/pb, j, ratio:z1/z2, contacts,
    c1:th => th - g1.psi(r1w), c2:th => Math.PI - g2.psi(r2w) - th*z1/z2 }; }

/* ---------- canvas view helper ---------- */
function view(cv, b, pad = 16){ const { ctx, W, H } = prep(cv); const sc = Math.min((W - 2*pad)/(b.xr - b.xl), (H - 2*pad)/(b.yt - b.yb));
  const ox = pad + (W - 2*pad - (b.xr - b.xl)*sc)/2 - b.xl*sc, oy = pad + (H - 2*pad - (b.yt - b.yb)*sc)/2 + b.yt*sc;
  return { ctx, W, H, sc, PX:x => ox + x*sc, PY:y => oy - y*sc }; }
function polyPolar(ctx, V, pts, cx, cy, fill, stroke, lw = 1.2){ ctx.beginPath(); pts.forEach(([r, t], i) => { const x = V.PX(cx + r*Math.cos(t)), y = V.PY(cy + r*Math.sin(t)); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.closePath();
  if (fill){ ctx.fillStyle = fill; ctx.fill(); } if (stroke){ ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); } }
function circ(ctx, V, cx, cy, r, col, dash = [], lw = 1){ ctx.strokeStyle = col; ctx.lineWidth = lw; ctx.setLineDash(dash); ctx.beginPath(); ctx.arc(V.PX(cx), V.PY(cy), r*V.sc, 0, 7); ctx.stroke(); ctx.setLineDash([]); }
function dot(ctx, x, y, col, r = 4){ ctx.fillStyle = col; ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); }
function lbl(ctx, t, x, y, col, al = 'left', size = 12){ label(ctx, t, x, y, col, al, size, true); }
function drawPair(ctx, V, M, th, opt = {}){
  const O1 = -M.r1w, O2 = M.r2w;
  polyPolar(ctx, V, gearPath(M.g1, M.c1(th)), O1, 0, alpha(C.crank, 0.2), C.crank);
  polyPolar(ctx, V, gearPath(M.g2, M.c2(th)), O2, 0, alpha(C.rod, 0.18), C.rod);
  if (opt.mark){ polyPolar(ctx, V, toothPath(M.g1, M.c1(th)), O1, 0, alpha(C.crank, 0.7)); polyPolar(ctx, V, toothPath(M.g2, M.c2(th)), O2, 0, alpha(C.rod, 0.65)); }
  for (const x of [O1, O2]){ ctx.fillStyle = C.sheet; ctx.strokeStyle = C.ink; ctx.lineWidth = 1.4; ctx.beginPath(); ctx.arc(V.PX(x), V.PY(0), 4, 0, 7); ctx.fill(); ctx.stroke(); }
  return { O1, O2 }; }

/* ================= 5.1 friction wheels against gears ================= */
{ const P = {}, cv = $('fw'); let a1 = 0, a2 = 0, b2 = 0;
  controls($('fw_c'), [['load', 'Load torque on the driven shaft', 0, 200, 1, 60, '% of friction limit', v => v > 100 ? 'friction wheels slip' : 'no slip']], P, () => draw());
  const G = mesh(14, 21, 1, 20);
  function draw(){ const { ctx, W, H } = prep(cv), half = W/2, R1 = Math.min(H*0.26, half*0.16), R2 = R1*1.5, cy = H*0.48;
    const slip = P.load > 100, f = slip ? 100/P.load : 1;
    // friction wheels
    const cx1 = half*0.5 - (R1 + R2)/2 + R1, cx2 = cx1 + R1 + R2;
    const wheel = (cx, R, ang, col) => { ctx.fillStyle = alpha(col, 0.18); ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(cx, cy, R, 0, 7); ctx.fill(); ctx.stroke();
      for (let k = 0; k < 6; k++){ const t = ang + k*TAU/6; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx + R*0.92*Math.cos(t), cy - R*0.92*Math.sin(t)); ctx.stroke(); }
      dot(ctx, cx + R*0.75*Math.cos(ang), cy - R*0.75*Math.sin(ang), C.acc, 5); };
    wheel(cx1, R1, a1, C.crank); wheel(cx2, R2, a2, C.rod);
    if (slip){ ctx.strokeStyle = C.bad; ctx.lineWidth = 2; for (let k = -2; k <= 2; k++){ ctx.beginPath(); ctx.moveTo(cx1 + R1 - 4, cy + k*6); ctx.lineTo(cx1 + R1 + 4, cy + k*6 + 3); ctx.stroke(); } }
    lbl(ctx, 'Friction wheels', half*0.5, 16, C.ink, 'center', 13); lbl(ctx, slip ? `slipping: wheel turns ${Math.round(100*f)}% of the right speed` : 'turning at the right speed', half*0.5, H - 14, slip ? C.bad : C.ok, 'center', 12);
    // gears
    const sc = R1/G.g1.r, V = { sc, PX:x => half + half*0.5 + (x - (G.r2w - G.r1w)/2)*sc, PY:y => cy - y*sc };
    drawPair(ctx, V, G, b2);
    dot(ctx, V.PX(-G.r1w + G.g1.r*0.6*Math.cos(b2)), V.PY(G.g1.r*0.6*Math.sin(b2)), C.acc, 5);
    lbl(ctx, 'Gears', half*1.5, 16, C.ink, 'center', 13); lbl(ctx, 'teeth cannot slip: always the right speed', half*1.5, H - 14, C.ok, 'center', 12);
    ctx.strokeStyle = C.grid; ctx.beginPath(); ctx.moveTo(half, 26); ctx.lineTo(half, H - 30); ctx.stroke();
    $('fw_cap').innerHTML = `Speed ratio wanted: driven = driver × r<sub>1</sub>/r<sub>2</sub>. ${slip ? `With the load above the friction limit μNr, the friction pair loses <b>${Math.round(100*(1 - f))}%</b> of its speed and the contact wears.` : 'Below the friction limit both pairs keep the ratio; push the load past 100% to see the difference.'}`; }
  const an = animate(cv, dt => { const w = 1.2; a1 += w*dt; const f = P.load > 100 ? 100/P.load : 1; a2 -= w*dt*f/1.5; b2 += w*dt; draw(); });
  watchSize(cv, draw); onRedraw(draw); draw(); }

/* ================= 5.2 gear types in 3D ================= */
const G3 = { type:'spur', speed:0.5 };
const TYPES = {
  spur:['Spur gears', 'parallel shafts', 'Teeth straight and parallel to the axis. The simplest and most common gear; noisy at high speed because each tooth takes its load all at once.'],
  helical:['Helical gears', 'parallel shafts', 'Teeth inclined to the axis at the helix angle (here 20°). Contact starts at one end and spreads across the face, so they run quietly, but the inclined teeth push the shafts sideways (end thrust). The two gears have opposite hands.'],
  herring:['Herringbone (double helical) gears', 'parallel shafts', 'Two opposite helices side by side. The end thrusts of the two halves cancel, so heavy, smooth drives such as turbine gearboxes use them.'],
  internal:['Internal gear and pinion', 'parallel shafts, same direction', 'Teeth cut on the inside of a ring. The pinion and ring turn the same way and the centres are close together; used in epicyclic (planetary) gearboxes.'],
  rack:['Rack and pinion', 'rotation to straight-line motion', 'A rack is a gear of infinite radius. One turn of the pinion moves the rack by π × pitch diameter. Used in car steering and lathe carriages. Rack teeth have straight sides at the pressure angle.'],
  bevel:['Bevel (mitre) gears', 'intersecting shafts at 90°', 'Teeth cut on cones whose apexes meet where the shafts cross. Equal gears on shafts at right angles are mitre gears. Turn on the pitch surfaces: two cones rolling on each other.'],
  worm:['Worm and worm wheel', 'shafts that neither meet nor are parallel', 'A screw (the worm) drives a wheel. With a single-start worm the wheel moves one tooth per turn of the worm, so a 30-tooth wheel gives 30:1 in one step. Spiral (skew) gears are the other kind of gear for such shafts.']
};
const v3 = view3d($('g3'), { cam:[0.0, 0.11, 0.26], target:[0, 0, 0], fov:30,
  build(v){ buildType(v); },
  update(dt, v){ if (!v.parts) return; const sp = $('g3_spin').checked ? 0.35 : 1.2; v.t = (v.t || 0) + dt*sp; v.parts.step(v.t); },
  theme(v){ if (v.mats){ v.mats.a.color.set(C.crank); v.mats.b.color.set(C.rod); v.mats.p.color.set(C.vel); } } });
function shapeFromPolar(pts){ const s = new THREE.Shape(); pts.forEach(([r, t], i) => { const x = r*Math.cos(t), y = r*Math.sin(t); i ? s.lineTo(x, y) : s.moveTo(x, y); }); s.closePath(); return s; }
function circlePath(r, n = 48, hole = true){ const p = new THREE.Path(); for (let i = 0; i <= n; i++){ const t = (hole ? -1 : 1)*i*TAU/n; i ? p.lineTo(r*Math.cos(t), r*Math.sin(t)) : p.moveTo(r, 0); } return p; }
function gearMesh(g, width, material, o = {}){
  const s = shapeFromPolar(gearPath(g, 0, 4)); if (o.bore !== false) s.holes.push(circlePath(o.bore || g.rf*0.35));
  const steps = o.twist || o.herring || o.cone ? 12 : 1;
  const geo = new THREE.ExtrudeGeometry(s, { depth:width, bevelEnabled:false, steps, curveSegments:8 }); geo.translate(0, 0, -width/2);
  if (o.twist || o.herring || o.cone){ const p = geo.attributes.position;
    for (let i = 0; i < p.count; i++){ let x = p.getX(i), y = p.getY(i); const z = p.getZ(i);
      if (o.cone){ const f = 1 - (z + width/2)/o.cone; x *= f; y *= f; }
      const a = o.twist ? z*o.twist : o.herring ? Math.abs(z)*o.herring : 0;
      if (a){ const c = Math.cos(a), s2 = Math.sin(a); const nx = x*c - y*s2, ny = x*s2 + y*c; x = nx; y = ny; }
      p.setXYZ(i, x, y, z); } geo.computeVertexNormals(); }
  const mesh3 = new THREE.Mesh(geo, material); return mesh3; }
function pitchSurf(r, w, mat2, cone){ const geo = cone ? new THREE.CylinderGeometry(r*(1 - w/cone), r, w, 48, 1, true) : new THREE.CylinderGeometry(r, r, w*1.02, 48, 1, true); geo.rotateX(Math.PI/2); if (cone) geo.rotateX(Math.PI); return new THREE.Mesh(geo, mat2); }
function buildType(v){
  const T = THREE, root = v.root; while (root.children.length) root.remove(root.children[0]); readColors();
  const mats = v.mats = { a:mat(C.crank, { metalness:0.45, roughness:0.38 }), b:mat(C.rod, { metalness:0.45, roughness:0.4 }), p:new T.MeshBasicMaterial({ color:C.vel, transparent:true, opacity:0.28, side:T.DoubleSide, depthWrite:false }), s:mat('#9aa6b2', { metalness:0.6, roughness:0.35 }) };
  const m = 0.004, w = 0.03, type = G3.type, pitch = $('g3_pitch').checked, shaft = (len, r, axis) => { const g = new T.CylinderGeometry(r, r, len, 24); if (axis === 'z') g.rotateX(Math.PI/2); if (axis === 'x') g.rotateZ(Math.PI/2); return new T.Mesh(g, mats.s); };
  let step = () => {};
  const grp = (obj, pos) => { const g = new T.Group(); g.add(obj); if (pos) g.position.set(...pos); root.add(g); return g; };
  if (['spur', 'helical', 'herring'].includes(type)){
    const z1 = 16, z2 = 26, g1 = gearGeo(z1, m, 20, 1, 1.25, 0.04), g2 = gearGeo(z2, m, 20, 1, 1.25, 0.04), beta = 20*DEG;
    const o1 = type === 'helical' ? { twist:Math.tan(beta)/g1.r } : type === 'herring' ? { herring:Math.tan(beta)/g1.r } : {};
    const o2 = type === 'helical' ? { twist:-Math.tan(beta)/g2.r } : type === 'herring' ? { herring:-Math.tan(beta)/g2.r } : {};
    const A = grp(gearMesh(g1, w, mats.a, o1), [-g1.r, 0, 0]), B = grp(gearMesh(g2, w, mats.b, o2), [g2.r, 0, 0]);
    A.add(shaft(0.09, g1.rf*0.34, 'z')); B.add(shaft(0.09, g2.rf*0.3, 'z'));
    if (pitch){ A.add(pitchSurf(g1.r, w, mats.p)); B.add(pitchSurf(g2.r, w, mats.p)); }
    step = t => { A.rotation.z = t; B.rotation.z = -t*z1/z2 + Math.PI + Math.PI/z2; };
    root.position.set(-(g2.r - g1.r)/2, 0, 0); v.cam.position.set(0.02, 0.09, 0.24);
  } else if (type === 'internal'){
    const z1 = 14, z2 = 40, g1 = gearGeo(z1, m, 20, 1, 1.25, 0.05), R = m*z2/2;
    const hole = gearGeo(z2, m, 20, 1.25, 1.0, -0.05);   // the ring's tooth spaces have the shape of external teeth
    const s = new T.Shape(); s.absarc(0, 0, R + 2.6*m, 0, TAU, false);
    const hp = new T.Path(); gearPath(hole, 0, 4).slice().reverse().forEach(([r, t], i) => { const x = r*Math.cos(t), y = r*Math.sin(t); i ? hp.lineTo(x, y) : hp.moveTo(x, y); }); hp.closePath(); s.holes.push(hp);
    const rg = new T.ExtrudeGeometry(s, { depth:w, bevelEnabled:false, curveSegments:48 }); rg.translate(0, 0, -w/2);
    const B = grp(new T.Mesh(rg, mats.b), [0, 0, 0]), A = grp(gearMesh(g1, w*1.15, mats.a), [R - g1.r, 0, 0]); A.add(shaft(0.08, g1.rf*0.34, 'z'));
    if (pitch){ A.add(pitchSurf(g1.r, w, mats.p)); B.add(pitchSurf(R, w, mats.p)); }
    step = t => { A.rotation.z = t; B.rotation.z = t*z1/z2; };
    root.position.set(0, 0, 0); v.cam.position.set(0.0, 0.12, 0.3);
  } else if (type === 'rack'){
    const z1 = 18, g1 = gearGeo(z1, m, 20, 1, 1.25, 0.04), p = Math.PI*m, tn = Math.tan(20*DEG), nT = 22, L = nT*p;
    const s = new T.Shape();
    const xt = m, xr = -1.25*m, back = -3.2*m; s.moveTo(back, -L/2);
    for (let k = 0; k < nT; k++){ const yc = -L/2 + (k + 0.5)*p;
      const hr = p/4 + 1.25*m*tn*0.98, ht = p/4 - m*tn*1.02;
      s.lineTo(xr, yc - hr); s.lineTo(xt, yc - ht); s.lineTo(xt, yc + ht); s.lineTo(xr, yc + hr); }
    s.lineTo(back, L/2); s.closePath();
    const rgeo = new T.ExtrudeGeometry(s, { depth:w, bevelEnabled:false }); rgeo.translate(0, 0, -w/2); rgeo.scale(-1, 1, 1);
    const B = grp(new T.Mesh(rgeo, mats.b), [g1.r, 0, 0]), A = grp(gearMesh(g1, w, mats.a), [0, 0, 0]); A.add(shaft(0.09, g1.rf*0.34, 'z'));
    if (pitch){ A.add(pitchSurf(g1.r, w, mats.p)); const pl = new T.Mesh(new T.PlaneGeometry(w, L), mats.p); pl.rotation.y = Math.PI/2; B.add(pl); }
    step = t => { const tt = ((t*0.6) % 3) - 1.5, ang = tt; A.rotation.z = ang; B.position.y = g1.r*ang + 0; };
    root.position.set(-g1.r/2, 0, 0); v.cam.position.set(0.1, 0.06, 0.52);
  } else if (type === 'bevel'){
    const z = 20, g = gearGeo(z, m, 20, 1, 1.25, 0.06), Hc = g.r, wb = Hc*0.32;   // 45° pitch cones: apex height = r
    const A = grp(gearMesh(g, wb, mats.a, { cone:Hc, bore:g.rf*0.3 })), B = grp(gearMesh(g, wb, mats.b, { cone:Hc, bore:g.rf*0.3 }));
    // gear A: axis along world +y, big end at y = −Hc... place so both apexes meet at the origin
    const holdA = new T.Group(), holdB = new T.Group(); root.remove(A); root.remove(B); holdA.add(A); holdB.add(B); root.add(holdA); root.add(holdB);
    A.position.z = wb/2 - Hc + wb*0; A.children[0].position.z = 0;
    holdA.rotation.x = -Math.PI/2; holdA.position.set(0, 0, 0); A.position.set(0, 0, -Hc + wb/2);
    holdB.rotation.y = Math.PI/2; B.position.set(0, 0, -Hc + wb/2);
    const sA = shaft(0.06, g.rf*0.28, 'z'); sA.position.z = -Hc - 0.02; holdA.add(sA); const sB = shaft(0.06, g.rf*0.28, 'z'); sB.position.z = -Hc - 0.02; holdB.add(sB);
    if (pitch){ const cA = new T.Mesh(new T.ConeGeometry(g.r, Hc, 48, 1, true), mats.p); cA.rotation.x = Math.PI/2; cA.position.z = -Hc/2; holdA.add(cA);
      const cB = new T.Mesh(new T.ConeGeometry(g.r, Hc, 48, 1, true), mats.p); cB.rotation.x = Math.PI/2; cB.position.z = -Hc/2; holdB.add(cB); }
    step = t => { A.rotation.z = t; B.rotation.z = -t + Math.PI/z + 0.5*Math.PI; };
    root.position.set(0, 0, 0); v.ctl.target.set(-g.r*0.5, -g.r*0.4, -g.r*0.5); v.cam.position.set(0.12, 0.1, 0.16);
  } else if (type === 'worm'){
    const z2 = 30, g2 = gearGeo(z2, m, 20, 1, 1.25, 0.12), rw = 2.2*m*2, p = Math.PI*m, lead = Math.atan(p/(TAU*rw)), C2 = g2.r + rw;
    const B = grp(gearMesh(g2, w*0.8, mats.b, { twist:-Math.tan(lead)/g2.r }), [0, 0, 0]); B.add(shaft(0.06, g2.rf*0.25, 'z'));
    const W = new T.Group(); W.position.set(0, C2, 0); root.add(W);
    const core = new T.Mesh(new T.CylinderGeometry(rw - 1.1*m, rw - 1.1*m, 12*p, 32), mats.a); core.rotation.z = Math.PI/2; W.add(core);
    class Helix extends T.Curve { getPoint(u){ const turns = 10, t = (u - 0.5)*turns*TAU; return new T.Vector3(p*t/TAU, (rw - 0.45*m)*Math.cos(t), (rw - 0.45*m)*Math.sin(t)); } }
    const th = new T.Mesh(new T.TubeGeometry(new Helix(), 600, 0.75*m, 10, false), mats.a); W.add(th);
    const ws = shaft(16*p + 0.04, (rw - 1.1*m)*0.45, 'x'); W.add(ws);
    if (pitch){ B.add(pitchSurf(g2.r, w*0.8, mats.p)); const pc = new T.Mesh(new T.CylinderGeometry(rw, rw, 11*p, 32, 1, true), mats.p); pc.rotation.z = Math.PI/2; W.add(pc); }
    step = t => { const a = t*2.2; W.rotation.x = a; B.rotation.z = a/z2 + Math.PI/2 + Math.PI/z2*0.5; };
    root.position.set(0, -C2*0.45, 0); v.cam.position.set(0.16, 0.12, 0.26);
  }
  v.parts = { step }; step(v.t || 0); v.ctl.update();
  const [ti, sub, cap] = TYPES[type]; $('g3_title').textContent = ti; $('g3_sub').textContent = sub; $('g3_cap').textContent = cap; }
if (v3){
  $('g3_chips').innerHTML = Object.entries(TYPES).map(([k, t]) => `<button type="button" data-t="${k}" aria-pressed="${k === 'spur'}">${t[0].replace(' gears', '').replace(' (double helical)', '')}</button>`).join('');
  $('g3_chips').querySelectorAll('button').forEach(b => b.addEventListener('click', () => { G3.type = b.dataset.t; $('g3_chips').querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', x === b)); v3.ctl.target.set(0, 0, 0); buildType(v3); v3.render(); }));
  $('g3_pitch').addEventListener('change', () => { buildType(v3); v3.render(); });
  playButton($('g3_play'), v3.anim);
} else { $('g3_chips').innerHTML = ''; ['g3_play'].forEach(id => $(id) && ($(id).disabled = true)); }

/* peripheral velocity class */
{ const P = {}; const up = () => { const v = Math.PI*P.d/1000*P.N/60, cls = v < 3 ? 'low velocity' : v <= 15 ? 'medium velocity' : 'high velocity';
    $('pv_s').innerHTML = `<div class="stat"><div class="k">Pitch line speed</div><div class="v">${n2(v)} <small>m/s</small></div></div><div class="stat"><div class="k">Class</div><div class="v">${cls}</div></div>`; };
  controls($('pv_c'), [['d', 'Pitch circle diameter', 20, 1000, 1, 250, 'mm'], ['N', 'Speed', 10, 6000, 10, 650, 'rpm']], P, up); up(); }

/* ================= 5.3 terminology ================= */
{ const P = {}, cv = $('tm');
  const SYS = { fd20:[20, 1, 1.25], fd145:[14.5, 1, 1.25], stub20:[20, 0.8, 1] };
  const TERMS = [['pitch', 'Pitch circle', 'The imaginary circle that, rolling without slip on the other gear’s pitch circle, gives the same motion as the gear. Its diameter D = mT is the size of the gear.'],
    ['P', 'Pitch point', 'Where the two pitch circles touch, on the line of centres.'],
    ['phi', 'Pressure angle', 'The angle between the common normal at the point of contact (the line of action) and the common tangent at the pitch point. Standard values 14.5° and 20°.'],
    ['add', 'Addendum', 'Radial distance from the pitch circle to the top of the tooth.'],
    ['ded', 'Dedendum', 'Radial distance from the pitch circle to the bottom of the tooth space.'],
    ['addc', 'Addendum circle', 'The circle through the tops of the teeth.'],
    ['root', 'Dedendum (root) circle', 'The circle through the bottoms of the teeth: diameter D − 2 × dedendum.'],
    ['base', 'Base circle', 'The circle the involute is unwound from: diameter D cos φ. Below it a tooth flank cannot be an involute.'],
    ['pc', 'Circular pitch', 'Distance along the pitch circle from a point on one tooth to the same point on the next: p<sub>c</sub> = πD/T = πm.'],
    ['thick', 'Tooth thickness', 'Width of a tooth measured along the pitch circle: πm/2 for standard teeth.'],
    ['space', 'Tooth space', 'Width of the gap between teeth along the pitch circle. Space − thickness = backlash (zero in this drawing).'],
    ['clear', 'Clearance', 'Radial gap between the tip of one gear and the root of the other: dedendum − addendum.'],
    ['work', 'Working depth', 'Depth to which the two gears’ teeth overlap: the sum of the two addenda.'],
    ['total', 'Total depth', 'Addendum + dedendum.'],
    ['face', 'Face and flank', 'Face: the tooth surface above the pitch circle. Flank: the surface below it.'],
    ['top', 'Top land', 'The surface at the top of the tooth.'],
    ['path', 'Path of contact', 'The part of the line of action where the teeth actually touch: from K, where the wheel’s addendum circle cuts it, to L, where the pinion’s does.']];
  let sel = 'pitch';
  $('tm_chips').innerHTML = TERMS.map(([k, n]) => `<button type="button" data-k="${k}" aria-pressed="${k === sel}">${n}</button>`).join('');
  $('tm_chips').querySelectorAll('button').forEach(b => b.addEventListener('click', () => { sel = b.dataset.k; $('tm_chips').querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', x === b)); draw(); }));
  controls($('tm_c'), [['m', 'Module', 1, 20, 0.25, 5, 'mm'], ['z1', 'Teeth on pinion', 10, 60, 1, 20], ['z2', 'Teeth on wheel', 12, 120, 1, 40]], P, () => draw());
  $('tm_sys').addEventListener('change', () => draw());
  function draw(){ const [ph, ha, hf] = SYS[$('tm_sys').value], m = P.m, z1 = P.z1, z2 = Math.max(P.z2, 12);
    const g1 = gearGeo(z1, m, ph, ha, hf), g2 = gearGeo(z2, m, ph, ha, hf), r1 = g1.r, r2 = g2.r, pc = Math.PI*m;
    const span = 3.2*pc, V = view(cv, { xl:-span, xr:span, yb:-span*0.62, yt:span*0.62 }, 10), { ctx } = V;
    const O1 = [0, -r1], O2 = [0, r2], c1 = Math.PI/2, c2 = -Math.PI/2 + Math.PI/z2;
    const hl = k => sel === k, faint = 0.55;
    ctx.save(); ctx.beginPath(); ctx.rect(0, 0, V.W, V.H); ctx.clip();
    polyPolar(ctx, V, gearPath(g1, c1), O1[0], O1[1], alpha(C.crank, 0.16), C.crank, 1.3);
    polyPolar(ctx, V, gearPath(g2, c2), O2[0], O2[1], alpha(C.rod, 0.14), C.rod, 1.3);
    const cc = (c, r, col, dash, lw) => circ(ctx, V, c[0], c[1], r, col, dash, lw);
    cc(O1, r1, hl('pitch') ? C.ink : alpha(C.ink, faint), [10, 3, 2, 3], hl('pitch') ? 2.5 : 1); cc(O2, r2, hl('pitch') ? C.ink : alpha(C.ink, faint), [10, 3, 2, 3], hl('pitch') ? 2.5 : 1);
    if (hl('addc') || hl('work')){ cc(O1, g1.ra, C.acc, [], 2.2); cc(O2, g2.ra, C.acc, [], 2.2); }
    if (hl('root') || hl('clear')){ cc(O1, g1.rf, C.vel, [], 2.2); if (hl('root')) cc(O2, g2.rf, C.vel, [], 2.2); }
    if (hl('clear')) cc(O2, g2.ra, C.acc, [], 2.2);
    if (hl('base') || hl('phi') || hl('path')){ cc(O1, g1.rb, C.muted, [3, 3], hl('base') ? 2.2 : 1); cc(O2, g2.rb, C.muted, [3, 3], hl('base') ? 2.2 : 1); }
    const P1 = (r, t) => [V.PX(O1[0] + r*Math.cos(t)), V.PY(O1[1] + r*Math.sin(t))];
    const arcP = (r, t0, t1, col, lw = 3) => { ctx.strokeStyle = col; ctx.lineWidth = lw; ctx.beginPath(); for (let i = 0; i <= 24; i++){ const [x, y] = P1(r, t0 + (t1 - t0)*i/24); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); } ctx.stroke(); };
    const radial = (t, ra, rb, col, txt, side = 1) => { const [x0, y0] = P1(ra, t), [x1, y1] = P1(rb, t); arrow(ctx, (x0 + x1)/2, (y0 + y1)/2, x0, y0, col, 1.8); arrow(ctx, (x0 + x1)/2, (y0 + y1)/2, x1, y1, col, 1.8);
      lbl(ctx, txt, (x0 + x1)/2 + side*10, (y0 + y1)/2, col, side > 0 ? 'left' : 'right'); };
    const tr = c1 - 2*TAU/z1;   // a pinion tooth to the right of P, clear of the mesh
    if (hl('add')) radial(tr, r1, g1.ra, C.acc, `addendum ${n2(ha*m)} mm`);
    if (hl('ded')) radial(tr + Math.PI/z1, r1, g1.rf, C.vel, `dedendum ${n2(hf*m)} mm`);
    if (hl('total')) radial(tr, g1.rf, g1.ra, C.force, `total depth ${n2((ha + hf)*m)} mm`);
    if (hl('pc')){ arcP(r1, tr - g1.psi(r1), tr - g1.psi(r1) + TAU/z1, C.acc); const [x, y] = P1(r1*1.0, tr + Math.PI/z1 - g1.psi(r1)); lbl(ctx, `p_c = πm = ${n2(pc)} mm`, x, y - 22, C.acc, 'center'); }
    if (hl('thick')){ arcP(r1, tr - g1.psi(r1), tr + g1.psi(r1), C.acc); const [x, y] = P1(r1, tr); lbl(ctx, `thickness ${n2(pc/2)} mm`, x + 8, y - 16, C.acc, 'left'); }
    if (hl('space')){ arcP(r1, tr - TAU/z1 + g1.psi(r1), tr - g1.psi(r1), C.vel); const [x, y] = P1(r1, tr - Math.PI/z1); lbl(ctx, `space ${n2(pc/2)} mm`, x + 8, y - 16, C.vel, 'left'); }
    if (hl('top')){ const tp = g1.psi(g1.fr.rt); arcP(g1.fr.rt, tr - tp, tr + tp, C.acc, 4); const [x, y] = P1(g1.ra, tr); lbl(ctx, 'top land', x + 8, y - 10, C.acc, 'left'); }
    if (hl('face')){ const T = toothPath(g1, tr), face = T.filter(([r]) => r >= r1 - 1e-9), flank = T.filter(([r]) => r <= r1 + 1e-9);
      const ln = (pts, col) => { ctx.strokeStyle = col; ctx.lineWidth = 4; const L = pts.filter(([r, t]) => t < tr); ctx.beginPath(); L.forEach(([r, t], i) => { const [x, y] = P1(r, t); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); };
      ln(face, C.acc); ln(flank, C.vel); const [x1, y1] = P1(g1.ra*0.99, tr - g1.psi(g1.ra)), [x2, y2] = P1((r1 + g1.rf)/2, tr - g1.psi(r1));
      lbl(ctx, 'face', x1 + 10, y1, C.acc); lbl(ctx, 'flank', x2 + 10, y2 + 6, C.vel); }
    if (hl('clear')){ const t2 = c2 + TAU/z2*1, tipX = O2[0] + g2.ra*Math.cos(t2), tipY = O2[1] + g2.ra*Math.sin(t2), a = Math.atan2(tipY - O1[1], tipX - O1[0]);
      const [rx, ry] = P1(g1.rf, a); arrow(ctx, V.PX(tipX), V.PY(tipY), rx, ry, C.ink, 1.8); lbl(ctx, `clearance ${n2((hf - ha)*m)} mm`, rx + 10, ry, C.ink); }
    if (hl('work')){ const x = V.PX(span*0.7); arrow(ctx, x, V.PY(0), x, V.PY(ha*m), C.force, 1.8); arrow(ctx, x, V.PY(0), x, V.PY(-ha*m), C.force, 1.8); lbl(ctx, `working depth ${n2(2*ha*m)} mm`, x - 8, V.PY(0), C.force, 'right'); }
    // line of action and path of contact
    const s = Math.sin(g1.phi), c = Math.cos(g1.phi), MP = r1*s, PN = r2*s, KP = Math.sqrt(g2.ra**2 - g2.rb**2) - PN, PL = Math.sqrt(g1.ra**2 - g1.rb**2) - MP;
    const along = u => [V.PX(u*c), V.PY(u*s)];   // pinion below, wheel above: line of action rises to the right
    if (hl('phi') || hl('path')){ ctx.strokeStyle = alpha(C.force, 0.6); ctx.lineWidth = 1.4; const [ax, ay] = along(-MP*1.6), [bx, by] = along(PN*1.6); ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke(); }
    if (hl('phi')){ ctx.strokeStyle = C.muted; ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(V.PX(-span), V.PY(0)); ctx.lineTo(V.PX(span), V.PY(0)); ctx.stroke(); ctx.setLineDash([]);
      ctx.strokeStyle = C.force; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(V.PX(0), V.PY(0), 46, -g1.phi, 0); ctx.stroke(); lbl(ctx, `φ = ${ph}°`, V.PX(0) + 52, V.PY(0) - 12, C.force); }
    if (hl('path')){ const [kx, ky] = along(-KP), [lx, ly] = along(PL); ctx.strokeStyle = C.force; ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(kx, ky); ctx.lineTo(lx, ly); ctx.stroke();
      dot(ctx, kx, ky, C.force); dot(ctx, lx, ly, C.force); lbl(ctx, 'K', kx - 8, ky + 4, C.force, 'right'); lbl(ctx, 'L', lx + 8, ly - 4, C.force); lbl(ctx, `KL = ${n2(KP + PL)} mm`, lx + 8, ly + 16, C.force); }
    dot(ctx, V.PX(0), V.PY(0), hl('P') ? C.acc : C.ink, hl('P') ? 6 : 3.5); if (hl('P') || hl('pitch')) lbl(ctx, 'P', V.PX(0) - 10, V.PY(0) + 12, C.ink, 'right', 13);
    lbl(ctx, 'wheel', V.PX(-span*0.9), V.PY(span*0.5), C.rod); lbl(ctx, 'pinion', V.PX(-span*0.9), V.PY(-span*0.5), C.crank);
    ctx.restore();
    const t = TERMS.find(x => x[0] === sel); $('tm_cap').innerHTML = `<b>${t[1]}.</b> ${t[2]}`;
    $('tm_sub').textContent = `module ${m} mm · ${z1} and ${z2} teeth · φ = ${ph}°`;
    const st = [['Pitch diameters', `${n2(m*z1, 1)} / ${n2(m*z2, 1)}`, 'mm'], ['Centre distance', n2(m*(z1 + z2)/2, 1), 'mm'], ['Circular pitch πm', n2(pc), 'mm'], ['Diametral pitch T/D', n2(1/m, 3), 'per mm'], ['Addendum', n2(ha*m), 'mm'], ['Dedendum', n2(hf*m), 'mm'],
      ['Clearance', n2((hf - ha)*m), 'mm'], ['Working depth', n2(2*ha*m), 'mm'], ['Total depth', n2((ha + hf)*m), 'mm'], ['Tooth thickness', n2(pc/2), 'mm'], ['Base circle, pinion', n2(2*g1.rb, 1), 'mm dia.'], ['Root circle, pinion', n2(2*g1.rf, 1), 'mm dia.'], ['Velocity ratio ω₁/ω₂', n2(z2/z1, 3), '']];
    $('tm_s').innerHTML = st.map(([k, v, u]) => `<div class="stat"><div class="k">${k}</div><div class="v">${v} <small>${u}</small></div></div>`).join(''); }
  watchSize(cv, draw); onRedraw(draw); draw(); }

/* ================= 5.4 law of gearing: meshing pair with moveable centres ================= */
{ const P = {}, cv = $('lg'); let th = 0, M;
  controls($('lg_c'), [['z1', 'Teeth on pinion', 12, 40, 1, 18], ['z2', 'Teeth on wheel', 12, 80, 1, 30], ['phi', 'Pressure angle', 14.5, 25, 0.5, 20, '°'], ['da', 'Move the centres apart by', 0, 0.6, 0.01, 0, '× module']], P, () => { rebuild(); draw(); });
  function rebuild(){ M = mesh(P.z1, P.z2, 1, P.phi, 1, 1.25, P.da); }
  function draw(){ const span = M.r1w + M.r2w, V = view(cv, { xl:-2*M.r1w - 1.2, xr:2*M.r2w + 1.2, yb:-Math.max(M.g1.ra, M.g2.ra) - 0.5, yt:Math.max(M.g1.ra, M.g2.ra) + 0.5 }, 10), { ctx } = V;
    const { O1, O2 } = drawPair(ctx, V, M, th, { mark:true });
    circ(ctx, V, O1, 0, M.g1.rb, C.muted, [3, 3]); circ(ctx, V, O2, 0, M.g2.rb, C.muted, [3, 3]); circ(ctx, V, O1, 0, M.r1w, alpha(C.ink, 0.5), [8, 3, 2, 3]); circ(ctx, V, O2, 0, M.r2w, alpha(C.ink, 0.5), [8, 3, 2, 3]);
    const s = Math.sin(M.alpha), c = Math.cos(M.alpha), pt = u => [V.PX(u*s), V.PY(u*c)];
    const [mx, my] = pt(-M.MP), [nx, ny] = pt(M.PN); ctx.strokeStyle = alpha(C.force, 0.7); ctx.lineWidth = 1.6; ctx.beginPath(); ctx.moveTo(mx, my); ctx.lineTo(nx, ny); ctx.stroke();
    ctx.strokeStyle = alpha(C.muted, 0.7); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(V.PX(O1), V.PY(0)); ctx.lineTo(mx, my); ctx.moveTo(V.PX(O2), V.PY(0)); ctx.lineTo(nx, ny); ctx.stroke();
    dot(ctx, mx, my, C.force, 3.5); dot(ctx, nx, ny, C.force, 3.5); lbl(ctx, 'M', mx - 8, my, C.force, 'right'); lbl(ctx, 'N', nx + 8, ny, C.force);
    const t = ((th*180/Math.PI % 360) + 540) % 360 - 180, u0 = (th - Math.round(th/(TAU/M.z1))*TAU/M.z1)*M.g1.rb;
    for (const u of M.contacts(u0)){ const [x, y] = pt(u); dot(ctx, x, y, C.vel, 5.5); ctx.strokeStyle = alpha(C.vel, 0.9); ctx.lineWidth = 1; }
    dot(ctx, V.PX(0), V.PY(0), C.ink, 4); lbl(ctx, 'P', V.PX(0) + 6, V.PY(0) + 14, C.ink); lbl(ctx, 'O₁', V.PX(O1) - 8, V.PY(0) - 12, C.ink, 'right'); lbl(ctx, 'O₂', V.PX(O2) + 8, V.PY(0) - 12, C.ink);
    lbl(ctx, `ω₂/ω₁ = O₁P/O₂P = ${n2(M.r1w/M.r2w, 4)}`, 12, 16, C.ink, 'left', 12.5);
    $('lg_sub').textContent = `${M.contacts(u0).length} pair${M.contacts(u0).length > 1 ? 's' : ''} touching now`;
    const u = ' <small>× m</small>';
    $('lg_s').innerHTML = [['Speed ratio ω₂/ω₁ = z₁/z₂', n2(M.ratio, 4)], ['Working pressure angle', n2(M.alpha/DEG, 2) + '°'], ['Centre distance', n2(M.aw, 3) + u], ['Working pitch radii', `${n2(M.r1w, 3)}, ${n2(M.r2w, 3)}` + u], ['Backlash on pitch circle', n2(Math.abs(M.j) < 1e-9 ? 0 : M.j, 3) + u], ['Contact ratio', n2(M.eps, 2)]]
      .map(([k, v]) => `<div class="stat${k === 'Contact ratio' && M.eps < 1 ? ' bad' : ''}"><div class="k">${k}</div><div class="v">${v}</div></div>`).join(''); }
  rebuild(); const an = animate(cv, dt => { th += dt*0.35; draw(); }); playButton($('lg_play'), an); watchSize(cv, draw); onRedraw(draw); draw(); }

/* ================= 5.5 involute and cycloid generation ================= */
{ const cv = $('cvv'); let mode = 'inv', t = 0;
  document.querySelectorAll('[data-cur]').forEach(b => b.addEventListener('click', () => { mode = b.dataset.cur; t = 0; document.querySelectorAll('[data-cur]').forEach(x => x.setAttribute('aria-pressed', x === b)); draw(); }));
  function draw(){ const V = view(cv, { xl:-1.5, xr:2.6, yb:-1.35, yt:1.75 }, 12), { ctx } = V, R = 1;
    const T = mode === 'inv' ? 0.15 + (t % 6)/6*2.2 : 0.15 + (t % 6)/6*(mode === 'epi' ? TAU*0.3/0.3*0.999 : TAU*0.999);
    if (mode === 'inv'){ $('cv_t').textContent = 'Drawing an involute'; circ(ctx, V, 0, 0, R, C.muted, [], 2); lbl(ctx, 'base circle', V.PX(-0.7), V.PY(-0.8), C.muted, 'center');
      // involute from A = (R, 0), unwinding anticlockwise: point = R(cos u + u sin u, sin u − u cos u)
      const P = u => [R*(Math.cos(u) + u*Math.sin(u)), R*(Math.sin(u) - u*Math.cos(u))];
      ctx.strokeStyle = alpha(C.acc, 0.25); ctx.lineWidth = 1.5; ctx.beginPath(); for (let i = 0; i <= 120; i++){ const [x, y] = P(2.4*i/120); i ? ctx.lineTo(V.PX(x), V.PY(y)) : ctx.moveTo(V.PX(x), V.PY(y)); } ctx.stroke();
      ctx.strokeStyle = C.acc; ctx.lineWidth = 3; ctx.beginPath(); for (let i = 0; i <= 80; i++){ const [x, y] = P(T*i/80); i ? ctx.lineTo(V.PX(x), V.PY(y)) : ctx.moveTo(V.PX(x), V.PY(y)); } ctx.stroke();
      const tp = [R*Math.cos(T), R*Math.sin(T)], q = P(T);
      ctx.strokeStyle = C.crank; ctx.lineWidth = 2.5; ctx.beginPath(); ctx.arc(V.PX(0), V.PY(0), R*V.sc, -T, 0, false); ctx.stroke();
      ctx.strokeStyle = C.vel; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(V.PX(tp[0]), V.PY(tp[1])); ctx.lineTo(V.PX(q[0]), V.PY(q[1])); ctx.stroke();
      // tangent to the involute at q is perpendicular to the string
      const d = [Math.cos(T), Math.sin(T)]; ctx.strokeStyle = alpha(C.ink, 0.6); ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.moveTo(V.PX(q[0] - d[0]*0.5), V.PY(q[1] - d[1]*0.5)); ctx.lineTo(V.PX(q[0] + d[0]*0.5), V.PY(q[1] + d[1]*0.5)); ctx.stroke(); ctx.setLineDash([]);
      dot(ctx, V.PX(R), V.PY(0), C.ink); lbl(ctx, 'A', V.PX(R) + 6, V.PY(0) + 12, C.ink); dot(ctx, V.PX(tp[0]), V.PY(tp[1]), C.vel); dot(ctx, V.PX(q[0]), V.PY(q[1]), C.acc, 5);
      $('cv_cap').innerHTML = `The string (green) is unwound from A. Its free length equals the arc it came off (blue): ${n2(T*R)} × r<sub>b</sub>. The string is always <b>normal</b> to the involute (the dashed tangent is at right angles to it) and always <b>tangent</b> to the base circle. That is why every contact between two involute teeth lies on the common tangent of the base circles.`;
      $('cv_sub').textContent = 'a point on a taut string unwound from the base circle';
    } else { const epi = mode === 'epi', a = 0.3, Rp = 1; $('cv_t').textContent = epi ? 'Drawing an epicycloid (tooth face)' : 'Drawing a hypocycloid (tooth flank)';
      circ(ctx, V, 0, 0, Rp, C.ink, [10, 3, 2, 3], 1.5); lbl(ctx, 'pitch circle', V.PX(-0.75), V.PY(-0.85), C.muted, 'center');
      const k = epi ? (Rp + a)/a : (Rp - a)/a, sg = epi ? 1 : -1, ang = T*a/Rp;   // rolling circle has turned through T, its centre through ang
      const ctr = s => [(Rp + sg*a)*Math.cos(s), (Rp + sg*a)*Math.sin(s)], pnt = s => { const [cx, cy] = ctr(s), b = epi ? s*k : -s*k; return [cx - sg*a*Math.cos(b), cy - sg*a*Math.sin(b)]; };
      const full = epi ? TAU*a/Rp : TAU*a/Rp;
      ctx.strokeStyle = alpha(C.acc, 0.25); ctx.lineWidth = 1.5; ctx.beginPath(); for (let i = 0; i <= 140; i++){ const [x, y] = pnt(full*i/140); i ? ctx.lineTo(V.PX(x), V.PY(y)) : ctx.moveTo(V.PX(x), V.PY(y)); } ctx.stroke();
      ctx.strokeStyle = C.acc; ctx.lineWidth = 3; ctx.beginPath(); for (let i = 0; i <= 80; i++){ const [x, y] = pnt(Math.min(ang, full)*i/80); i ? ctx.lineTo(V.PX(x), V.PY(y)) : ctx.moveTo(V.PX(x), V.PY(y)); } ctx.stroke();
      const s = Math.min(ang, full), [cx, cy] = ctr(s); circ(ctx, V, cx, cy, a, C.crank, [], 2); const [px, py] = pnt(s);
      ctx.strokeStyle = C.crank; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(V.PX(cx), V.PY(cy)); ctx.lineTo(V.PX(px), V.PY(py)); ctx.stroke(); dot(ctx, V.PX(px), V.PY(py), C.acc, 5);
      const cp = [Rp*Math.cos(s), Rp*Math.sin(s)]; dot(ctx, V.PX(cp[0]), V.PY(cp[1]), C.vel, 4); ctx.strokeStyle = alpha(C.vel, 0.8); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(V.PX(cp[0]), V.PY(cp[1])); ctx.lineTo(V.PX(px), V.PY(py)); ctx.stroke(); ctx.setLineDash([]);
      $('cv_cap').innerHTML = `A circle (blue) rolls without slipping on the ${epi ? 'outside' : 'inside'} of the pitch circle; a point on its rim traces the ${epi ? 'epicycloid, the <b>face</b> of a cycloidal tooth' : 'hypocycloid, the <b>flank</b> of a cycloidal tooth'}. The green line from the rolling contact to the tracing point is the normal to the curve, and it passes through the contact on the pitch circle. When two cycloidal teeth mesh, that contact is the pitch point, so the law of gearing holds here too.`;
      $('cv_sub').textContent = epi ? 'rolling outside the pitch circle' : 'rolling inside the pitch circle'; } }
  const an = animate(cv, dt => { t += dt*0.9; draw(); }); playButton($('cv_play'), an); watchSize(cv, draw); onRedraw(draw); draw(); }

/* ================= 5.6 tooth shapes of the three systems ================= */
{ const P = {}, cv = $('sp');
  controls($('sp_c'), [['z', 'Number of teeth', 12, 80, 1, 24]], P, () => draw());
  function draw(){ const z = P.z, m = 1, r = z/2, V = view(cv, { xl:-2.3, xr:2.3, yb:r - 1.6, yt:r + 1.25 }, 14), { ctx } = V;
    const sys = [[14.5, 1, 1.25, C.crank], [20, 1, 1.25, C.rod], [20, 0.8, 1, C.vel]];
    for (const [ph, ha, hf, col] of sys){ const g = gearGeo(z, m, ph, ha, hf, 0, 24), pts = [];
      for (const k of [-1, 0, 1]){ const c = Math.PI/2 + k*TAU/z; toothPath(g, c).forEach(p => pts.push(p)); }
      ctx.strokeStyle = col; ctx.lineWidth = 2.2; ctx.beginPath(); pts.forEach(([rr, t], i) => { const x = V.PX(rr*Math.cos(t)), y = V.PY(rr*Math.sin(t)); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); }); ctx.stroke(); }
    ctx.strokeStyle = alpha(C.ink, 0.6); ctx.setLineDash([10, 3, 2, 3]); ctx.beginPath(); ctx.arc(V.PX(0), V.PY(0), r*V.sc, -Math.PI/2 - 0.3, -Math.PI/2 + 0.3); ctx.stroke(); ctx.setLineDash([]);
    lbl(ctx, 'pitch circle', V.PX(1.9), V.PY(r) - 8, C.muted, 'center'); }
  watchSize(cv, draw); onRedraw(draw); draw(); }

/* ================= 5.7 contact ratio ================= */
{ const P = {}, cv = $('cr'), cv2 = $('cr2'); let th = 0, M;
  controls($('cr_c'), [['z1', 'Teeth on pinion', 12, 40, 1, 20], ['z2', 'Teeth on wheel', 12, 80, 1, 40], ['phi', 'Pressure angle', 14.5, 25, 0.5, 20, '°'], ['ha', 'Addendum', 0.6, 1.3, 0.05, 1, '× module']], P, () => { M = mesh(P.z1, P.z2, 1, P.phi, P.ha, 1.25); draw(); });
  M = mesh(P.z1, P.z2, 1, P.phi, P.ha, 1.25);
  function draw(){ const V = view(cv, { xl:-M.r1w*0.5 - 2.5, xr:M.r2w*0.35 + 2.5, yb:-4.2, yt:4.2 }, 6), { ctx } = V;
    drawPair(ctx, V, M, th, { mark:true });
    const s = Math.sin(M.alpha), c = Math.cos(M.alpha), pt = u => [V.PX(u*s), V.PY(u*c)];
    const [mx, my] = pt(-M.MP), [nx, ny] = pt(M.PN), [kx, ky] = pt(-M.approach), [lx, ly] = pt(M.recess);
    ctx.strokeStyle = alpha(C.force, 0.5); ctx.lineWidth = 1.4; ctx.beginPath(); ctx.moveTo(mx, my); ctx.lineTo(nx, ny); ctx.stroke();
    ctx.strokeStyle = C.force; ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(kx, ky); ctx.lineTo(lx, ly); ctx.stroke();
    circ(ctx, V, -M.r1w, 0, M.g1.ra, alpha(C.crank, 0.7), [2, 3]); circ(ctx, V, M.r2w, 0, M.g2.ra, alpha(C.rod, 0.7), [2, 3]);
    lbl(ctx, 'K', kx - 8, ky, C.force, 'right'); lbl(ctx, 'L', lx + 8, ly, C.force); dot(ctx, V.PX(0), V.PY(0), C.ink, 3.5); lbl(ctx, 'P', V.PX(0) + 7, V.PY(0) + 12, C.ink);
    const u0 = (th - Math.round(th/(TAU/M.z1))*TAU/M.z1)*M.g1.rb, cs = M.contacts(u0);
    for (const u of cs){ const [x, y] = pt(u); dot(ctx, x, y, C.vel, 6); }
    lbl(ctx, `${cs.length} pair${cs.length > 1 ? 's' : ''} in contact`, 10, 16, cs.length > 1 ? C.ok : C.crank, 'left', 13);
    // right: pairs in contact and sliding speed over one pitch
    const X = [], N = [], VS = []; const pbA = TAU/M.z1; for (let i = 0; i <= 400; i++){ const a = -pbA*1.5 + 3*pbA*i/400, u = (a - Math.round(a/pbA)*pbA)*M.g1.rb; X.push(a/DEG); N.push(M.contacts(u).length); }
    for (let i = 0; i <= 400; i++){ const u = (-M.approach - 0.15) + (M.path + 0.3)*i/400; VS.push(u); }
    const cur = ((th % pbA) + pbA*1.5) % (3*pbA) - pbA*1.5;
    chart(cv2, { x:X, xlab:'pinion angle [°] (three tooth pitches)', ylab:'pairs in contact', series:[{ y:N, color:C.vel, label:'pairs touching', w:2.2 }], hlines:[{ v:M.eps, color:C.force, label:`average = contact ratio ${n2(M.eps, 2)}`, right:true }], cursor:cur/DEG, cursorIndex:Math.round((cur/DEG - X[0])/(X[400] - X[0])*400), ymin:0, value:'' });
    $('cr_sub').textContent = `contact ratio ${n2(M.eps, 2)}`;
    $('cr_s').innerHTML = [['Path of approach KP', n2(M.approach, 3) + ' m'], ['Path of recess PL', n2(M.recess, 3) + ' m'], ['Path of contact KL', n2(M.path, 3) + ' m'], ['Arc of contact', n2(M.path/Math.cos(M.alpha), 3) + ' m'], ['Contact ratio', n2(M.eps, 2)], ['Interference', M.approach > M.MP + 1e-9 || M.recess > M.PN + 1e-9 ? 'yes' : 'no']]
      .map(([k, v]) => `<div class="stat${(k === 'Interference' && v === 'yes') || (k === 'Contact ratio' && M.eps < 1) ? ' bad' : ''}"><div class="k">${k}</div><div class="v">${v.replace(' m', ' <small>× m</small>')}</div></div>`).join(''); }
  const an = animate(cv, dt => { th += dt*0.25; draw(); }); playButton($('cr_play'), an); watchSize(cv, draw); watchSize(cv2, draw); onRedraw(draw); draw(); }

/* ================= 5.8 undercut by a rack cutter, and interference points ================= */
{ const P = {}, cv = $('uc'), cv2 = $('it'); let prog = 1, cutting = false;
  controls($('uc_c'), [['z', 'Teeth on the pinion', 8, 30, 1, 12], ['phi', 'Pressure angle', 14.5, 25, 0.5, 20, '°'], ['z2', 'Teeth on the mating wheel (right)', 12, 80, 1, 40]], P, () => { prog = 1; draw(); });
  function draw(){ const z = P.z, m = 1, r = z/2, phi = P.phi*DEG, V = view(cv, { xl:-r*0.55 - 1.8, xr:r*0.55 + 1.8, yb:r - 3.4, yt:r + 1.9 }, 8), { ctx } = V;
    const zmin = 2/Math.sin(phi)**2, under = z < zmin;
    // blank, then cut away every position of the rack cutter (cutter addendum 1.25m makes the root, its dedendum 1m makes the tip)
    const off = document.createElement('canvas'); off.width = V.W*(window.devicePixelRatio || 1); off.height = V.H*(window.devicePixelRatio || 1);
    const o = off.getContext('2d'), dpr = window.devicePixelRatio || 1; o.setTransform(dpr, 0, 0, dpr, 0, 0);
    o.fillStyle = alpha(C.crank, 0.55); o.beginPath(); o.arc(V.PX(0), V.PY(0), (r + m)*V.sc, 0, 7); o.fill();
    o.globalCompositeOperation = 'destination-out'; o.fillStyle = '#000';
    const p = Math.PI*m, tn = Math.tan(phi);
    const steps = 160, sMax = (p*3.2)/r;
    const rackAt = (s, ctxR, fill) => { // pinion turned by s anticlockwise = rack moved by r·s; draw in the pinion frame
      const cs = Math.cos(-s), sn = Math.sin(-s), T = (x, y) => [x*cs - y*sn, x*sn + y*cs];
      ctxR.beginPath(); for (let k = -5; k <= 5; k++){ const xc = k*p - r*s;   // tooth centre along the pitch line (horizontal), pitch line at y = r
        const pts = [[xc - p/4 + 1.25*m*tn, r - 1.25*m], [xc - p/4 - 1.2*m*tn, r + m*1.2], [xc + p/4 + 1.2*m*tn, r + m*1.2], [xc + p/4 - 1.25*m*tn, r - 1.25*m]];
        pts.forEach(([x, y], i) => { const [X, Y] = T(x, y); i ? ctxR.lineTo(V.PX(X), V.PY(Y)) : ctxR.moveTo(V.PX(X), V.PY(Y)); }); ctxR.closePath(); }
      if (fill) ctxR.fill(); };
    // cutter teeth are the solid parts; to cut the pinion we remove the area the cutter teeth sweep
    const nShow = Math.round(steps*prog);
    for (let i = 0; i <= nShow; i++){ const s = -sMax + 2*sMax*i/steps; rackAt(s, o, true); }
    o.globalCompositeOperation = 'source-over';
    ctx.drawImage(off, 0, 0, V.W, V.H);
    // current cutter position
    const sNow = prog >= 1 ? 0 : -sMax + 2*sMax*prog;
    ctx.fillStyle = alpha(C.rod, 0.35); ctx.strokeStyle = C.rod; ctx.lineWidth = 1.2; rackAt(sNow, ctx, true); ctx.stroke();
    circ(ctx, V, 0, 0, r, alpha(C.ink, 0.6), [10, 3, 2, 3]); circ(ctx, V, 0, 0, r*Math.cos(phi), C.muted, [3, 3], 1.5);
    lbl(ctx, 'base circle', V.PX(-r*0.45), V.PY(r*Math.cos(phi)) + 4, C.muted, 'center', 11); lbl(ctx, 'rack cutter', V.PX(r*0.35), V.PY(r + 1.5), C.rod, 'center', 11);
    $('uc_sub').textContent = `fewest teeth that avoid undercut with a rack: 2/sin²φ = ${n2(zmin, 1)}`;
    $('uc_cap').innerHTML = under ? `<b>Undercut.</b> With ${z} teeth at ${P.phi}° the cutter’s straight sides reach below the base circle and scoop metal out of the root, so the tooth is thin where it should be strongest. Use at least ${Math.ceil(zmin - 1e-9)} teeth, a bigger pressure angle, or stub teeth.` : `<b>No undercut.</b> ${z} teeth at ${P.phi}° is enough: the cutter stays outside the base circle while it forms the involute.`;
    drawInterf(); }
  function drawInterf(){ const z1 = P.z, z2 = P.z2, phi = P.phi, M = mesh(z1, z2, 1, phi), V = view(cv2, { xl:-M.r1w*0.55 - 1, xr:M.r2w*0.35 + 1, yb:-M.r1w*0.6, yt:M.r1w*0.6 }, 8), { ctx } = V;
    drawPair(ctx, V, M, 0);
    circ(ctx, V, -M.r1w, 0, M.g1.rb, C.muted, [3, 3], 1.4); circ(ctx, V, M.r2w, 0, M.g2.rb, C.muted, [3, 3], 1.4);
    const s = Math.sin(M.alpha), c = Math.cos(M.alpha), pt = u => [V.PX(u*s), V.PY(u*c)], [mx, my] = pt(-M.MP), [nx, ny] = pt(M.PN), [kx, ky] = pt(-M.approach), [lx, ly] = pt(M.recess);
    ctx.strokeStyle = alpha(C.force, 0.6); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(mx, my); ctx.lineTo(nx, ny); ctx.stroke();
    const bad1 = M.approach > M.MP + 1e-9, bad2 = M.recess > M.PN + 1e-9;
    ctx.strokeStyle = C.force; ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(bad1 ? mx : kx, bad1 ? my : ky); ctx.lineTo(bad2 ? nx : lx, bad2 ? ny : ly); ctx.stroke();
    if (bad1){ ctx.strokeStyle = C.bad; ctx.lineWidth = 5; ctx.beginPath(); ctx.moveTo(kx, ky); ctx.lineTo(mx, my); ctx.stroke(); }
    circ(ctx, V, M.r2w, 0, M.g2.ra, bad1 ? C.bad : alpha(C.rod, 0.8), [2, 3], 1.6);
    dot(ctx, mx, my, C.ink); dot(ctx, nx, ny, C.ink); lbl(ctx, 'M', mx - 8, my, C.ink, 'right'); lbl(ctx, 'N', nx + 8, ny, C.ink); lbl(ctx, 'K', kx - 8, ky + 12, bad1 ? C.bad : C.force, 'right');
    lbl(ctx, bad1 ? `Interference: wheel tip passes M (KP ${n2(M.approach, 2)}m > MP ${n2(M.MP, 2)}m)` : `No interference: K lies between M and P`, 10, 16, bad1 ? C.bad : C.ok, 'left', 12.5); }
  const an = animate(cv, dt => { if (prog < 1){ prog = Math.min(1, prog + dt*0.18); draw(); } });
  $('uc_cut').addEventListener('click', () => { prog = 0; an.playing = true; $('uc_play').textContent = 'Pause'; });
  playButton($('uc_play'), an); watchSize(cv, draw); watchSize(cv2, drawInterf); onRedraw(draw); draw(); }

/* ================= 5.9 minimum teeth chart ================= */
const tminWheel = (G, phi, Aw) => { const s2 = Math.sin(phi*DEG)**2; return 2*Aw/(G*(Math.sqrt(1 + (1/G)*(1/G + 2)*s2) - 1)); };
const tminPin = (G, phi, Ap) => { const s2 = Math.sin(phi*DEG)**2; return 2*Ap/(Math.sqrt(1 + G*(G + 2)*s2) - 1); };
{ const P = {}, cv = $('mt');
  controls($('mt_c'), [['A', 'Addendum (both gears)', 0.6, 1.2, 0.05, 1, '× module'], ['G', 'Your gear ratio G = T/t', 1, 10, 0.05, 3]], P, () => draw());
  function draw(){ const x = []; for (let g = 1; g <= 10.0001; g += 0.05) x.push(+g.toFixed(2));
    const cols = { 14.5:C.crank, 20:C.rod, 25:C.vel };
    chart(cv, { x, xlab:'gear ratio G = T/t', ylab:'fewest pinion teeth', zero:false, ymin:0,
      series:[14.5, 20, 25].map(p => ({ y:x.map(g => tminWheel(g, p, P.A)), color:cols[p], label:`${p}°` })).concat([{ y:x.map(g => tminPin(g, 20, P.A)), color:C.muted, dash:[5, 4], w:1.4, label:'20°, pinion-tip limit' }]),
      vlines:[{ v:P.G, color:C.ink, label:`G = ${P.G}` }], cursor:null, value:'' });
    const t20 = tminWheel(P.G, 20, P.A), t145 = tminWheel(P.G, 14.5, P.A), t25 = tminWheel(P.G, 25, P.A);
    $('mt_s').innerHTML = [['14.5°', t145], ['20°', t20], ['25°', t25]].map(([k, v]) => `<div class="stat"><div class="k">${k}, G = ${P.G}</div><div class="v">${Math.ceil(v - 1e-9)} <small>teeth (${n2(v, 2)})</small></div></div>`).join('') +
      `<div class="stat"><div class="k">Rack (G → ∞), 20°</div><div class="v">${Math.ceil(2*P.A/Math.sin(20*DEG)**2 - 1e-9)} <small>teeth</small></div></div>`; }
  watchSize(cv, draw); onRedraw(draw); draw(); }

/* ================= worked examples ================= */
const deg = v => v*DEG;
example($('ex1'), { title:'Example 1', src:'tooth load from power (notes p. 93; Khurmi 12.1)',
  q:'A single reduction gear of 120 kW with a pinion 250 mm pitch circle diameter and speed 650 rpm is supported in bearings on either side. Find the total load due to the power transmitted, the pressure angle being 20°.',
  inputs:[['P', 'Power', 120, 'kW'], ['d', 'Pinion pitch diameter', 250, 'mm'], ['N', 'Speed', 650, 'rpm'], ['phi', 'Pressure angle', 20, '°']],
  solve:v => { const w = 2*Math.PI*v.N/60, T = v.P*1000/w, Ft = T/(v.d/2000), F = Ft/Math.cos(deg(v.phi));
    return [['ω = 2πN/60 =', `${n2(w)} rad/s`], ['Torque T = P/ω =', `${n2(T, 0)} N·m`], ['Tangential load F<sub>T</sub> = T/r =', `${n2(Ft, 0)} N`], ['Radial load F<sub>R</sub> = F<sub>T</sub> tan φ =', `${n2(Ft*Math.tan(deg(v.phi)), 0)} N`], ['Total load F = F<sub>T</sub>/cos φ =', `${n2(F, 0)} N`, true]]; } });
example($('ex2'), { title:'Example 2', src:'addendum from the arc of contact (notes p. 95; Khurmi 12.2)',
  q:'The number of teeth on each of two equal spur gears in mesh is 40. The teeth have 20° involute profile and the module is 6 mm. If the arc of contact is 1.75 times the circular pitch, find the addendum.',
  inputs:[['T', 'Teeth on each gear', 40, ''], ['phi', 'Pressure angle', 20, '°'], ['m', 'Module', 6, 'mm'], ['k', 'Arc of contact ÷ circular pitch', 1.75, '']],
  solve:v => { const pc = Math.PI*v.m, arc = v.k*pc, path = arc*Math.cos(deg(v.phi)), R = v.m*v.T/2, s = Math.sin(deg(v.phi)), c = Math.cos(deg(v.phi)), RA = Math.sqrt((path/2 + R*s)**2 + (R*c)**2);
    return [['Circular pitch p<sub>c</sub> = πm =', `${n2(pc)} mm`], ['Arc of contact = k p<sub>c</sub> =', `${n2(arc)} mm`], ['Path of contact = arc × cos φ =', `${n2(path)} mm`], ['Pitch radius R = mT/2 =', `${n2(R)} mm`],
      ['Equal gears: path/2 = √(R<sub>A</sub>² − R² cos² φ) − R sin φ, so R<sub>A</sub> = √((path/2 + R sin φ)² + R² cos² φ) =', `${n2(RA)} mm`], ['Addendum = R<sub>A</sub> − R =', `${n2(RA - R)} mm`, true]]; } });
example($('ex3'), { title:'Example 3', src:'angle of action and sliding (notes p. 96; Khurmi 12.4)',
  q:'Two involute gears of 20° pressure angle are in mesh. The pinion has 20 teeth and the gear ratio is 2. The module is 5 mm and the pitch line speed is 1.2 m/s; addendum = one module. Find (1) the angle turned through by the pinion while one pair of teeth is in mesh, and (2) the maximum velocity of sliding.',
  inputs:[['phi', 'Pressure angle', 20, '°'], ['t', 'Pinion teeth', 20, ''], ['G', 'Gear ratio', 2, ''], ['m', 'Module', 5, 'mm'], ['v', 'Pitch line speed', 1.2, 'm/s'], ['a', 'Addendum', 5, 'mm']],
  solve:v => { const s = Math.sin(deg(v.phi)), c = Math.cos(deg(v.phi)), r = v.m*v.t/2, R = r*v.G, rA = r + v.a, RA = R + v.a;
    const KP = Math.sqrt(RA*RA - R*R*c*c) - R*s, PL = Math.sqrt(rA*rA - r*r*c*c) - r*s, KL = KP + PL, arc = KL/c, ang = arc/(2*Math.PI*r)*360, w1 = v.v*1000/r, w2 = v.v*1000/R;
    return [['r = mt/2 =', `${n2(r)} mm`], ['R = mGt/2 =', `${n2(R)} mm`], ['r<sub>A</sub> = r + addendum, R<sub>A</sub> = R + addendum =', `${n2(rA)}, ${n2(RA)} mm`], ['Path of approach KP = √(R<sub>A</sub>² − R² cos² φ) − R sin φ =', `${n2(KP)} mm`],
      ['Path of recess PL = √(r<sub>A</sub>² − r² cos² φ) − r sin φ =', `${n2(PL)} mm`], ['Path of contact KL =', `${n2(KL)} mm`], ['Arc of contact = KL/cos φ =', `${n2(arc)} mm`], ['(1) Angle turned by pinion = arc × 360°/(2πr) =', `${n2(ang)}°`, true],
      ['ω<sub>1</sub> = v/r, ω<sub>2</sub> = v/R =', `${n2(w1)}, ${n2(w2)} rad/s`], ['(2) Largest sliding velocity = (ω<sub>1</sub> + ω<sub>2</sub>) × max(KP, PL) =', `${n2((w1 + w2)*Math.max(KP, PL), 1)} mm/s`, true]]; } });
example($('ex3b'), { title:'Example 3 (second part)', src:'sliding at engagement, pitch point and disengagement (notes p. 98; Khurmi 12.5)',
  q:'A pair of gears with 40 and 20 teeth are in mesh; the smaller, the driver, runs at 2000 rpm. Find the velocity of sliding at the point of engagement, at the pitch point and at the point of disengagement. Teeth are 20° involute, addendum 5 mm, module 5 mm. Also find the angle the pinion turns while any pair of teeth is in contact.',
  inputs:[['T', 'Wheel teeth', 40, ''], ['t', 'Pinion teeth', 20, ''], ['N', 'Pinion speed', 2000, 'rpm'], ['phi', 'Pressure angle', 20, '°'], ['a', 'Addendum', 5, 'mm'], ['m', 'Module', 5, 'mm']],
  solve:v => { const s = Math.sin(deg(v.phi)), c = Math.cos(deg(v.phi)), w1 = 2*Math.PI*v.N/60, w2 = w1*v.t/v.T, r = v.m*v.t/2, R = v.m*v.T/2, rA = r + v.a, RA = R + v.a;
    const KP = Math.sqrt(RA*RA - R*R*c*c) - R*s, PL = Math.sqrt(rA*rA - r*r*c*c) - r*s, arc = (KP + PL)/c;
    return [['ω<sub>1</sub>, ω<sub>2</sub> =', `${n2(w1)}, ${n2(w2)} rad/s`], ['KP =', `${n2(KP)} mm`], ['PL =', `${n2(PL)} mm`], ['Sliding at engagement K = (ω<sub>1</sub> + ω<sub>2</sub>) KP =', `${n2((w1 + w2)*KP, 0)} mm/s`, true], ['Sliding at the pitch point =', '0 (pure rolling)', true],
      ['Sliding at disengagement L = (ω<sub>1</sub> + ω<sub>2</sub>) PL =', `${n2((w1 + w2)*PL, 0)} mm/s`, true], ['Angle turned = (KL/cos φ) × 360°/(2πr) =', `${n2(arc/(2*Math.PI*r)*360)}°`, true]]; } });
example($('ex4'), { title:'Example 4', src:'addenda for half the largest path (notes p. 100; Khurmi 12.8)',
  q:'Two mating gears have 20 and 40 involute teeth of module 10 mm and 20° pressure angle. The addendum on each wheel is to be made of such a length that the line of contact on each side of the pitch point has half the maximum possible length. Find the addendum for each wheel, the length of the path of contact, arc of contact and contact ratio.',
  inputs:[['t', 'Pinion teeth', 20, ''], ['T', 'Wheel teeth', 40, ''], ['m', 'Module', 10, 'mm'], ['phi', 'Pressure angle', 20, '°']],
  solve:v => { const s = Math.sin(deg(v.phi)), c = Math.cos(deg(v.phi)), r = v.m*v.t/2, R = v.m*v.T/2, KP = r*s/2, PL = R*s/2;
    const RA = Math.sqrt((KP + R*s)**2 + (R*c)**2), rA = Math.sqrt((PL + r*s)**2 + (r*c)**2), path = KP + PL, arc = path/c;
    return [['r, R =', `${n2(r)}, ${n2(R)} mm`], ['KP = ½MP = ½ r sin φ, so R<sub>A</sub> = √((KP + R sin φ)² + R² cos² φ) =', `${n2(RA)} mm`], ['Addendum on the larger wheel = R<sub>A</sub> − R =', `${n2(RA - R)} mm`, true],
      ['PL = ½PN = ½ R sin φ, so r<sub>A</sub> = √((PL + r sin φ)² + r² cos² φ) =', `${n2(rA)} mm`], ['Addendum on the smaller wheel = r<sub>A</sub> − r =', `${n2(rA - r)} mm`, true], ['Path of contact = (r + R) sin φ/2 =', `${n2(path)} mm`, true],
      ['Arc of contact = path/cos φ =', `${n2(arc)} mm`, true], ['Contact ratio = arc/(πm) =', `${n2(arc/(Math.PI*v.m))} (so 2 pairs at times)`, true]]; } });
example($('ex5'), { title:'Example 5', src:'fewest pinion teeth (notes p. 103; Khurmi 12.9)',
  q:'Find the minimum number of teeth on a pinion, to avoid interference, which is to gear with (1) a wheel to give a gear ratio of 3 to 1, and (2) an equal wheel. Pressure angle 20°, standard addendum of 1 module for the wheel.',
  inputs:[['G', 'Gear ratio', 3, ''], ['phi', 'Pressure angle', 20, '°'], ['Aw', 'Wheel addendum', 1, '× module']],
  solve:v => { const s2 = Math.sin(deg(v.phi))**2, d = v.G*(Math.sqrt(1 + (1/v.G)*(1/v.G + 2)*s2) - 1), t1 = 2*v.Aw/d, d2 = Math.sqrt(1 + 3*s2) - 1, t2 = 2*v.Aw/d2;
    return [['sin² φ =', n2(s2, 4)], ['(1) t = 2A<sub>w</sub> / (G[√(1 + (1/G)(1/G + 2) sin² φ) − 1]) = 2 ×', `${v.Aw} / ${n2(d, 4)} = ${n2(t1)} → ${Math.ceil(t1 - 1e-9)} teeth`, true], ['(2) Equal wheel: t = 2A<sub>w</sub>/(√(1 + 3 sin² φ) − 1) =', `${n2(t2)} → ${Math.ceil(t2 - 1e-9)} teeth`, true]]; } });
example($('ex6'), { title:'Example 6', src:'addenda that just avoid interference (notes p. 104; Khurmi 12.11)',
  q:'A pair of involute spur gears with 16° pressure angle and module 6 mm is in mesh. The pinion has 16 teeth and runs at 240 rpm; the gear ratio is 1.75. Find, so that interference is just avoided, (1) the addenda on pinion and gear wheel, (2) the length of the path of contact, (3) the maximum velocity of sliding on either side of the pitch point.',
  inputs:[['phi', 'Pressure angle', 16, '°'], ['m', 'Module', 6, 'mm'], ['t', 'Pinion teeth', 16, ''], ['N', 'Pinion speed', 240, 'rpm'], ['G', 'Gear ratio', 1.75, '']],
  solve:v => { const s = Math.sin(deg(v.phi)), c = Math.cos(deg(v.phi)), s2 = s*s, T = v.G*v.t, r = v.m*v.t/2, R = v.m*T/2, w1 = 2*Math.PI*v.N/60, w2 = w1/v.G;
    const ap = (v.m*v.t/2)*(Math.sqrt(1 + (T/v.t)*(T/v.t + 2)*s2) - 1), aw = (v.m*T/2)*(Math.sqrt(1 + (v.t/T)*(v.t/T + 2)*s2) - 1), rA = r + ap, RA = R + aw;
    const KP = Math.sqrt(RA*RA - R*R*c*c) - R*s, PL = Math.sqrt(rA*rA - r*r*c*c) - r*s;
    return [['T = Gt =', `${n2(T, 0)} teeth; r = ${n2(r)} mm, R = ${n2(R)} mm`], ['(1) Pinion addendum = (mt/2)[√(1 + (T/t)(T/t + 2) sin² φ) − 1] =', `${n2(ap)} mm`, true], ['Wheel addendum = (mT/2)[√(1 + (t/T)(t/T + 2) sin² φ) − 1] =', `${n2(aw)} mm`, true],
      ['r<sub>A</sub> = r + pinion addendum, R<sub>A</sub> = R + wheel addendum =', `${n2(rA)}, ${n2(RA)} mm`], ['KP = √(R<sub>A</sub>² − R² cos² φ) − R sin φ =', `${n2(KP)} mm`], ['PL = √(r<sub>A</sub>² − r² cos² φ) − r sin φ =', `${n2(PL)} mm`],
      ['(2) Path of contact KL =', `${n2(KP + PL)} mm (= (r + R) sin φ = ${n2((r + R)*s)}, as it must be when interference is just avoided)`, true], ['ω<sub>1</sub>, ω<sub>2</sub> =', `${n2(w1, 3)}, ${n2(w2, 3)} rad/s`],
      ['(3) Sliding at K (approach side) = (ω<sub>1</sub> + ω<sub>2</sub>) KP =', `${n2((w1 + w2)*KP, 0)} mm/s`, true], ['Sliding at L (recess side) = (ω<sub>1</sub> + ω<sub>2</sub>) PL =', `${n2((w1 + w2)*PL, 0)} mm/s`, true]]; } });

/* ================= quiz ================= */
quiz($('qz'), [
  { q:'Two gears mesh correctly only if they have the same…', o:['number of teeth', 'module (circular pitch)', 'pitch circle diameter', 'face width'], a:1, why:'p<sub>c</sub> = πD/T must be equal on both, so D<sub>1</sub>/T<sub>1</sub> = D<sub>2</sub>/T<sub>2</sub>: the same module.' },
  { q:'A 20° full depth gear has module 4 mm. What is the dedendum?', o:['4 mm', '5 mm', '3.2 mm', '9 mm'], a:1, why:'Dedendum = 1.25m = 5 mm. Addendum is 1m = 4 mm and total depth 2.25m = 9 mm.' },
  { q:'The law of gearing says the common normal at the point of contact must always…', o:['be tangent to both pitch circles', 'pass through the pitch point', 'be parallel to the line of centres', 'pass through the centre of the larger gear'], a:1, why:'Then the speed ratio O<sub>2</sub>P/O<sub>1</sub>P stays constant.' },
  { q:'Why can involute gears run at a slightly wrong centre distance without changing the speed ratio?', o:['Because the pressure angle stays at 20°', 'Because the ratio depends only on the base circles, which do not change', 'Because the teeth have backlash', 'Because the pitch circles are fixed'], a:1, why:'The common tangent of the base circles still cuts the line of centres in the ratio r<sub>b1</sub>:r<sub>b2</sub>; the working pressure angle and pitch circles change instead.' },
  { q:'A contact ratio of 1.6 means…', o:['1.6 teeth touch at every instant', 'one pair always touches, and two pairs touch 60% of the time', 'the gears interfere', 'the pinion turns 1.6 times per wheel turn'], a:1, why:'It is the average number of pairs in contact: between 1 and 2.' },
  { q:'Where is the sliding velocity between the teeth zero?', o:['At the start of engagement K', 'At the pitch point P', 'At the end of engagement L', 'Nowhere'], a:1, why:'Sliding speed = (ω<sub>1</sub> + ω<sub>2</sub>) × distance from P, so it is zero at P and greatest at K or L.' },
  { q:'Interference is avoided when…', o:['the addendum circles cut the line of action between the interference points M and N', 'the contact ratio is less than 1', 'the dedendum equals the addendum', 'the pinion has more teeth than the wheel'], a:0, why:'Then every contact is on the involute part of both teeth.' },
  { q:'Which change does NOT help a 12-tooth pinion avoid undercut?', o:['Use 25° instead of 20° pressure angle', 'Use stub teeth', 'Use a larger module with the same 12 teeth', 'Use more teeth'], a:2, why:'Undercut depends on the number of teeth and the tooth proportions, not the size: scaling the module scales everything together.' },
  { q:'The base circle diameter of a 40-tooth, 5 mm module, 20° gear is about…', o:['200 mm', '187.9 mm', '187.5 mm', '212.5 mm'], a:1, why:'D cos φ = 200 × cos 20° = 187.9 mm. The root circle is 200 − 2 × 6.25 = 187.5 mm, close but a different circle.' }
]);
})();
