/* BME 2102 interactive notes, Chapter 3: velocity and acceleration analysis */
(() => {
'use strict';
const PI = Math.PI, TAU = 2*PI;
const lbl = (ctx, t, x, y, col, al = 'left', size = 12) => label(ctx, t, x, y, col, al, size, true);
const cr = (w, r) => [-w*r[1], w*r[0]];                       // ω k × r
const mid = (p, q, k = 0.5) => [p[0] + (q[0] - p[0])*k, p[1] + (q[1] - p[1])*k];
const rpm = N => N*TAU/60;
const sense = w => Math.abs(w) < 1e-9 ? '' : w > 0 ? 'anticlockwise' : 'clockwise';
function solve2(a, b, c, d, e, f){ const D = a*d - b*c; return [(e*d - b*f)/D, (a*f - c*e)/D]; }
function lineX(p1, d1, p2, d2){ const D = d1[0]*(-d2[1]) - d1[1]*(-d2[0]); if (Math.abs(D) < 1e-12) return null; const r = vsub(p2, p1); const t = (r[0]*(-d2[1]) - r[1]*(-d2[0]))/D; return vadd(p1, vmul(d1, t)); }

/* four-bar: frame pivots A (crank) and D (rocker); crank AB = a, coupler BC = b, rocker DC = c. w2, al2 of the crank (anticlockwise +). */
function fbKin(A, D, a, b, c, th, w2, al2 = 0, br = 1){ const B = vadd(A, [a*Math.cos(th), a*Math.sin(th)]), BD = vsub(D, B), L = vlen(BD);
  if (L > b + c || L < Math.abs(b - c)) return null; const k = (b*b - c*c + L*L)/(2*L), h = Math.sqrt(Math.max(0, b*b - k*k)), u = vmul(BD, 1/L), Cc = vadd(vadd(B, vmul(u, k)), vmul(vperp(u), br*h));
  const rAB = vsub(B, A), rBC = vsub(Cc, B), rDC = vsub(Cc, D), vB = cr(w2, rAB);
  const [w3, w4] = solve2(-rBC[1], rDC[1], rBC[0], -rDC[0], -vB[0], -vB[1]);
  const aB = vadd(cr(al2, rAB), vmul(rAB, -w2*w2)), rhs = vsub(vmul(rDC, -w4*w4), vadd(aB, vmul(rBC, -w3*w3)));
  const [a3, a4] = solve2(-rBC[1], rDC[1], rBC[0], -rDC[0], rhs[0], rhs[1]);
  return { A, D, B, C:Cc, rAB, rBC, rDC, vB, w3, w4, vC:cr(w4, rDC), vCB:cr(w3, rBC), aB, a3, a4, aC:vadd(cr(a4, rDC), vmul(rDC, -w4*w4)), arCB:vmul(rBC, -w3*w3), atCB:cr(a3, rBC), arCD:vmul(rDC, -w4*w4), atCD:cr(a4, rDC) }; }
/* slider-crank: crank centre O at the origin, slider on the x axis to the left. th from +x; w2 anticlockwise + */
function scKin(r, l, th, w2){ const B = [r*Math.cos(th), r*Math.sin(th)]; if (l <= Math.abs(B[1])) return null; const P = [B[0] - Math.sqrt(l*l - B[1]*B[1]), 0], rBP = vsub(P, B), vB = cr(w2, B);
  const w3 = -vB[1]/rBP[0], vP = vB[0] - w3*rBP[1], aB = vmul(B, -w2*w2), t = vadd(aB, vmul(rBP, -w3*w3)), a3 = -t[1]/rBP[0], aP = t[0] - a3*rBP[1];
  return { O:[0, 0], B, P, rBP, vB, w3, vP, aB, a3, aP, arPB:vmul(rBP, -w3*w3), atPB:cr(a3, rBP) }; }

/* draw a vector diagram (velocity or acceleration) with equal scales. pts: name -> vector; segs: [from, to, colour, label, style] */
function vecDiagram(cv, pts, segs, o = {}){ const xs = Object.values(pts).map(p => p[0]), ys = Object.values(pts).map(p => p[1]);
  let xl = Math.min(...xs), xr = Math.max(...xs), yb = Math.min(...ys), yt = Math.max(...ys); const span = Math.max(xr - xl, yt - yb, 1e-9), pad = span*0.18;
  const V = fitView(cv, { xl:xl - pad, xr:xr + pad, yb:yb - pad, yt:yt + pad }, 16), { ctx } = V, P = V.P;
  for (const [f, t, col, txt, st] of segs){ const a = P(pts[f]), b = P(pts[t]); if (!a || !b) continue;
    if (st === 'dash'){ dashLine(ctx, ...a, ...b, col, [5, 4], 1.4); } else if (st === 'line'){ ctx.strokeStyle = col; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke(); } else arrow(ctx, ...a, ...b, col, st === 'thin' ? 1.6 : 2.4);
    if (txt){ const m = mid(a, b), d = vsub(b, a), n = vperp(d), nl = vlen(n) || 1; lbl(ctx, txt, m[0] + n[0]/nl*12, m[1] + n[1]/nl*12, col, 'center', 11.5); } }
  const c = Object.values(pts).reduce((s, p) => vadd(s, p), [0, 0]).map(v => v/Object.keys(pts).length), cc = P(c);
  for (const [nm, p] of Object.entries(pts)){ if (o.hide && o.hide.includes(nm)) continue; const q = P(p), d = vsub(q, cc), dl = vlen(d) || 1; dotAt(ctx, ...q, C.ink, 2.6);
    lbl(ctx, (o.names && o.names[nm]) || nm, q[0] + d[0]/dl*13, q[1] + d[1]/dl*13, C.ink, 'center', 12.5); }
  if (o.title) lbl(ctx, o.title, 8, 12, C.muted, 'left', 11); return V; }

/* ================= 3.1 any move is a rotation ================= */
{ const P = {}, cv = $('mv'); let k = 0;
  controls($('mv_c'), [['dx', 'Move along x', -150, 150, 5, 70, 'mm'], ['dy', 'Move along y', -100, 100, 5, 40, 'mm'], ['ph', 'Turn', -90, 90, 1, 35, '°']], P, () => draw());
  function draw(){ const A = [-70, -40], B = [70, -40], M = mid(A, B), f = P.ph*DEG, R = q => { const d = vsub(q, M); return vadd(M, [d[0]*Math.cos(f) - d[1]*Math.sin(f), d[0]*Math.sin(f) + d[1]*Math.cos(f)]); };
    const A1 = vadd(R(A), [P.dx, P.dy]), B1 = vadd(R(B), [P.dx, P.dy]); const mA = mid(A, A1), mB = mid(B, B1), nA = vperp(vsub(A1, A)), nB = vperp(vsub(B1, B)), I = Math.abs(P.ph) > 0.2 ? lineX(mA, nA, mB, nB) : null;
    const V = fitView(cv, { xl:-260, xr:260, yb:-170, yt:170 }), { ctx } = V, Pp = V.P;
    barAt(ctx, ...Pp(A), ...Pp(B), alpha(C.muted, 0.45), 7); ctx.setLineDash([6, 5]); barAt(ctx, ...Pp(A1), ...Pp(B1), alpha(C.crank, 0.55), 3); ctx.setLineDash([]);
    lbl(ctx, 'A', Pp(A)[0] - 10, Pp(A)[1] + 12, C.muted, 'right'); lbl(ctx, 'B', Pp(B)[0] + 10, Pp(B)[1] + 12, C.muted); lbl(ctx, 'A₁', Pp(A1)[0] - 10, Pp(A1)[1] - 12, C.crank, 'right'); lbl(ctx, 'B₁', Pp(B1)[0] + 10, Pp(B1)[1] - 12, C.crank);
    dashLine(ctx, ...Pp(A), ...Pp(A1), C.muted, [2, 3]); dashLine(ctx, ...Pp(B), ...Pp(B1), C.muted, [2, 3]);
    let cur;
    if (I){ const big = 900; for (const [m, n] of [[mA, nA], [mB, nB]]){ const u = vmul(n, 1/vlen(n)); dashLine(ctx, ...Pp(vadd(m, vmul(u, -big))), ...Pp(vadd(m, vmul(u, big))), C.force, [6, 4], 1.2); }
      const ang = f*k, Rot = q => { const d = vsub(q, I); return vadd(I, [d[0]*Math.cos(ang) - d[1]*Math.sin(ang), d[0]*Math.sin(ang) + d[1]*Math.cos(ang)]); }; cur = [Rot(A), Rot(B)];
      for (const q of [A, B]){ ctx.strokeStyle = alpha(C.acc, 0.6); ctx.lineWidth = 1.4; ctx.beginPath(); for (let i = 0; i <= 40; i++){ const a = f*i/40, d = vsub(q, I), p = vadd(I, [d[0]*Math.cos(a) - d[1]*Math.sin(a), d[0]*Math.sin(a) + d[1]*Math.cos(a)]); i ? ctx.lineTo(...Pp(p)) : ctx.moveTo(...Pp(p)); } ctx.stroke(); }
      const ip = Pp(I); if (ip[0] > 0 && ip[0] < V.W && ip[1] > 0 && ip[1] < V.H){ dotAt(ctx, ...ip, C.acc, 6); lbl(ctx, 'I', ip[0] + 10, ip[1] - 10, C.acc, 'left', 14); } else lbl(ctx, 'I lies off the drawing', 8, 14, C.acc);
      $('mv_sub').textContent = `the move AB → A₁B₁ is a turn of ${P.ph}° about I`; }
    else { cur = [vadd(A, vmul([P.dx, P.dy], k)), vadd(B, vmul([P.dx, P.dy], k))]; $('mv_sub').textContent = 'no turn: a pure translation, with I at infinity'; }
    barAt(ctx, ...Pp(cur[0]), ...Pp(cur[1]), C.crank, 7); pinAt(ctx, ...Pp(cur[0])); pinAt(ctx, ...Pp(cur[1])); }
  watchSize(cv, draw); onRedraw(draw); const an = animate(cv, dt => { k = (k + dt*0.5) % 1.3; if (k > 1) { draw(); return; } draw(); }); playButton($('mv_play'), an); k = 1; draw(); }

/* rules for locating the centre */
{ const cv = $('rl'); let mode = 'pin', t = 0;
  const CAP = { pin:'<b>Pin joint.</b> The centre is the pin. Every point of the link moves perpendicular to its line to the pin, at a speed in proportion to its distance from it.',
    roll:'<b>Pure rolling.</b> The centre I<sub>12</sub> is the contact point, which is momentarily at rest. The top of the wheel moves at twice the speed of the centre: v<sub>A</sub>/v<sub>B</sub> = I<sub>12</sub>A/I<sub>12</sub>B.',
    slide:'<b>Sliding on a straight guide.</b> The normals to the guide are parallel, so the centre is at infinity: every point of the slider has the same velocity.',
    curve:'<b>Sliding on a curved guide.</b> The normals meet at the centre of curvature, so that is the centre. For a guide of constant radius it is the same point in every position.' };
  function draw(){ const V = fitView(cv, { xl:-260, xr:260, yb:-130, yt:170 }), { ctx } = V, P = V.P;
    const vArrow = (p, v, k = 1) => arrow(ctx, ...P(p), ...P(vadd(p, vmul(v, k))), C.vel, 2.2);
    if (mode === 'pin'){ const O = [-60, 0], a = 0.6*Math.sin(t*0.8), L = 200, E = vadd(O, [L*Math.cos(a), L*Math.sin(a)]); barAt(ctx, ...P(O), ...P(E), C.crank, 8); pivotAt(ctx, ...P(O));
      for (const s of [0.35, 0.7, 1]){ const q = vadd(O, [L*s*Math.cos(a), L*s*Math.sin(a)]); vArrow(q, vmul([-Math.sin(a), Math.cos(a)], 110*s)); dotAt(ctx, ...P(q), C.ink, 3.5); }
      lbl(ctx, 'I (the pin)', P(O)[0] - 12, P(O)[1] - 14, C.acc, 'right'); }
    if (mode === 'roll'){ const R = 70, x = ((t*60) % 520) - 260, ang = -(t*60)/R, Cc = [x, R - 100]; groundAt(ctx, ...P([0, -100]), 520*V.sc, 0);
      ctx.strokeStyle = C.crank; ctx.lineWidth = 4; ctx.beginPath(); ctx.arc(...P(Cc), R*V.sc, 0, 7); ctx.stroke(); const I = [x, -100];
      for (let i = 0; i < 6; i++){ const a = ang + i*PI/3, q = vadd(Cc, [R*Math.cos(a), R*Math.sin(a)]); ctx.strokeStyle = alpha(C.crank, 0.5); ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(...P(Cc)); ctx.lineTo(...P(q)); ctx.stroke(); }
      for (const q of [vadd(Cc, [0, R]), Cc, vadd(Cc, [R*0.87, R*0.5]), vadd(Cc, [-R, 0])]){ const r = vsub(q, I); vArrow(q, vmul(vperp(r), -0.75)); dotAt(ctx, ...P(q), C.ink, 3.5); dashLine(ctx, ...P(I), ...P(q), alpha(C.acc, 0.6), [3, 3], 1); }
      dotAt(ctx, ...P(I), C.acc, 6); lbl(ctx, 'I₁₂', P(I)[0] + 8, P(I)[1] + 14, C.acc); }
    if (mode === 'slide'){ groundAt(ctx, ...P([0, -40]), 500*V.sc, 0); const x = 120*Math.sin(t*0.9), v = 120*0.9*Math.cos(t*0.9);
      ctx.fillStyle = alpha(C.crank, 0.85); ctx.fillRect(...P([x - 60, 40]), 120*V.sc, 80*V.sc);
      for (const q of [[x - 40, 20], [x + 40, 20], [x, 0], [x - 40, -20 + 0]]){ vArrow([q[0], q[1]], [v*0.7, 0]); dotAt(ctx, ...P(q), C.ink, 3.5); }
      for (const xx of [x - 40, x + 40]) dashLine(ctx, ...P([xx, -40]), ...P([xx, 160]), alpha(C.acc, 0.6), [3, 3], 1);
      lbl(ctx, 'normals are parallel: I is at infinity', 10, 16, C.acc); }
    if (mode === 'curve'){ const O = [0, -110], R = 230, a = PI/2 + 0.55*Math.sin(t*0.8), w = 0.55*0.8*Math.cos(t*0.8);
      ctx.strokeStyle = C.muted; ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(...P(O), R*V.sc, -PI*0.85, -PI*0.15); ctx.stroke();
      const q = vadd(O, [R*Math.cos(a), R*Math.sin(a)]); ctx.save(); ctx.translate(...P(q)); ctx.rotate(-(a - PI/2)); ctx.fillStyle = alpha(C.crank, 0.85); ctx.fillRect(-35, -22, 70, 22); ctx.restore();
      for (const da of [-0.12, 0, 0.12]){ const p = vadd(O, [(R + 10)*Math.cos(a + da), (R + 10)*Math.sin(a + da)]); dashLine(ctx, ...P(O), ...P(p), alpha(C.acc, 0.6), [3, 3], 1); vArrow(p, vmul([-Math.sin(a + da), Math.cos(a + da)], w*R*0.8)); }
      dotAt(ctx, ...P(O), C.acc, 6); lbl(ctx, 'I (centre of curvature)', P(O)[0] + 10, P(O)[1] - 10, C.acc); }
    $('rl_sub').textContent = 'green: velocities'; }
  segGroup($('rl_seg'), m => { mode = m; $('rl_cap').innerHTML = CAP[m]; draw(); }); $('rl_cap').innerHTML = CAP.pin;
  watchSize(cv, draw); onRedraw(draw); const an = animate(cv, dt => { t += dt; draw(); }); playButton($('rl_play'), an); draw(); }

/* ================= 3.1.2 instantaneous centres of a four-bar ================= */
{ const P = {}, cv = $('ic'), kv = $('ic_k'); let step = 4, cent = null;
  const L = { AD:0.6, AB:0.3, BC:0.36, CD:0.36 };
  controls($('ic_c'), [['th', 'Crank angle BAD', 0, 359, 1, 60, '°'], ['N', 'Crank speed', 10, 300, 5, 100, 'rpm']], P, () => draw());
  const kin = th => fbKin([0, 0], [L.AD, 0], L.AB, L.BC, L.CD, th*DEG, -rpm(P.N));
  function centres(k){ const I13 = lineX(k.A, vsub(k.B, k.A), k.D, vsub(k.C, k.D)), I24 = lineX(k.B, vsub(k.C, k.B), k.A, [1, 0]); return { I13, I24 }; }
  function centrodes(){ const sp = [], bd = []; for (let i = 0; i < 360; i += 2){ const k = kin(i); if (!k) continue; const { I13 } = centres(k); if (!I13 || vlen(I13) > 5) { sp.push(null); bd.push(null); continue; }
      sp.push(I13); const u = vmul(vsub(k.C, k.B), 1/L.BC), d = vsub(I13, k.B); bd.push([d[0]*u[0] + d[1]*u[1], -d[0]*u[1] + d[1]*u[0]]); } return { sp, bd }; }
  function draw(){ const k = kin(P.th); if (!k) return; const { I13, I24 } = centres(k), Lc = { 1:C.muted, 2:C.crank, 3:C.rod, 4:C.vel };
    const V = fitView(cv, { xl:-0.35, xr:1.05, yb:-0.25, yt:1.0 }), { ctx } = V, Pp = V.P, inView = q => q && q[0] > -0.4 && q[0] < 1.1 && q[1] > -0.3 && q[1] < 1.05;
    if ($('ic_cent').checked){ if (!cent) cent = centrodes(); const path = (arr, map, col) => { ctx.strokeStyle = col; ctx.lineWidth = 1.6; ctx.beginPath(); let on = false; for (const q of arr){ if (!q){ on = false; continue; } const m = Pp(map(q)); on ? ctx.lineTo(...m) : ctx.moveTo(...m); on = true; } ctx.stroke(); };
      const u = vmul(vsub(k.C, k.B), 1/L.BC); path(cent.sp, q => q, alpha(C.acc, 0.7)); path(cent.bd, q => vadd(k.B, [q[0]*u[0] - q[1]*u[1], q[0]*u[1] + q[1]*u[0]]), alpha(C.force, 0.7)); }
    groundAt(ctx, ...Pp([L.AD/2, 0]), L.AD*V.sc + 60, 0);
    if (step >= 3 && I13){ const far = inView(I13) ? I13 : vadd(k.A, vmul(vsub(I13, k.A), 2/vlen(vsub(I13, k.A)))); dashLine(ctx, ...Pp(k.A), ...Pp(far), C.acc, [6, 4]); dashLine(ctx, ...Pp(k.D), ...Pp(inView(I13) ? I13 : vadd(k.D, vmul(vsub(I13, k.D), 2/vlen(vsub(I13, k.D))))), C.acc, [6, 4]); }
    if (step >= 4 && I24){ dashLine(ctx, ...Pp(k.B), ...Pp(inView(I24) ? I24 : vadd(k.B, vmul(vsub(I24, k.B), 2/vlen(vsub(I24, k.B))))), C.force, [6, 4]); dashLine(ctx, ...Pp([-0.4, 0]), ...Pp([1.1, 0]), C.force, [6, 4]); }
    barAt(ctx, ...Pp(k.A), ...Pp(k.B), Lc[2], 7); barAt(ctx, ...Pp(k.B), ...Pp(k.C), Lc[3], 7); barAt(ctx, ...Pp(k.D), ...Pp(k.C), Lc[4], 7);
    pivotAt(ctx, ...Pp(k.A)); pivotAt(ctx, ...Pp(k.D)); pinAt(ctx, ...Pp(k.B)); pinAt(ctx, ...Pp(k.C));
    for (const [q, n] of [[k.A, 'A'], [k.B, 'B'], [k.C, 'C'], [k.D, 'D']]) lbl(ctx, n, Pp(q)[0] - 10, Pp(q)[1] + 14, C.ink, 'right');
    if (step >= 2) for (const [q, n] of [[k.A, 'I₁₂'], [k.B, 'I₂₃'], [k.C, 'I₃₄'], [k.D, 'I₁₄']]) lbl(ctx, n, Pp(q)[0] + 9, Pp(q)[1] - 13, C.ink, 'left', 12);
    for (const [q, n, col, s] of [[I13, 'I₁₃', C.acc, 3], [I24, 'I₂₄', C.force, 4]]){ if (step < s || !q) continue; if (inView(q)){ dotAt(ctx, ...Pp(q), col, 6); lbl(ctx, n, Pp(q)[0] + 9, Pp(q)[1] - 12, col, 'left', 13); } else lbl(ctx, `${n} is off the drawing`, 8, s === 3 ? 14 : 32, col); }
    // velocities by I13
    const vB = rpm(P.N)*L.AB, I13B = I13 ? vlen(vsub(k.B, I13)) : Infinity, wBC = vB/I13B, I13C = I13 ? vlen(vsub(k.C, I13)) : Infinity, vC = wBC*I13C;
    if (step >= 3){ const sB = 0.12/Math.max(vB, 1e-6); arrow(ctx, ...Pp(k.B), ...Pp(vadd(k.B, vmul(k.vB, sB))), C.crank, 2.4); arrow(ctx, ...Pp(k.C), ...Pp(vadd(k.C, vmul(k.vC, sB))), C.vel, 2.4); }
    // Kennedy circle diagram
    { const W = fitView(kv, { xl:-1.5, xr:1.5, yb:-1.5, yt:1.5 }, 22), c2 = W.ctx, Q = { 1:[-1, -1], 2:[-1, 1], 3:[1, 1], 4:[1, -1] }.valueOf(); const ang = { 1:-135, 2:135, 3:45, 4:-45 }, pos = n => W.P([Math.cos(ang[n]*DEG), Math.sin(ang[n]*DEG)]);
      c2.strokeStyle = alpha(C.muted, 0.6); c2.lineWidth = 1.2; c2.beginPath(); c2.arc(...W.P([0, 0]), W.sc, 0, 7); c2.stroke();
      const seg = (a, b, col, dash) => { c2.strokeStyle = col; c2.lineWidth = 2.4; c2.setLineDash(dash ? [6, 4] : []); c2.beginPath(); c2.moveTo(...pos(a)); c2.lineTo(...pos(b)); c2.stroke(); c2.setLineDash([]); };
      if (step >= 2) [[1, 2], [2, 3], [3, 4], [4, 1]].forEach(([a, b]) => seg(a, b, C.ink, false));
      if (step >= 3) seg(1, 3, C.acc, step === 3); if (step >= 4) seg(2, 4, C.force, true);
      for (const n of [1, 2, 3, 4]){ const q = pos(n); c2.fillStyle = C.sheet; c2.strokeStyle = C.ink; c2.lineWidth = 1.6; c2.beginPath(); c2.arc(...q, 13, 0, 7); c2.fill(); c2.stroke(); c2.fillStyle = C.ink; c2.font = `600 13px ${C.mono}`; c2.textAlign = 'center'; c2.textBaseline = 'middle'; c2.fillText(String(n), q[0], q[1] + 0.5); }
      const msg = { 1:'Four links: N = 4 × 3/2 = 6 centres: 12, 13, 14, 23, 24, 34.', 2:'Mark 1, 2, 3, 4 on a circle. Join the pairs found by inspection: 12, 23, 34, 14.', 3:'Line 13 closes triangles 123 and 134: I₁₃ is where I₁₂I₂₃ meets I₁₄I₃₄.', 4:'Line 24 closes triangles 234 and 124: I₂₄ is where I₂₃I₃₄ meets I₁₂I₁₄.' }[step];
      lbl(c2, 'Kennedy circle diagram', 8, 12, C.muted, 'left', 11); c2.font = `12px ${C.mono}`; c2.fillStyle = C.text; c2.textAlign = 'left'; c2.textBaseline = 'top';
      const words = msg.split(' '); let line = '', y = W.H - 46; for (const w of words){ if (c2.measureText(line + w).width > W.W - 20){ c2.fillText(line, 10, y); line = ''; y += 16; } line += w + ' '; } c2.fillText(line, 10, y); }
    $('ic_st').innerHTML = statsHTML([['ω of AB', `${n2(rpm(P.N), 2)} <small>rad/s</small>`], ['v_B = ω AB', `${n2(vB, 3)} <small>m/s</small>`], ['I₁₃B', `${n2(I13B, 3)} <small>m</small>`], ['ω_BC = v_B/I₁₃B', `${n2(wBC, 3)} <small>rad/s</small>`], ['v_C = ω_BC × I₁₃C', `${n2(vC, 3)} <small>m/s</small>`], ['ω_CD = v_C/CD', `${n2(vC/L.CD, 3)} <small>rad/s</small>`]]);
    $('ic_sub').textContent = 'crank AB turns clockwise'; }
  segGroup($('ic_seg'), v => { step = +v; draw(); }); $('ic_cent').addEventListener('change', () => { cent = null; draw(); });
  [cv, kv].forEach(c => watchSize(c, draw)); onRedraw(draw); const S = { th:P.th }; const an = animate(cv, dt => { P.th = (P.th - dt*30 + 360) % 360; draw(); }); an.playing = false; playButton($('ic_play'), an); draw(); }

example($('ex1'), { title:'Four-bar: angular velocity of the coupler', src:'notes, instantaneous centre example (Khurmi Example 6.1)',
  q:'In a pin-jointed four-bar ABCD, AB = 300 mm, BC = CD = 360 mm and AD = 600 mm (fixed). Angle BAD = 60° and the crank AB turns uniformly at 100 rpm. Locate the instantaneous centres and find the angular velocity of BC.',
  inputs:[['AB', 'AB', 300, 'mm'], ['BC', 'BC', 360, 'mm'], ['CD', 'CD', 360, 'mm'], ['AD', 'AD', 600, 'mm'], ['th', 'Angle BAD', 60, '°'], ['N', 'Speed of AB', 100, 'rpm']],
  solve:v => { const k = fbKin([0, 0], [v.AD/1000, 0], v.AB/1000, v.BC/1000, v.CD/1000, v.th*DEG, -rpm(v.N)); const I13 = lineX(k.A, vsub(k.B, k.A), k.D, vsub(k.C, k.D)), w = rpm(v.N), vB = w*v.AB/1000, d = vlen(vsub(k.B, I13));
    return [['ω_AB = 2πN/60 =', `${n2(w, 2)} rad/s`], ['v_B = ω_AB × AB =', `${n2(vB, 3)} m/s`], ['N = n(n − 1)/2 = 4 × 3/2 =', '6 centres'], ['I₁₃ is where AB produced meets DC produced (Kennedy). Calculated I₁₃B =', `${n2(d*1000, 1)} mm`],
      ['ω_BC = v_B / I₁₃B =', `${n2(vB/d, 3)} rad/s`, true]]; } });

/* ================= 3.2 four-bar velocity diagram, Example 7.1 ================= */
{ const P = {}, cv = $('fv'); const L = { AD:0.15, AB:0.04, BC:0.15, CD:0.08 };
  controls($('fv_c'), [['th', 'Angle BAD', 0, 359, 1, 60, '°'], ['N', 'Crank speed (clockwise)', 10, 300, 5, 120, 'rpm']], P, () => draw());
  function draw(){ const k = fbKin([0, 0], [L.AD, 0], L.AB, L.BC, L.CD, P.th*DEG, -rpm(P.N)); if (!k) return;
    const V = fitView(cv, { xl:-0.06, xr:0.22, yb:-0.05, yt:0.13 }), { ctx } = V, Pp = V.P; groundAt(ctx, ...Pp([L.AD/2, 0]), L.AD*V.sc + 40, 0);
    barAt(ctx, ...Pp(k.A), ...Pp(k.B), C.crank, 7); barAt(ctx, ...Pp(k.B), ...Pp(k.C), C.rod, 7); barAt(ctx, ...Pp(k.D), ...Pp(k.C), C.vel, 7);
    pivotAt(ctx, ...Pp(k.A)); pivotAt(ctx, ...Pp(k.D)); pinAt(ctx, ...Pp(k.B)); pinAt(ctx, ...Pp(k.C));
    const s = 0.05/Math.max(vlen(k.vB), 1e-9); arrow(ctx, ...Pp(k.B), ...Pp(vadd(k.B, vmul(k.vB, s))), C.crank, 2.2); arrow(ctx, ...Pp(k.C), ...Pp(vadd(k.C, vmul(k.vC, s))), C.vel, 2.2);
    for (const [q, n] of [[k.A, 'A'], [k.B, 'B'], [k.C, 'C'], [k.D, 'D']]) lbl(ctx, n, Pp(q)[0] - 10, Pp(q)[1] - 12, C.ink, 'right');
    lbl(ctx, 'v_B', ...vadd(Pp(vadd(k.B, vmul(k.vB, s))), [8, 0]), C.crank); lbl(ctx, 'v_C', ...vadd(Pp(vadd(k.C, vmul(k.vC, s))), [8, 0]), C.vel);
    vecDiagram($('fv_d'), { 'a, d':[0, 0], b:k.vB, c:k.vC }, [['a, d', 'b', C.crank, 'v_B ⟂ AB'], ['b', 'c', C.rod, 'v_CB ⟂ BC'], ['a, d', 'c', C.vel, 'v_C ⟂ DC']], { title:'velocity diagram (m/s)' });
    $('fv_st').innerHTML = statsHTML([['v_B = ω AB', `${n2(vlen(k.vB), 3)} <small>m/s</small>`], ['v_CB = bc', `${n2(vlen(k.vCB), 3)} <small>m/s</small>`], ['v_C = dc', `${n2(vlen(k.vC), 3)} <small>m/s</small>`], ['ω_BC = bc/BC', `${n2(Math.abs(k.w3), 2)} <small>rad/s</small> ${sense(k.w3)}`], ['ω_CD = dc/CD', `${n2(Math.abs(k.w4), 2)} <small>rad/s</small> ${sense(k.w4)}`]]);
    $('fv_sub').textContent = 'AD = 150, AB = 40, BC = 150, CD = 80 mm'; }
  [cv, $('fv_d')].forEach(c => watchSize(c, draw)); onRedraw(draw); const an = animate(cv, dt => { P.th = (P.th - dt*40 + 360) % 360; draw(); }); an.playing = false; playButton($('fv_play'), an); draw(); }

example($('ex2'), { title:'Example 7.1: four-bar ABCD', src:'notes, Example 7.1 (Khurmi)',
  q:'In a four-bar chain ABCD, AD is fixed and 150 mm long. The crank AB is 40 mm long and turns at 120 rpm clockwise, while CD = 80 mm oscillates about D. BC and AD are equal. Find the angular velocity of CD when angle BAD = 60°.',
  inputs:[['AD', 'AD', 150, 'mm'], ['AB', 'AB', 40, 'mm'], ['BC', 'BC', 150, 'mm'], ['CD', 'CD', 80, 'mm'], ['th', 'Angle BAD', 60, '°'], ['N', 'Speed of AB', 120, 'rpm']],
  solve:v => { const k = fbKin([0, 0], [v.AD/1000, 0], v.AB/1000, v.BC/1000, v.CD/1000, v.th*DEG, -rpm(v.N)); const w = rpm(v.N);
    return [['ω_BA = 2πN/60 =', `${n2(w, 3)} rad/s`], ['v_B = ω_BA × AB (ab ⟂ AB) =', `${n2(w*v.AB/1000, 3)} m/s`], ['bc ⟂ CB and dc ⟂ CD meet at c. Calculated: v_CB = bc =', `${n2(vlen(k.vCB), 3)} m/s`],
      ['v_C = dc =', `${n2(vlen(k.vC), 3)} m/s`], ['ω_CD = v_C/CD =', `${n2(Math.abs(k.w4), 2)} rad/s, ${sense(k.w4)} about D`, true]]; } });

/* ================= 3.2.2 steam engine (3D) and its velocity diagram ================= */
const ENG = { r:0.5, l:2, N:180, PE:1.5, dO:50, dB:60, dC:30 };
function engine(phi){ const th = PI - phi, k = scKin(ENG.r, ENG.l, th, -rpm(ENG.N)), w2 = -rpm(ENG.N);
  const E = vadd(k.P, vmul(vsub(k.B, k.P), ENG.PE/ENG.l)), vE = vadd([k.vP, 0], cr(k.w3, vsub(E, k.P)));
  let best = [1e9, 0]; for (let i = 0; i <= 400; i++){ const s = i/400, q = vadd(k.P, vmul(vsub(k.B, k.P), s)), v = vlen(vadd([k.vP, 0], cr(k.w3, vsub(q, k.P)))); if (v < best[0]) best = [v, s]; }
  const G = vadd(k.P, vmul(vsub(k.B, k.P), best[1])), vG = vadd([k.vP, 0], cr(k.w3, vsub(G, k.P)));
  return { ...k, w2, E, vE, G, vG, BG:(1 - best[1])*ENG.l,
    rub:{ O:ENG.dO/2000*Math.abs(w2), B:ENG.dB/2000*Math.abs(w2 - k.w3), C:ENG.dC/2000*Math.abs(k.w3) } }; }
{ let phi = 45*DEG; const parts = {}; const S = 0.5;
  const v = view3d($('se3'), { cam:[-1.2, 1.6, 4.9], target:[-1.6, -0.1, 0], fov:38,
    build(v){ const T = THREE; readColors(); const steel = mat('#9aa6b2', { metalness:0.7, roughness:0.3 }), body = mat('#7f8c99', { metalness:0.2, roughness:0.7 });
      const glass = new T.MeshStandardMaterial({ color:new T.Color(C.muted), transparent:true, opacity:0.18, side:T.DoubleSide, depthWrite:false });
      const disc = new T.Mesh(new T.CylinderGeometry(0.62*S*2, 0.62*S*2, 0.08, 48), mat(C.crank, { metalness:0.4 })); disc.rotation.x = PI/2; disc.position.z = -0.12; const crank = new T.Group(); crank.add(disc);
      const pin = new T.Mesh(new T.CylinderGeometry(0.04, 0.04, 0.25, 16), steel); pin.rotation.x = PI/2; pin.position.set(ENG.r, 0, 0); crank.add(pin); v.root.add(crank);
      const shaft = new T.Mesh(new T.CylinderGeometry(0.05, 0.05, 0.6, 16), steel); shaft.rotation.x = PI/2; shaft.position.z = -0.2; v.root.add(shaft);
      const rod = new T.Mesh(new T.BoxGeometry(1, 0.07, 0.05), mat(C.rod)); v.root.add(rod);
      const xh = new T.Mesh(new T.BoxGeometry(0.22, 0.16, 0.18), mat(C.vel)); v.root.add(xh);
      for (const y of [0.11, -0.11]){ const g = new T.Mesh(new T.BoxGeometry(1.2, 0.03, 0.26), body); g.position.set(-2.0, y, 0); v.root.add(g); }
      const prod = new T.Mesh(new T.CylinderGeometry(0.03, 0.03, 1, 12), steel); prod.rotation.z = PI/2; v.root.add(prod);
      const cyl = new T.Mesh(new T.CylinderGeometry(0.28, 0.28, 1.3, 40, 1, true), glass); cyl.rotation.z = PI/2; cyl.position.x = -3.2; v.root.add(cyl);
      const pist = new T.Mesh(new T.CylinderGeometry(0.26, 0.26, 0.14, 40), mat(C.vel)); pist.rotation.z = PI/2; v.root.add(pist);
      const bed = new T.Mesh(new T.BoxGeometry(5.2, 0.05, 0.9), body); bed.position.set(-1.8, -0.75, 0); v.root.add(bed);
      const arr = col => { const a = new T.ArrowHelper(new T.Vector3(1, 0, 0), new T.Vector3(), 0.5, new T.Color(col).getHex(), 0.1, 0.06); a.position.z = 0.15; v.root.add(a); return a; };
      Object.assign(parts, { crank, rod, xh, prod, pist, aB:arr(C.crank), aP:arr(C.vel), aE:arr(C.rod), aG:arr(C.force) }); },
    update(dt){ phi += dt*rpm(ENG.N)*($('se_slow').checked ? 0.02 : 0.08); place(); } });
  function place(){ const e = engine(phi); if (parts.crank){ parts.crank.rotation.z = PI - phi; const m = mid(e.B, e.P), d = vsub(e.B, e.P);
      parts.rod.position.set(m[0], m[1], 0.06); parts.rod.scale.x = vlen(d); parts.rod.rotation.z = Math.atan2(d[1], d[0]); parts.xh.position.set(e.P[0], 0, 0.06);
      parts.prod.position.set(e.P[0] - 0.6, 0, 0); parts.prod.scale.y = 1.2; parts.pist.position.set(e.P[0] - 1.2, 0, 0);
      const sc = 0.07; for (const [a, p, vv] of [[parts.aB, e.B, e.vB], [parts.aP, e.P, [e.vP, 0]], [parts.aE, e.E, e.vE], [parts.aG, e.G, e.vG]]){ const L = vlen(vv); a.position.set(p[0], p[1], 0.15); a.setDirection(new THREE.Vector3(vv[0]/L, vv[1]/L, 0)); a.setLength(L*sc, 0.1, 0.06); } }
    vecDiagram($('se_d'), { o:[0, 0], b:e.vB, p:[e.vP, 0], e:e.vE, g:e.vG }, [['o', 'b', C.crank, 'v_B'], ['b', 'p', C.rod, 'v_PB'], ['o', 'p', C.vel, 'v_P'], ['o', 'e', C.rod, '', 'thin'], ['o', 'g', C.force, '', 'thin']], { title:'velocity diagram (m/s)' });
    const ang = ((phi/DEG) % 360 + 360) % 360;
    $('se_st').innerHTML = statsHTML([['Crank from IDC', `${n2(ang, 0)}°`], ['v_B', `${n2(vlen(e.vB), 3)} <small>m/s</small>`], ['Piston v_P', `${n2(Math.abs(e.vP), 3)} <small>m/s</small>`], ['ω_PB', `${n2(Math.abs(e.w3), 2)} <small>rad/s</small>`], ['v_E (PE = 1.5 m)', `${n2(vlen(e.vE), 3)} <small>m/s</small>`],
      ['Slowest point G: BG', `${n2(e.BG, 2)} <small>m</small>`], ['v_G', `${n2(vlen(e.vG), 3)} <small>m/s</small>`], ['Rubbing at O, B, P', `${n2(e.rub.O, 3)}, ${n2(e.rub.B, 3)}, ${n2(e.rub.C, 3)} <small>m/s</small>`]]);
    $('se_sub').textContent = 'crank 0.5 m, rod 2 m, 180 rpm clockwise (shown slowed down)'; }
  watchSize($('se_d'), place); onRedraw(place); if (v) playButton($('se_play'), v.anim); else { animate($('se_d'), dt => { phi += dt*0.5; place(); }); } place(); }

example($('ex3'), { title:'Example 7.2: steam engine mechanism', src:'notes, Example 7.2 (Khurmi)',
  q:'The crank and connecting rod of a steam engine are 0.5 m and 2 m long; the crank makes 180 rpm clockwise. When it has turned 45° from the inner dead centre, find: the velocity of the piston; the angular velocity of the rod; the velocity of a point E on the rod 1.5 m from the gudgeon pin; the rubbing velocities at the crankshaft, crank pin and cross-head pins (diameters 50, 60 and 30 mm); and the point G on the rod with the least velocity.',
  inputs:[['r', 'Crank OB', 0.5, 'm'], ['l', 'Rod PB', 2, 'm'], ['N', 'Speed', 180, 'rpm'], ['phi', 'Angle from IDC', 45, '°'], ['PE', 'PE', 1.5, 'm'], ['dO', 'Shaft pin', 50, 'mm'], ['dB', 'Crank pin', 60, 'mm'], ['dC', 'Cross-head pin', 30, 'mm']],
  solve:v => { const old = { ...ENG }; Object.assign(ENG, { r:v.r, l:v.l, N:v.N, PE:v.PE, dO:v.dO, dB:v.dB, dC:v.dC }); const e = engine(v.phi*DEG); Object.assign(ENG, old); const w = rpm(v.N);
    return [['ω_BO = 2πN/60 =', `${n2(w, 3)} rad/s`], ['v_B = ω_BO × OB =', `${n2(w*v.r, 3)} m/s`], ['Velocity of the piston v_P = op =', `${n2(Math.abs(e.vP), 3)} m/s`, true],
      ['v_PB = bp =', `${n2(Math.abs(e.w3)*v.l, 3)} m/s`], ['ω_PB = v_PB/PB =', `${n2(Math.abs(e.w3), 3)} rad/s ${sense(e.w3)}`, true], ['e divides bp as E divides BP; v_E = oe =', `${n2(vlen(e.vE), 3)} m/s`, true],
      ['Rubbing at the crankshaft = (d_O/2) ω_BO =', `${n2(e.rub.O, 3)} m/s`], ['At the crank pin = (d_B/2)(ω_BO + ω_PB), the two turn opposite ways =', `${n2(e.rub.B, 4)} m/s`], ['At the cross-head = (d_C/2) ω_PB =', `${n2(e.rub.C, 4)} m/s`, true],
      ['G is the foot of the perpendicular from o to bp. BG =', `${n2(e.BG, 3)} m`], ['v_G = og =', `${n2(vlen(e.vG), 3)} m/s`, true]]; } });

/* ================= 3.3 mechanical advantage, Example 7.10 ================= */
{ const P = {}, cv = $('ma'); const L = { DC:0.6, DA:0.3, AB:0.36, CB:0.36 };
  controls($('ma_c'), [['th', 'Angle ADC', 0, 359, 1, 60, '°'], ['TA', 'Driving torque on DA', 5, 200, 1, 50, 'N·m'], ['eta', 'Efficiency η', 0.3, 1, 0.01, 0.7, '']], P, () => draw());
  const kin = th => fbKin([0, 0], [L.DC, 0], L.DA, L.AB, L.CB, th*DEG, -rpm(100));
  function draw(){ const k = kin(P.th); if (!k) return; const wA = rpm(100), wB = Math.abs(k.w4), MA = P.eta*wA/wB, TB = MA*P.TA;
    const V = fitView(cv, { xl:-0.35, xr:0.95, yb:-0.2, yt:0.75 }), { ctx } = V, Pp = V.P; groundAt(ctx, ...Pp([L.DC/2, 0]), L.DC*V.sc + 40, 0);
    barAt(ctx, ...Pp(k.A), ...Pp(k.B), C.crank, 7); barAt(ctx, ...Pp(k.B), ...Pp(k.C), C.rod, 7); barAt(ctx, ...Pp(k.D), ...Pp(k.C), C.vel, 7);
    pivotAt(ctx, ...Pp(k.A)); pivotAt(ctx, ...Pp(k.D)); pinAt(ctx, ...Pp(k.B)); pinAt(ctx, ...Pp(k.C));
    for (const [q, n] of [[k.A, 'D'], [k.B, 'A'], [k.C, 'B'], [k.D, 'C']]) lbl(ctx, n, Pp(q)[0] - 10, Pp(q)[1] - 12, C.ink, 'right');
    const tq = (q, col, txt, cw) => { const c = Pp(q); ctx.strokeStyle = col; ctx.lineWidth = 2.2; ctx.beginPath(); ctx.arc(...c, 24, cw ? -2.4 : 0.6, cw ? 0.6 : -2.4, !cw); ctx.stroke(); lbl(ctx, txt, c[0], c[1] + 40, col, 'center', 11.5); };
    tq(k.A, C.crank, `T_A = ${n2(P.TA, 0)} N·m`, true); tq(k.D, C.vel, `T_B = ${n2(TB, 1)} N·m`, false);
    const X = [], Y = []; for (let i = 0; i <= 360; i++){ const q = kin(i); X.push(i); Y.push(q ? Math.min(8, P.eta*wA/Math.abs(q.w4)) : NaN); }
    chart($('ma_g'), { x:X, series:[{ y:Y, color:C.force, label:'actual M.A. = η ω_A/ω_B (capped at 8)' }], ylab:'M.A.', xlab:'angle ADC [°]', cursor:P.th, cursorIndex:Math.round(P.th), value:`M.A. = ${n2(MA, 2)}` });
    $('ma_st').innerHTML = statsHTML([['ω_A of DA', `${n2(wA, 2)} <small>rad/s</small>`], ['v_A', `${n2(wA*L.DA, 3)} <small>m/s</small>`], ['v_B', `${n2(vlen(k.vC), 3)} <small>m/s</small>`], ['ω_B of CB', `${n2(wB, 3)} <small>rad/s</small>`], ['Actual M.A.', n2(MA, 3)], ['Resisting torque T_B', `${n2(TB, 1)} <small>N·m</small>`]]);
    $('ma_sub').textContent = 'the M.A. shoots up where CB stops (its toggle positions)'; }
  [cv, $('ma_g')].forEach(c => watchSize(c, draw)); onRedraw(draw); const an = animate(cv, dt => { P.th = (P.th - dt*30 + 360) % 360; draw(); }); an.playing = false; playButton($('ma_play'), an); draw(); }

example($('ex4'), { title:'Example 7.10: mechanical advantage and resisting torque', src:'notes, Example 7.10 (Khurmi)',
  q:'In the four-bar mechanism DABC the fixed link DC = 600 mm, crank DA = 300 mm, AB = CB = 360 mm, and angle ADC = 60°. DA turns at 100 rpm with a driving torque of 50 N·m. If the efficiency is 70%, find the velocity of B, the angular velocity of CB, the actual mechanical advantage and the resisting torque.',
  inputs:[['DC', 'DC', 600, 'mm'], ['DA', 'DA', 300, 'mm'], ['AB', 'AB', 360, 'mm'], ['CB', 'CB', 360, 'mm'], ['th', 'Angle ADC', 60, '°'], ['N', 'Speed of DA', 100, 'rpm'], ['TA', 'Driving torque', 50, 'N·m'], ['eta', 'Efficiency', 0.7, '']],
  solve:v => { const k = fbKin([0, 0], [v.DC/1000, 0], v.DA/1000, v.AB/1000, v.CB/1000, v.th*DEG, -rpm(v.N)), wA = rpm(v.N), wB = Math.abs(k.w4), MA = v.eta*wA/wB;
    return [['ω_A = 2πN/60 =', `${n2(wA, 2)} rad/s`], ['v_A = ω_A × DA (da ⟂ DA) =', `${n2(wA*v.DA/1000, 3)} m/s`], ['ab ⟂ AB and cb ⟂ CB meet at b: v_B = cb =', `${n2(vlen(k.vC), 3)} m/s`, true],
      ['ω_B = v_B/CB =', `${n2(wB, 3)} rad/s ${sense(k.w4)}`, true], ['M.A.(actual) = η ω_A/ω_B =', n2(MA, 3), true], ['η = T_B ω_B/(T_A ω_A), so T_B = η T_A ω_A/ω_B =', `${n2(MA*v.TA, 1)} N·m`, true]]; } });

/* ================= 3.4 the two components ================= */
{ const P = {}, cv = $('ac'); let th = 0.5;
  controls($('ac_c'), [['w', 'Angular velocity ω', 0, 10, 0.1, 4, 'rad/s'], ['al', 'Angular acceleration α', -60, 60, 1, 20, 'rad/s²'], ['L', 'Length AB', 0.1, 0.6, 0.01, 0.4, 'm']], P, () => draw());
  function draw(){ const V = fitView(cv, { xl:-0.62, xr:0.62, yb:-0.42, yt:0.5 }), { ctx } = V, Pp = V.P, A = [-0.25, -0.2], B = vadd(A, [P.L*Math.cos(th), P.L*Math.sin(th)]), r = vsub(B, A), u = vmul(r, 1/P.L);
    const ar = vmul(u, -P.w*P.w*P.L), at = cr(P.al, r), a = vadd(ar, at), mx = Math.max(vlen(ar) + vlen(at), 1e-6), s = 0.4/mx;
    barAt(ctx, ...Pp(A), ...Pp(B), C.crank, 8); pivotAt(ctx, ...Pp(A)); pinAt(ctx, ...Pp(B));
    const p1 = vadd(B, vmul(ar, s)), p2 = vadd(p1, vmul(at, s)); arrow(ctx, ...Pp(B), ...Pp(p1), C.acc, 2.6); arrow(ctx, ...Pp(p1), ...Pp(p2), C.force, 2.6); arrow(ctx, ...Pp(B), ...Pp(p2), C.ink, 1.8);
    const off = (p, q, k) => { const d = vsub(Pp(q), Pp(p)), n = vperp(d), l = vlen(n) || 1; return vadd(Pp(mid(p, q)), vmul(n, k/l)); }; lbl(ctx, 'aʳ (towards A)', ...off(B, p1, 16), C.acc, 'center'); lbl(ctx, 'aᵗ (⟂ AB)', ...off(p1, p2, -16), C.force, 'center'); lbl(ctx, 'a_BA', ...off(B, p2, 16), C.ink, 'center'); lbl(ctx, 'A', Pp(A)[0] - 10, Pp(A)[1] - 10, C.ink, 'right'); lbl(ctx, 'B', Pp(B)[0] + 10, Pp(B)[1] + 12, C.ink);
    $('ac_st').innerHTML = statsHTML([['v_BA = ω AB', `${n2(P.w*P.L, 3)} <small>m/s</small>`], ['aʳ = ω² AB = v²/AB', `${n2(vlen(ar), 3)} <small>m/s²</small>`], ['aᵗ = α AB', `${n2(Math.abs(P.al)*P.L, 3)} <small>m/s²</small>`], ['Total a_BA', `${n2(vlen(a), 3)} <small>m/s²</small>`]]);
    $('ac_sub').textContent = 'radial part along the link, tangential part across it'; }
  watchSize(cv, draw); onRedraw(draw); animate(cv, dt => { th += dt*0.15; draw(); }); draw(); }

/* ================= slider-crank diagrams, Example 8.1 ================= */
{ const P = {}, cv = $('sa');
  controls($('sa_c'), [['phi', 'Crank angle from IDC', 0, 359, 1, 45, '°'], ['N', 'Crank speed (clockwise)', 50, 600, 10, 300, 'rpm']], P, () => draw());
  function draw(){ const r = 0.15, l = 0.6, k = scKin(r, l, PI - P.phi*DEG, -rpm(P.N)); if (!k) return; const D = mid(k.B, k.P), vD = vadd(k.vB, cr(k.w3, vsub(D, k.B))), aD = vadd(vadd(k.aB, cr(k.a3, vsub(D, k.B))), vmul(vsub(D, k.B), -k.w3*k.w3));
    const V = fitView(cv, { xl:-0.82, xr:0.2, yb:-0.3, yt:0.3 }, 10), { ctx } = V, Pp = V.P;
    ctx.strokeStyle = C.muted; ctx.lineWidth = 1.3; ctx.beginPath(); ctx.moveTo(...Pp([-0.82, -0.03])); ctx.lineTo(...Pp([-0.4, -0.03])); ctx.stroke(); groundAt(ctx, ...Pp([-0.61, -0.03]), 0.42*V.sc, 0);
    barAt(ctx, ...Pp(k.O), ...Pp(k.B), C.crank, 7); barAt(ctx, ...Pp(k.B), ...Pp(k.P), C.rod, 6); ctx.fillStyle = alpha(C.vel, 0.85); ctx.fillRect(Pp(k.P)[0] - 16, Pp(k.P)[1] - 11, 32, 22);
    pivotAt(ctx, ...Pp(k.O)); pinAt(ctx, ...Pp(k.B)); pinAt(ctx, ...Pp(k.P)); dotAt(ctx, ...Pp(D), C.force, 4.5);
    for (const [q, n, dy] of [[k.O, 'O', -12], [k.B, 'B', -12], [k.P, 'A', -18], [D, 'D', -12]]) lbl(ctx, n, Pp(q)[0], Pp(q)[1] + dy, C.ink, 'center');
    vecDiagram($('sa_v'), { o:[0, 0], b:k.vB, a:[k.vP, 0], d:vD }, [['o', 'b', C.crank, 'v_B'], ['b', 'a', C.rod, 'v_AB'], ['o', 'a', C.vel, 'v_A'], ['o', 'd', C.force, 'v_D', 'thin']], { title:'velocity diagram (m/s)' });
    const x = vadd(k.aB, k.arPB);
    vecDiagram($('sa_a'), { "o′":[0, 0], "b′":k.aB, x, "a′":[k.aP, 0], "d′":aD }, [["o′", "b′", C.crank, 'a_B'], ["b′", 'x', C.acc, 'aʳ_AB', 'thin'], ['x', "a′", C.force, 'aᵗ_AB', 'thin'], ["o′", "a′", C.vel, 'a_A'], ["o′", "d′", C.ink, 'a_D', 'thin'], ["b′", "a′", alpha(C.rod, 0.6), '', 'line']], { title:'acceleration diagram (m/s²)' });
    $('sa_st').innerHTML = statsHTML([['v_B', `${n2(vlen(k.vB), 3)} <small>m/s</small>`], ['v_A (slider)', `${n2(Math.abs(k.vP), 3)} <small>m/s</small>`], ['v_AB', `${n2(Math.abs(k.w3)*l, 3)} <small>m/s</small>`], ['v_D (midpoint)', `${n2(vlen(vD), 3)} <small>m/s</small>`], ['ω_AB', `${n2(Math.abs(k.w3), 2)} <small>rad/s</small>`],
      ['a_B', `${n2(vlen(k.aB), 1)} <small>m/s²</small>`], ['aʳ_AB', `${n2(vlen(k.arPB), 2)} <small>m/s²</small>`], ['aᵗ_AB', `${n2(vlen(k.atPB), 1)} <small>m/s²</small>`], ['a_D', `${n2(vlen(aD), 1)} <small>m/s²</small>`], ['α_AB', `${n2(Math.abs(k.a3), 1)} <small>rad/s²</small> ${sense(k.a3)}`]]);
    $('sa_sub').textContent = 'crank 150 mm, rod 600 mm, constant crank speed'; }
  ['sa', 'sa_v', 'sa_a'].forEach(c => watchSize($(c), draw)); onRedraw(draw); const an = animate(cv, dt => { P.phi = (P.phi + dt*30) % 360; draw(); }); an.playing = false; playButton($('sa_play'), an); draw(); }

example($('ex5'), { title:'Example 8.1: slider-crank, midpoint of the rod', src:'notes, Example 8.1 (Khurmi)',
  q:'The crank of a slider-crank mechanism turns clockwise at a constant 300 rpm. The crank is 150 mm and the connecting rod 600 mm. At 45° from the inner dead centre find the velocity and acceleration of the midpoint D of the rod, and the angular velocity and angular acceleration of the rod.',
  inputs:[['r', 'Crank OB', 150, 'mm'], ['l', 'Rod BA', 600, 'mm'], ['N', 'Speed', 300, 'rpm'], ['phi', 'Angle from IDC', 45, '°']],
  solve:v => { const r = v.r/1000, l = v.l/1000, k = scKin(r, l, PI - v.phi*DEG, -rpm(v.N)), D = mid(k.B, k.P), vD = vadd(k.vB, cr(k.w3, vsub(D, k.B))), aD = vadd(vadd(k.aB, cr(k.a3, vsub(D, k.B))), vmul(vsub(D, k.B), -k.w3*k.w3)), w = rpm(v.N);
    return [['ω_BO =', `${n2(w, 2)} rad/s`], ['v_B = ω_BO × OB =', `${n2(w*r, 3)} m/s`], ['v_AB = ba =', `${n2(Math.abs(k.w3)*l, 3)} m/s`], ['v_A = oa =', `${n2(Math.abs(k.vP), 3)} m/s`], ['d is the midpoint of ba: v_D = od =', `${n2(vlen(vD), 3)} m/s`, true],
      ['a_B = v_B²/OB (no tangential part: constant speed) =', `${n2(vlen(k.aB), 1)} m/s²`], ['aʳ_AB = v_AB²/BA =', `${n2(vlen(k.arPB), 2)} m/s²`], ['d′ is the midpoint of b′a′: a_D = o′d′ =', `${n2(vlen(aD), 1)} m/s²`, true],
      ['ω_AB = v_AB/BA =', `${n2(Math.abs(k.w3), 2)} rad/s ${sense(k.w3)}`, true], ['aᵗ_AB from the diagram =', `${n2(vlen(k.atPB), 1)} m/s²`], ['α_AB = aᵗ_AB/BA =', `${n2(Math.abs(k.a3), 1)} rad/s² ${sense(k.a3)}`, true]]; } });

function ex83(v){ const A = [0, 0], B = [v.AB*Math.sin(v.t1*DEG), -v.AB*Math.cos(v.t1*DEG)], Cc = vadd(B, [-v.BC*Math.cos(v.t2*DEG), -v.BC*Math.sin(v.t2*DEG)]), rAB = vsub(B, A), rCB = vsub(B, Cc);
  const [w2, w3] = solve2(-rAB[1], rCB[1], rAB[0], -rCB[0], v.vC, 0), rhs = vadd(vsub([v.aC, 0], vmul(rCB, w3*w3)), vmul(rAB, w2*w2)), [a2, a3] = solve2(-rAB[1], rCB[1], rAB[0], -rCB[0], rhs[0], rhs[1]);
  return { w2, w3, a2, a3, aB:vadd(cr(a2, rAB), vmul(rAB, -w2*w2)) }; }
example($('ex6'), { title:'Example 8.3: link and slider', src:'notes, Example 8.3 (Khurmi)',
  q:'In the mechanism, the slider C moves to the right at 1 m/s with an acceleration of 2.5 m/s². AB = 3 m is inclined at 45° to the vertical and BC = 1.5 m at 45° to the horizontal. Find the vertical and horizontal components of the acceleration of B and the angular accelerations of AB and BC.',
  inputs:[['AB', 'AB', 3, 'm'], ['BC', 'BC', 1.5, 'm'], ['t1', 'AB to the vertical', 45, '°'], ['t2', 'BC to the horizontal', 45, '°'], ['vC', 'v_C', 1, 'm/s'], ['aC', 'a_C', 2.5, 'm/s²']],
  solve:v => { const r = ex83(v); return [['ab ⟂ AB and cb ⟂ CB from the velocity diagram: v_BA =', `${n2(Math.abs(r.w2)*v.AB, 3)} m/s`], ['v_BC =', `${n2(Math.abs(r.w3)*v.BC, 3)} m/s`],
    ['aʳ_BC = v_BC²/CB =', `${n2(r.w3*r.w3*v.BC, 3)} m/s²`], ['aʳ_BA = v_BA²/AB =', `${n2(r.w2*r.w2*v.AB, 3)} m/s²`], ['Vertical component of a_B =', `${n2(Math.abs(r.aB[1]), 3)} m/s²`, true], ['Horizontal component of a_B =', `${n2(Math.abs(r.aB[0]), 3)} m/s²`, true],
    ['α_AB = aᵗ_BA/AB =', `${n2(Math.abs(r.a2), 3)} rad/s²`, true], ['α_BC = aᵗ_BC/CB =', `${n2(Math.abs(r.a3), 3)} rad/s²`, true]]; } });

/* ================= four-bar diagrams, Example 8.4 ================= */
{ const P = {}, cv = $('fa'); const L = { PS:0.2, PQ:0.0625, QR:0.175, RS:0.1125 };
  controls($('fa_c'), [['th', 'Angle QPS', 0, 359, 1, 60, '°'], ['w', 'ω of PQ (clockwise)', 1, 30, 0.5, 10, 'rad/s']], P, () => draw());
  function draw(){ const k = fbKin([0, 0], [L.PS, 0], L.PQ, L.QR, L.RS, P.th*DEG, -P.w); if (!k) return;
    const V = fitView(cv, { xl:-0.05, xr:0.25, yb:-0.06, yt:0.16 }, 10), { ctx } = V, Pp = V.P; groundAt(ctx, ...Pp([L.PS/2, 0]), L.PS*V.sc + 40, 0);
    barAt(ctx, ...Pp(k.A), ...Pp(k.B), C.crank, 7); barAt(ctx, ...Pp(k.B), ...Pp(k.C), C.rod, 7); barAt(ctx, ...Pp(k.D), ...Pp(k.C), C.vel, 7);
    pivotAt(ctx, ...Pp(k.A)); pivotAt(ctx, ...Pp(k.D)); pinAt(ctx, ...Pp(k.B)); pinAt(ctx, ...Pp(k.C));
    for (const [q, n] of [[k.A, 'P'], [k.B, 'Q'], [k.C, 'R'], [k.D, 'S']]) lbl(ctx, n, Pp(q)[0] - 10, Pp(q)[1] - 12, C.ink, 'right');
    vecDiagram($('fa_v'), { 'p, s':[0, 0], q:k.vB, r:k.vC }, [['p, s', 'q', C.crank, 'v_Q'], ['q', 'r', C.rod, 'v_RQ'], ['p, s', 'r', C.vel, 'v_R']], { title:'velocity diagram (m/s)' });
    const x = vadd(k.aB, k.arCB), y = k.arCD;
    vecDiagram($('fa_a'), { "p′, s′":[0, 0], "q′":k.aB, x, y, "r′":k.aC }, [["p′, s′", "q′", C.crank, 'a_Q'], ["q′", 'x', C.acc, 'aʳ_RQ', 'thin'], ['x', "r′", C.force, 'aᵗ_RQ', 'thin'], ["p′, s′", 'y', C.acc, 'aʳ_RS', 'thin'], ['y', "r′", C.force, 'aᵗ_RS', 'thin'], ["p′, s′", "r′", C.vel, 'a_R']], { title:'acceleration diagram (m/s²)' });
    $('fa_st').innerHTML = statsHTML([['v_Q', `${n2(vlen(k.vB), 3)} <small>m/s</small>`], ['v_RQ', `${n2(vlen(k.vCB), 3)} <small>m/s</small>`], ['v_R', `${n2(vlen(k.vC), 3)} <small>m/s</small>`], ['ω_QR', `${n2(Math.abs(k.w3), 2)} <small>rad/s</small> ${sense(k.w3)}`], ['ω_RS', `${n2(Math.abs(k.w4), 2)} <small>rad/s</small> ${sense(k.w4)}`],
      ['aᵗ_RQ', `${n2(vlen(k.atCB), 2)} <small>m/s²</small>`], ['aᵗ_RS', `${n2(vlen(k.atCD), 2)} <small>m/s²</small>`], ['α_QR', `${n2(Math.abs(k.a3), 1)} <small>rad/s²</small> ${sense(k.a3)}`], ['α_RS', `${n2(Math.abs(k.a4), 1)} <small>rad/s²</small> ${sense(k.a4)}`]]);
    $('fa_sub').textContent = 'PQ = 62.5, QR = 175, RS = 112.5, PS = 200 mm'; }
  ['fa', 'fa_v', 'fa_a'].forEach(c => watchSize($(c), draw)); onRedraw(draw); const an = animate(cv, dt => { P.th = (P.th - dt*30 + 360) % 360; draw(); }); an.playing = false; playButton($('fa_play'), an); draw(); }

example($('ex7'), { title:'Example 8.4: four-bar PQRS', src:'notes, Example 8.4 (Khurmi)',
  q:'PQRS is a four-bar chain with PS fixed. PQ = 62.5 mm, QR = 175 mm, RS = 112.5 mm and PS = 200 mm. The crank PQ turns at 10 rad/s clockwise. With angle QPS = 60° and Q and R on the same side of PS, find the angular velocity and angular acceleration of QR and RS.',
  inputs:[['PQ', 'PQ', 62.5, 'mm'], ['QR', 'QR', 175, 'mm'], ['RS', 'RS', 112.5, 'mm'], ['PS', 'PS', 200, 'mm'], ['th', 'Angle QPS', 60, '°'], ['w', 'ω of PQ', 10, 'rad/s']],
  solve:v => { const k = fbKin([0, 0], [v.PS/1000, 0], v.PQ/1000, v.QR/1000, v.RS/1000, v.th*DEG, -v.w);
    return [['v_Q = ω × PQ =', `${n2(v.w*v.PQ/1000, 3)} m/s`], ['v_RQ = qr =', `${n2(vlen(k.vCB), 3)} m/s`], ['v_R = sr =', `${n2(vlen(k.vC), 3)} m/s`], ['ω_QR = v_RQ/QR =', `${n2(Math.abs(k.w3), 2)} rad/s ${sense(k.w3)}`, true],
      ['ω_RS = v_R/RS =', `${n2(Math.abs(k.w4), 2)} rad/s ${sense(k.w4)}`, true], ['a_Q = v_Q²/PQ (no α for PQ) =', `${n2(vlen(k.aB), 2)} m/s²`], ['aʳ_RQ = v_RQ²/QR =', `${n2(vlen(k.arCB), 3)} m/s²`], ['aʳ_RS = v_R²/SR =', `${n2(vlen(k.arCD), 3)} m/s²`],
      ['aᵗ_RQ = xr′ =', `${n2(vlen(k.atCB), 2)} m/s²`], ['aᵗ_RS = yr′ =', `${n2(vlen(k.atCD), 2)} m/s²`], ['α_QR = aᵗ_RQ/QR =', `${n2(Math.abs(k.a3), 2)} rad/s² ${sense(k.a3)}`, true], ['α_RS = aᵗ_RS/SR =', `${n2(Math.abs(k.a4), 2)} rad/s² ${sense(k.a4)}`, true]]; } });

/* ================= quiz ================= */
quiz($('qz'), [
  { q:'How many instantaneous centres does a six-link mechanism have?', o:['6', '12', '15', '30'], a:2, why:'N = n(n − 1)/2 = 6 × 5/2 = 15.' },
  { q:'A wheel rolls without slipping. Where is its instantaneous centre relative to the ground?', o:['At the wheel centre', 'At the contact point', 'At the top of the wheel', 'At infinity'], a:1, why:'The contact point is momentarily at rest, so the wheel turns about it; the top moves at twice the speed of the centre.' },
  { q:"Kennedy's theorem says that for any three bodies…", o:['their centres form a triangle', 'their three instantaneous centres lie on one straight line', 'one centre is always fixed', 'their angular velocities add to zero'], a:1, why:'The three instantaneous centres of three bodies are collinear; this finds the centres that cannot be seen by inspection.' },
  { q:'The velocity of B relative to A, two points on the same rigid link, is…', o:['along AB', 'perpendicular to AB', 'zero', 'parallel to the velocity of A'], a:1, why:'A rigid link cannot stretch, so relative motion is rotation: v_BA = ω × AB, perpendicular to AB.' },
  { q:'A point C is one third of the way from A to B on a link. On the velocity diagram, c is…', o:['at the pole o', 'one third of the way from a to b', 'two thirds of the way from a to b', 'perpendicular to ab'], a:1, why:'The velocity image: c divides ab in the same ratio as C divides AB.' },
  { q:'Two links joined by a 40 mm pin turn at 5 rad/s clockwise and 3 rad/s anticlockwise. The rubbing velocity is…', o:['0.04 m/s', '0.08 m/s', '0.16 m/s', '0.32 m/s'], a:2, why:'Opposite senses: (ω₁ + ω₂) r = 8 × 0.02 = 0.16 m/s.' },
  { q:'In a four-bar, the input link turns at 10 rad/s and the output at 4 rad/s. With 80% efficiency, the actual mechanical advantage is…', o:['0.32', '2', '2.5', '3.125'], a:1, why:'M.A.(actual) = η ω_A/ω_B = 0.8 × 10/4 = 2.' },
  { q:'Which component of the acceleration of B relative to A acts along the link AB?', o:['The tangential component', 'The radial (centripetal) component', 'Both', 'Neither'], a:1, why:'aʳ = ω² AB acts along BA towards A; aᵗ = α AB acts perpendicular to the link.' },
  { q:'A crank turns at constant speed. The acceleration of the crank pin has…', o:['only a tangential part', 'only a radial part ω²r towards the centre', 'both parts', 'no acceleration'], a:1, why:'Constant ω means α = 0, so only the centripetal part ω²r is left.' },
  { q:'A rod BA of 0.5 m has aᵗ_BA = 20 m/s². Its angular acceleration is…', o:['10 rad/s²', '20 rad/s²', '40 rad/s²', '100 rad/s²'], a:2, why:'α = aᵗ/AB = 20/0.5 = 40 rad/s².' } ]);
})();
