/* BME 2102 interactive notes, Chapter 4: belt drives */
(() => {
'use strict';
const PI = Math.PI, TAU = 2*PI;
const lbl = (ctx, t, x, y, col, al = 'left', size = 12) => label(ctx, t, x, y, col, al, size, true);
const mid = (p, q, k = 0.5) => [p[0] + (q[0] - p[0])*k, p[1] + (q[1] - p[1])*k];

/* A closed belt round circles taken in order of travel. s = +1: the pulley is on the left of the belt (belt runs anticlockwise round it); s = -1: on the right. */
function loop2D(cs, n = 48){ const T = [];
  for (let i = 0; i < cs.length; i++){ const a = cs[i], b = cs[(i + 1) % cs.length], D = vsub(b.c, a.c), L = vlen(D), be = Math.atan2(D[1], D[0]), k = b.s*b.r - a.s*a.r, g = Math.acos(clamp(k/L, -1, 1));
    let best = null; for (const gm of [be + g, be - g]){ const nn = [Math.cos(gm), Math.sin(gm)], d = [nn[1], -nn[0]]; if (D[0]*d[0] + D[1]*d[1] > 0) best = nn; }
    const nn = best; T.push({ pa:vsub(a.c, vmul(nn, a.s*a.r)), pb:vsub(b.c, vmul(nn, b.s*b.r)) }); }
  const pts = [], arcs = []; let L = 0;
  for (let i = 0; i < cs.length; i++){ const c = cs[i], arr = T[(i + cs.length - 1) % cs.length].pb, dep = T[i].pa; let a0 = Math.atan2(arr[1] - c.c[1], arr[0] - c.c[0]), a1 = Math.atan2(dep[1] - c.c[1], dep[0] - c.c[0]);
    if (c.s > 0){ while (a1 < a0) a1 += TAU; } else { while (a1 > a0) a1 -= TAU; } arcs.push(Math.abs(a1 - a0));
    for (let j = 0; j <= n; j++){ const a = a0 + (a1 - a0)*j/n; pts.push([c.c[0] + c.r*Math.cos(a), c.c[1] + c.r*Math.sin(a)]); }
    pts.push(T[i].pb); }
  for (let i = 0; i < pts.length; i++) L += vlen(vsub(pts[(i + 1) % pts.length], pts[i]));
  return { pts, L, T, arcs }; }

/* ================= 4.1 belt drives in 3D ================= */
{ let mode = 'open', t = 0; const G = {}, V0 = 0.7;
  const CAP = { open:'<b>Open belt.</b> The shafts are parallel and turn the same way. With the drive horizontal, keep the tight side at the bottom.',
    cross:'<b>Crossed (twist) belt.</b> The shafts are parallel and turn opposite ways. The belt rubs on itself where it crosses, so it wears faster; run it slowly and keep the centres at least 20 belt widths apart.',
    quarter:'<b>Quarter turn.</b> The shafts are at right angles. The belt must approach each pulley in that pulley\'s mid-plane, so the strands twist through 90°. Wide pulleys (at least 1.4 times the belt width) keep it on.',
    idler:'<b>With an idler pulley.</b> A free (idler) pulley presses on the slack side. It raises the tension and the arc of contact on both pulleys when the centres are fixed and close.',
    compound:'<b>Compound drive.</b> Pulley 1 drives 2; 3 is keyed to the same shaft as 2 and drives 4. N₄/N₁ = (d₁ × d₃)/(d₂ × d₄): here 2 × 2.25 = 4.5 times as fast. Used for large speed ratios.',
    cone:'<b>Stepped (cone) pulleys.</b> Moving the belt from step to step changes the speed of the driven shaft while the driver runs at constant speed, as in lathe and drilling machine drives. The belt moves to a new step every few seconds.',
    fast:'<b>Fast and loose pulleys.</b> The driven shaft carries a fast pulley keyed to it and a loose pulley that turns freely. Shifting the belt onto the loose pulley stops the driven shaft without stopping the driver.',
    vbelt:'<b>V-belts.</b> Several V-belts run side by side in grooved pulleys, for short centre distances and more power.' };
  function stripeTex(col){ const c = document.createElement('canvas'); c.width = 64; c.height = 8; const x = c.getContext('2d'); x.fillStyle = col; x.fillRect(0, 0, 64, 8); x.fillStyle = 'rgba(0,0,0,0.45)'; x.fillRect(0, 0, 14, 8);
    const tx = new THREE.CanvasTexture(c); tx.wrapS = THREE.RepeatWrapping; tx.wrapT = THREE.RepeatWrapping; tx.encoding = THREE.sRGBEncoding; return tx; }
  function ribbon(samples, width, tex, period = 0.12){ const T = THREE, n = samples.length, pos = new Float32Array((n + 1)*6), uv = new Float32Array((n + 1)*4), idx = []; let s = 0;
    for (let i = 0; i <= n; i++){ const q = samples[i % n], p = q.p, w = q.w; if (i > 0) s += p.distanceTo(samples[i - 1].p);
      pos.set([p.x - w.x*width/2, p.y - w.y*width/2, p.z - w.z*width/2, p.x + w.x*width/2, p.y + w.y*width/2, p.z + w.z*width/2], i*6); uv.set([s/period, 0, s/period, 1], i*4);
      if (i < n) idx.push(2*i, 2*i + 1, 2*i + 2, 2*i + 1, 2*i + 3, 2*i + 2); }
    const g = new T.BufferGeometry(); g.setAttribute('position', new T.BufferAttribute(pos, 3)); g.setAttribute('uv', new T.BufferAttribute(uv, 2)); g.setIndex(idx); g.computeVertexNormals();
    return new T.Mesh(g, new T.MeshStandardMaterial({ map:tex, side:T.DoubleSide, roughness:0.8, metalness:0 })); }
  const flat = (lp, z = 0) => lp.pts.map(p => ({ p:new THREE.Vector3(p[0], p[1], z), w:new THREE.Vector3(0, 0, 1) }));
  function pulley(r, w, col, axis = 'z', grooves = 0){ const T = THREE, g = new T.Group(); let m;
    if (grooves){ const pts = [new T.Vector2(0.05, -w/2)]; const gw = w/grooves; for (let i = 0; i < grooves; i++){ const z0 = -w/2 + i*gw; pts.push(new T.Vector2(r + 0.03, z0 + 0.01), new T.Vector2(r - 0.05, z0 + gw/2), new T.Vector2(r + 0.03, z0 + gw - 0.01)); } pts.push(new T.Vector2(r + 0.03, w/2), new T.Vector2(0.05, w/2));
      m = new T.Mesh(new T.LatheGeometry(pts, 48), mat(col, { side:T.DoubleSide })); m.rotation.x = PI/2; }
    else { m = new T.Mesh(new T.CylinderGeometry(r - 0.012, r - 0.012, w, 48), mat(col)); m.rotation.x = PI/2; }
    g.add(m); const mk = new T.Mesh(new T.BoxGeometry(r*0.9, 0.035, 0.02), mat('#22303c')); mk.position.set(r*0.45, 0, w/2 + 0.011); g.add(mk); const mk2 = mk.clone(); mk2.position.z = -w/2 - 0.011; g.add(mk2);
    const hub = new T.Mesh(new T.CylinderGeometry(0.05, 0.05, w + 0.3, 16), mat('#9aa6b2', { metalness:0.7 })); hub.rotation.x = PI/2; g.add(hub);
    if (axis === 'x'){ const o = new T.Group(); o.add(g); g.rotation.y = PI/2; o.userData.inner = g; return o; } g.userData.inner = g; return g; }
  const spin = (pl, a) => { const inner = pl.userData.inner; inner.rotation.z = a; };
  const v = view3d($('bt3'), { cam:[1.4, 1.2, 4.2], target:[1.15, 0.1, 0], fov:36,
    build(v){ const T = THREE; readColors(); const belt = stripeTex('#3b3f45'), vb = stripeTex('#2c2f34'); vb.repeat.set(1, 1); G.tex = [belt, vb]; const mk = () => { const g = new T.Group(); v.root.add(g); return g; };
      const steel = '#9aa6b2', blue = C.crank, green = C.vel, orange = C.rod;
      // open and crossed
      for (const k of ['open', 'cross']){ const g = mk(), cs = [{ c:[0, 0], r:0.55, s:1 }, { c:[2.3, 0], r:0.3, s:k === 'open' ? 1 : -1 }], lp = loop2D(cs);
        const A = pulley(0.55, 0.24, blue), B = pulley(0.3, 0.24, green); B.position.set(2.3, 0, 0); g.add(A, B, ribbon(flat(lp), 0.18, belt)); G[k] = { g, spin:[[A, 1/0.55], [B, (k === 'open' ? 1 : -1)/0.3]] }; }
      // quarter turn
      { const g = mk(), r1 = 0.5, r2 = 0.32, H = 2.0, S = [];
        for (let i = 0; i <= 40; i++){ const a = PI + PI*i/40; S.push({ p:new T.Vector3(r1*Math.cos(a), r1*Math.sin(a), 0), w:new T.Vector3(0, 0, 1) }); }
        for (let i = 1; i < 30; i++){ const k = i/30, w = new T.Vector3(0, 0, 1).lerp(new T.Vector3(1, 0, 0), k).normalize(); S.push({ p:new T.Vector3(r1, H*k, 0), w }); }
        for (let i = 0; i <= 40; i++){ const f = -PI/2 + PI*i/40; S.push({ p:new T.Vector3(r1, H + r2*Math.cos(f), r2 + r2*Math.sin(f)), w:new T.Vector3(1, 0, 0) }); }
        for (let i = 1; i < 30; i++){ const k = i/30, w = new T.Vector3(1, 0, 0).lerp(new T.Vector3(0, 0, 1), k).normalize(); S.push({ p:new T.Vector3(r1 + (-r1 - r1)*k, H*(1 - k), 2*r2*(1 - k)), w }); }
        const A = pulley(r1, 0.3, blue), B = pulley(r2, 0.3, green, 'x'); B.position.set(r1, H, r2); g.add(A, B, ribbon(S, 0.16, belt)); G.quarter = { g, spin:[[A, 1/r1], [B, 1/r2]] }; }
      // idler
      { const g = mk(), cs = [{ c:[0, 0], r:0.5, s:1 }, { c:[2.4, 0], r:0.3, s:1 }, { c:[1.25, 0.43], r:0.14, s:-1 }], lp = loop2D(cs);
        const A = pulley(0.5, 0.24, blue), B = pulley(0.3, 0.24, green), I = pulley(0.14, 0.24, orange); B.position.set(2.4, 0, 0); I.position.set(1.25, 0.43, 0); g.add(A, B, I, ribbon(flat(lp), 0.18, belt)); G.idler = { g, spin:[[A, 1/0.5], [B, 1/0.3], [I, -1/0.14]] }; }
      // compound
      { const g = mk(), S1 = [0, 0], S2 = [1.7, 0.25], S3 = [3.0, -0.1], z1 = 0.17, z2 = -0.17;
        const l1 = loop2D([{ c:S1, r:0.5, s:1 }, { c:S2, r:0.25, s:1 }]), l2 = loop2D([{ c:S2, r:0.45, s:1 }, { c:S3, r:0.2, s:1 }]);
        const P1 = pulley(0.5, 0.2, blue), P2 = pulley(0.25, 0.2, green), P3 = pulley(0.45, 0.2, green), P4 = pulley(0.2, 0.2, orange);
        P1.position.set(...S1, z1); P2.position.set(...S2, z1); P3.position.set(...S2, z2); P4.position.set(...S3, z2);
        const r2 = ribbon(flat(l2, z2), 0.15, belt); r2.userData.speed = 0.45/0.25; g.add(P1, P2, P3, P4, ribbon(flat(l1, z1), 0.15, belt), r2); G.compound = { g, spin:[[P1, 1/0.5], [P2, 1/0.25], [P3, 1/0.25], [P4, 0.45/0.25/0.2]], tex2:true }; }
      // stepped pulleys
      { const g = mk(), R1 = [0.25, 0.38, 0.5], R2 = [0.5, 0.38, 0.25], Z = [0.24, 0, -0.24], A = new T.Group(), B = new T.Group();
        R1.forEach((r, i) => { const p = pulley(r, 0.22, blue); p.position.z = Z[i]; A.add(p); }); R2.forEach((r, i) => { const p = pulley(r, 0.22, green); p.position.z = Z[i]; B.add(p); });
        A.userData.inner = A; B.userData.inner = B; B.position.set(2.3, 0, 0); g.add(A, B);
        const belts = R1.map((r, i) => { const m = ribbon(flat(loop2D([{ c:[0, 0], r:R1[i], s:1 }, { c:[2.3, 0], r:R2[i], s:1 }]), Z[i]), 0.16, belt); m.visible = i === 1; g.add(m); return m; });
        G.cone = { g, A, B, belts, R1, R2, step:1 }; }
      // fast and loose
      { const g = mk(), A = pulley(0.45, 0.6, blue), F = pulley(0.35, 0.22, green), Lp = pulley(0.35, 0.22, orange); F.position.set(2.3, 0, 0.14); Lp.position.set(2.3, 0, -0.14); const sh = new T.Mesh(new T.CylinderGeometry(0.05, 0.05, 1.2, 12), mat('#9aa6b2')); sh.rotation.x = PI/2; sh.position.set(2.3, 0, 0);
        const rb = ribbon(flat(loop2D([{ c:[0, 0], r:0.45, s:1 }, { c:[2.3, 0], r:0.35, s:1 }])), 0.17, belt); g.add(A, F, Lp, sh, rb); G.fast = { g, A, F, Lp, rb }; }
      // V-belts
      { const g = mk(), A = pulley(0.5, 0.42, blue, 'z', 3), B = pulley(0.3, 0.42, green, 'z', 3); B.position.set(2.3, 0, 0); g.add(A, B);
        const lp = loop2D([{ c:[0, 0], r:0.46, s:1 }, { c:[2.3, 0], r:0.26, s:1 }]);
        class Path extends T.Curve { constructor(z){ super(); this.z = z; } getPoint(u){ const n = lp.pts.length, f = u*n, i = Math.floor(f) % n, k = f - Math.floor(f), a = lp.pts[i], b = lp.pts[(i + 1) % n]; return new T.Vector3(a[0] + (b[0] - a[0])*k, a[1] + (b[1] - a[1])*k, this.z); } }
        for (const z of [-0.14, 0, 0.14]){ const tb = new T.Mesh(new T.TubeGeometry(new Path(z), 400, 0.045, 4, true), new T.MeshStandardMaterial({ map:vb, roughness:0.8 })); g.add(tb); }
        vb.repeat.set(lp.L/0.12, 1); G.vbelt = { g, spin:[[A, 1/0.46], [B, 1/0.26]] }; }
      for (const k in G) if (G[k] && G[k].g) G[k].g.visible = k === mode; },
    update(dt){ t += dt; step(dt); } });
  let shiftT = 0;
  function step(dt){ if (!G.open) return; const d = V0*dt; G.tex[0].offset.x -= d/0.12; G.tex[1].offset.x -= d/0.12;
    const cur = G[mode]; if (!cur) return;
    if (cur.spin) for (const [p, k] of cur.spin) spin(p, p.userData.inner.rotation.z + d*k);
    if (mode === 'cone'){ shiftT += dt; if (shiftT > 4){ shiftT = 0; cur.step = (cur.step + 1) % 3; cur.belts.forEach((b, i) => b.visible = i === cur.step); caption(); }
      spin(cur.A, cur.A.rotation.z + d/cur.R1[cur.step]); spin(cur.B, cur.B.rotation.z + d/cur.R2[cur.step]); }
    if (mode === 'fast'){ shiftT += dt; const ph = (shiftT % 8)/8, z = ph < 0.4 ? 0.14 : ph < 0.5 ? 0.14 - 0.28*(ph - 0.4)/0.1 : ph < 0.9 ? -0.14 : -0.14 + 0.28*(ph - 0.9)/0.1; cur.rb.position.z = z;
      spin(cur.A, cur.A.userData.inner.rotation.z + d/0.45); const onFast = clamp((z + 0.14)/0.28, 0, 1); spin(cur.F, cur.F.userData.inner.rotation.z + d/0.35*onFast); spin(cur.Lp, cur.Lp.userData.inner.rotation.z + d/0.35*(1 - onFast));
      $('bt_sub').textContent = onFast > 0.5 ? 'belt on the fast pulley: the driven shaft turns' : 'belt on the loose pulley: the driven shaft stops'; } }
  function caption(){ let c = CAP[mode]; if (mode === 'cone' && G.cone){ const i = G.cone.step; c += ` Now on step ${i + 1}: N₂/N₁ = r₁/r₂ = ${n2(G.cone.R1[i]/G.cone.R2[i], 2)}.`; } $('bt_cap').innerHTML = c;
    if (mode !== 'fast') $('bt_sub').textContent = { open:'pulleys turn the same way', cross:'pulleys turn opposite ways', quarter:'shafts at right angles', idler:'idler on the slack side', compound:'three shafts', cone:'three speeds from one driver', vbelt:'three belts in grooved pulleys' }[mode]; }
  const CAMS = { quarter:[[3.2, 2.2, 3.4], [0.3, 1.0, 0.2]], compound:[[1.6, 1.3, 4.6], [1.5, 0.1, 0]] };
  segGroup($('bt_seg'), m => { mode = m; shiftT = 0; if (v){ const c = CAMS[m] || [[1.4, 1.2, 4.2], [1.15, 0.1, 0]]; v.cam.position.set(...c[0]); v.ctl.target.set(...c[1]); v.ctl.update(); } for (const k in G) if (G[k] && G[k].g) G[k].g.visible = k === m; caption(); if (v) v.render(); });
  caption(); if (v) playButton($('bt_play'), v.anim); }

/* ================= 4.2 velocity ratio, slip and creep ================= */
{ const P = {}, cv = $('vr'); let a1 = 0, a2 = 0, off = 0;
  controls($('vr_c'), [['d1', 'Driver diameter d₁', 100, 1000, 10, 600, 'mm'], ['d2', 'Driven diameter d₂', 100, 1000, 10, 300, 'mm'], ['N1', 'Driver speed N₁', 50, 1500, 10, 300, 'rpm'], ['t', 'Belt thickness t', 0, 20, 1, 0, 'mm'], ['s', 'Total slip s', 0, 10, 0.5, 0, '%']], P, () => draw());
  const N2 = () => P.N1*(P.d1 + P.t)/(P.d2 + P.t)*(1 - P.s/100);
  function draw(){ const r1 = P.d1/2, r2 = P.d2/2, x = Math.max(r1, r2)*2 + 300, lp = loop2D([{ c:[0, 0], r:r1 + P.t/2, s:1 }, { c:[x, 0], r:r2 + P.t/2, s:1 }]);
    const V = fitView(cv, { xl:-r1 - 40, xr:x + r2 + 40, yb:-Math.max(r1, r2) - 40, yt:Math.max(r1, r2) + 40 }), { ctx } = V, Pp = V.P;
    const wheel = (c, r, a, col) => { ctx.fillStyle = alpha(col, 0.15); ctx.strokeStyle = col; ctx.lineWidth = 2.5; ctx.beginPath(); ctx.arc(...Pp(c), r*V.sc, 0, 7); ctx.fill(); ctx.stroke();
      for (let i = 0; i < 4; i++){ const q = a + i*PI/2; ctx.lineWidth = i ? 2 : 4; ctx.strokeStyle = i ? alpha(col, 0.6) : col; ctx.beginPath(); ctx.moveTo(...Pp(c)); ctx.lineTo(...Pp(vadd(c, [r*0.92*Math.cos(q), r*0.92*Math.sin(q)]))); ctx.stroke(); } pinAt(ctx, ...Pp(c), 5); };
    wheel([0, 0], r1, a1, C.crank); wheel([x, 0], r2, a2, C.vel);
    ctx.strokeStyle = '#3b3f45'; ctx.lineWidth = Math.max(3, P.t*V.sc*0.9 + 3); ctx.setLineDash([14, 8]); ctx.lineDashOffset = a1*(r1 + P.t/2)*V.sc; ctx.beginPath(); lp.pts.forEach((q, i) => i ? ctx.lineTo(...Pp(q)) : ctx.moveTo(...Pp(q))); ctx.closePath(); ctx.stroke(); ctx.setLineDash([]); ctx.lineDashOffset = 0;
    lbl(ctx, `driver ${P.N1} rpm`, ...Pp([0, -r1 - 25]), C.crank, 'center'); lbl(ctx, `driven ${n2(N2(), 1)} rpm`, ...Pp([x, -r2 - 25]), C.vel, 'center');
    const v = PI*P.d1/1000*P.N1/60;
    $('vr_st').innerHTML = statsHTML([['Belt speed π d₁ N₁/60', `${n2(v, 2)} <small>m/s</small>`], ['N₂ = N₁ d₁/d₂', `${n2(P.N1*P.d1/P.d2, 1)} <small>rpm</small>`], ['With thickness (d₁ + t)/(d₂ + t)', `${n2(P.N1*(P.d1 + P.t)/(P.d2 + P.t), 1)} <small>rpm</small>`], ['With slip as well', `${n2(N2(), 1)} <small>rpm</small>`], ['Speed lost to slip', `${n2(P.N1*(P.d1 + P.t)/(P.d2 + P.t)*P.s/100, 1)} <small>rpm</small>`]]);
    $('vr_sub').textContent = 'both pulleys turn the same way (open belt)'; }
  watchSize(cv, draw); onRedraw(draw);
  const an = animate(cv, dt => { const k = 0.06; a1 += dt*P.N1*TAU/60*k; a2 += dt*N2()*TAU/60*k; draw(); }); playButton($('vr_play'), an); draw(); }

example($('ex1'), { title:'Example 11.1: compound drive with slip', src:'notes, Example 1 (Khurmi 11.1)',
  q:'An engine running at 150 rpm drives a line shaft by a belt. The engine pulley is 750 mm in diameter and the line-shaft pulley 450 mm. A 900 mm pulley on the line shaft drives a 150 mm pulley on a dynamo shaft. Find the dynamo speed (1) with no slip, (2) with 2% slip at each drive.',
  inputs:[['N1', 'Engine speed N₁', 150, 'rpm'], ['d1', 'd₁', 750, 'mm'], ['d2', 'd₂', 450, 'mm'], ['d3', 'd₃', 900, 'mm'], ['d4', 'd₄', 150, 'mm'], ['s1', 'Slip, first drive', 2, '%'], ['s2', 'Slip, second drive', 2, '%']],
  solve:v => { const r = v.d1*v.d3/(v.d2*v.d4); return [['N₄/N₁ = (d₁ × d₃)/(d₂ × d₄) =', n2(r, 3)], ['No slip: N₄ =', `${n2(v.N1*r, 1)} rpm`, true],
    ['With slip: N₄/N₁ = (d₁ d₃/d₂ d₄)(1 − s₁/100)(1 − s₂/100) =', n2(r*(1 - v.s1/100)*(1 - v.s2/100), 3)], ['N₄ =', `${n2(v.N1*r*(1 - v.s1/100)*(1 - v.s2/100), 1)} rpm`, true]]; } });
example($('ex2'), { title:'Example 11.2: speed lost to creep', src:'notes, Example 2 (Khurmi 11.2)',
  q:'Power is transmitted from a pulley 1 m in diameter at 200 rpm to a pulley 2.25 m in diameter by a belt. Find the speed lost by the driven pulley through creep, if the stress on the tight and slack sides is 1.4 MPa and 0.5 MPa and E for the belt is 100 MPa.',
  inputs:[['d1', 'd₁', 1, 'm'], ['N1', 'N₁', 200, 'rpm'], ['d2', 'd₂', 2.25, 'm'], ['s1', 'σ₁', 1.4, 'MPa'], ['s2', 'σ₂', 0.5, 'MPa'], ['E', 'E', 100, 'MPa']],
  solve:v => { const n0 = v.N1*v.d1/v.d2, E = v.E*1e6, f = (E + Math.sqrt(v.s2*1e6))/(E + Math.sqrt(v.s1*1e6)), n1 = n0*f;
    return [['Without creep N₂ = N₁ d₁/d₂ =', `${n2(n0, 4)} rpm`], ['√σ₁ and √σ₂, with σ in N/m² =', `${n2(Math.sqrt(v.s1*1e6), 1)} and ${n2(Math.sqrt(v.s2*1e6), 1)}`], ['(E + √σ₂)/(E + √σ₁) =', n2(f, 8)],
      ['With creep N₂ = N₁ (d₁/d₂)(E + √σ₂)/(E + √σ₁) =', `${n2(n1, 4)} rpm`], ['Speed lost to creep =', `${(n0 - n1).toFixed(5)} rpm`, true]]; } });

/* ================= 4.5 length and angle of contact ================= */
{ const P = {}, cv = $('bl'); let kind = 'open';
  controls($('bl_c'), [['r1', 'Larger radius r₁', 50, 400, 5, 250, 'mm'], ['r2', 'Smaller radius r₂', 20, 400, 5, 120, 'mm'], ['x', 'Centre distance x', 200, 2000, 10, 800, 'mm']], P, () => draw());
  function draw(){ const r1 = Math.max(P.r1, P.r2), r2 = Math.min(P.r1, P.r2), x = Math.max(P.x, (kind === 'cross' ? r1 + r2 : r1 - r2) + 10), cs = kind === 'open';
    const sa = (cs ? r1 - r2 : r1 + r2)/x, al = Math.asin(clamp(sa, -1, 1)), lp = loop2D([{ c:[0, 0], r:r1, s:1 }, { c:[x, 0], r:r2, s:cs ? 1 : -1 }]);
    const V = fitView(cv, { xl:-r1 - 50, xr:x + r2 + 50, yb:-r1 - 60, yt:r1 + 50 }), { ctx } = V, Pp = V.P;
    for (const [c, r, col] of [[[0, 0], r1, C.crank], [[x, 0], r2, C.vel]]){ ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.fillStyle = alpha(col, 0.1); ctx.beginPath(); ctx.arc(...Pp(c), r*V.sc, 0, 7); ctx.fill(); ctx.stroke(); pinAt(ctx, ...Pp(c), 4); }
    dashLine(ctx, ...Pp([-r1 - 30, 0]), ...Pp([x + r2 + 30, 0]), C.muted);
    ctx.strokeStyle = '#3b3f45'; ctx.lineWidth = 3.5; ctx.beginPath(); lp.pts.forEach((q, i) => i ? ctx.lineTo(...Pp(q)) : ctx.moveTo(...Pp(q))); ctx.closePath(); ctx.stroke();
    // tangent points (upper strand): E on pulley 1, F on pulley 2
    const T = lp.T; const up = T.find(q => q.pa[1] + q.pb[1] > 0 && q.pa[0] > q.pb[0]) || T[1], E = cs ? up.pb : up.pb, F = up.pa;
    const n1 = vmul(vsub(E, [0, 0]), 1/r1); dashLine(ctx, ...Pp([0, 0]), ...Pp(E), C.muted, [3, 3]); dashLine(ctx, ...Pp([x, 0]), ...Pp(F), C.muted, [3, 3]);
    // M: O2M parallel to FE meets O1E (produced)
    const dir = vsub(E, F), M = lineInt([x, 0], dir, [0, 0], n1); if (M){ dashLine(ctx, ...Pp([x, 0]), ...Pp(M), C.force, [5, 4]); dashLine(ctx, ...Pp([0, 0]), ...Pp(M), C.force, [2, 3]); lbl(ctx, 'M', ...vadd(Pp(M), [-10, -8]), C.force, 'right'); }
    for (const [q, n] of [[E, 'E'], [F, cs ? 'F' : 'H']]){ dotAt(ctx, ...Pp(q), C.acc, 4); lbl(ctx, n, Pp(q)[0] + 6, Pp(q)[1] - 12, C.acc); }
    lbl(ctx, 'O₁', Pp([0, 0])[0] - 8, Pp([0, 0])[1] + 14, C.ink, 'right'); lbl(ctx, 'O₂', Pp([x, 0])[0] + 8, Pp([x, 0])[1] + 14, C.ink);
    lbl(ctx, `α = ${n2(al/DEG, 2)}°`, Pp([x, 0])[0] - 60, Pp([x, 0])[1] - 10, C.force, 'right', 12);
    const th1 = PI + 2*al, th2 = cs ? PI - 2*al : PI + 2*al;
    const Lex = cs ? 2*Math.sqrt(x*x - (r1 - r2)**2) + r1*(PI + 2*al) + r2*(PI - 2*al) : 2*Math.sqrt(x*x - (r1 + r2)**2) + (r1 + r2)*(PI + 2*al);
    const Lap = PI*(r1 + r2) + 2*x + (cs ? (r1 - r2)**2 : (r1 + r2)**2)/x;
    $('bl_st').innerHTML = statsHTML([['sin α', `${cs ? '(r₁ − r₂)' : '(r₁ + r₂)'}/x = ${n2(sa, 4)}`], ['α', `${n2(al/DEG, 2)}°`], ['Arc on the larger pulley', `${n2(th1/DEG, 1)}° = ${n2(th1, 3)} <small>rad</small>`], ['Arc on the smaller pulley', `${n2(th2/DEG, 1)}° = ${n2(th2, 3)} <small>rad</small>`],
      ['Length by the formula', `${n2(Lap, 1)} <small>mm</small>`], ['Exact length from the geometry', `${n2(Lex, 1)} <small>mm</small>`], ['Formula error', `${n2(Lap - Lex, 2)} <small>mm</small>`]]);
    $('bl_sub').textContent = cs ? 'open belt: pulleys turn the same way' : 'crossed belt: pulleys turn opposite ways'; }
  function lineInt(p1, d1, p2, d2){ const D = d1[0]*(-d2[1]) - d1[1]*(-d2[0]); if (Math.abs(D) < 1e-12) return null; const r = vsub(p2, p1), t = (r[0]*(-d2[1]) - r[1]*(-d2[0]))/D; return vadd(p1, vmul(d1, t)); }
  segGroup($('bl_seg'), k => { kind = k; draw(); }); watchSize(cv, draw); onRedraw(draw); draw(); }

example($('ex3'), { title:'Example 11.3: stepped pulleys', src:'notes, Example 11.3 (Khurmi)',
  q:'A shaft turning at a constant 160 rpm drives a parallel shaft 720 mm away, which has to run at 60, 80 and 100 rpm. The smallest pulley on the driving shaft is 40 mm in radius. Find the other radii of the two stepped pulleys for (1) a crossed belt, (2) an open belt. Neglect belt thickness and slip.',
  inputs:[['N1', 'Driver speed', 160, 'rpm'], ['x', 'Centre distance x', 720, 'mm'], ['N2', 'N₂', 60, 'rpm'], ['N4', 'N₄', 80, 'rpm'], ['N6', 'N₆', 100, 'rpm'], ['r1', 'r₁', 40, 'mm']],
  solve:v => { const r2 = v.r1*v.N1/v.N2, S = v.r1 + r2, k4 = v.N1/v.N4, k6 = v.N1/v.N6, r3c = S/(1 + k4), r5c = S/(1 + k6);
    const L = PI*(v.r1 + r2) + (r2 - v.r1)**2/v.x + 2*v.x, solve = k => { const a = (k - 1)**2/v.x, b = PI*(1 + k), c = 2*v.x - L; return (-b + Math.sqrt(b*b - 4*a*c))/(2*a); }, r3o = solve(k4), r5o = solve(k6);
    return [['Pulleys 1 and 2: r₂ = r₁ N₁/N₂ =', `${n2(r2, 1)} mm`, true], ['Crossed belt: the length depends only on r₁ + r₂, so every pair sums to', `${n2(S, 1)} mm`],
      ['r₄ = r₃ N₃/N₄ = ' + n2(k4, 2) + ' r₃, so r₃ = sum/(1 + ' + n2(k4, 2) + ') =', `${n2(r3c, 1)} mm, r₄ = ${n2(k4*r3c, 1)} mm`, true], ['r₆ = ' + n2(k6, 2) + ' r₅, so r₅ =', `${n2(r5c, 1)} mm, r₆ = ${n2(k6*r5c, 1)} mm`, true],
      ['Open belt: r₂ is the same. L = π(r₁ + r₂) + (r₂ − r₁)²/x + 2x =', `${n2(L, 1)} mm`], ['Same L with r₄ = ' + n2(k4, 2) + ' r₃ gives a quadratic in r₃:', `r₃ = ${n2(r3o, 1)} mm, r₄ = ${n2(k4*r3o, 1)} mm`, true],
      ['and with r₆ = ' + n2(k6, 2) + ' r₅:', `r₅ = ${n2(r5o, 1)} mm, r₆ = ${n2(k6*r5o, 1)} mm`, true]]; } });

/* ================= 4.6 tension round the arc ================= */
{ const P = {}, cv = $('tr');
  controls($('tr_c'), [['mu', 'Coefficient of friction μ', 0.05, 0.6, 0.01, 0.25, ''], ['th', 'Angle of contact θ', 30, 300, 1, 160, '°'], ['T1', 'Tight side T₁', 100, 5000, 50, 2500, 'N'], ['v', 'Belt speed v', 1, 40, 0.1, 6.28, 'm/s']], P, () => draw());
  function draw(){ const th = P.th*DEG, ratio = Math.exp(P.mu*th), T2 = P.T1/ratio, R = 1, V = fitView(cv, { xl:-2.2, xr:2.6, yb:-1.8, yt:1.8 }), { ctx } = V, Pp = V.P;
    ctx.fillStyle = alpha(C.crank, 0.12); ctx.strokeStyle = C.crank; ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(...Pp([0, 0]), R*V.sc, 0, 7); ctx.fill(); ctx.stroke(); pinAt(ctx, ...Pp([0, 0]), 5);
    // contact arc symmetric about the left side; slack side enters at the top, tight side leaves at the bottom
    const aS = PI - th/2, aE = PI + th/2;   // belt arrives at aS (slack side, top) and runs anticlockwise round the left of the pulley to aE (tight side, bottom)
    const wmax = 16, w = T => 2 + wmax*T/P.T1;
    for (let i = 0; i < 60; i++){ const f0 = i/60, f1 = (i + 1)/60, b0 = aS + th*f0, b1 = aS + th*f1, T = T2*Math.exp(P.mu*th*(f0 + f1)/2);
      ctx.strokeStyle = '#3b3f45'; ctx.lineWidth = w(T); ctx.beginPath(); ctx.moveTo(...Pp([R*Math.cos(b0), R*Math.sin(b0)])); ctx.lineTo(...Pp([R*Math.cos(b1), R*Math.sin(b1)])); ctx.stroke(); }
    const tg = a => [-Math.sin(a), Math.cos(a)], pS = [R*Math.cos(aS), R*Math.sin(aS)], tS = tg(aS), pE = [R*Math.cos(aE), R*Math.sin(aE)], tE = tg(aE);
    ctx.lineWidth = w(T2); ctx.beginPath(); ctx.moveTo(...Pp(pS)); ctx.lineTo(...Pp(vsub(pS, vmul(tS, 1.6)))); ctx.stroke(); ctx.lineWidth = w(P.T1); ctx.beginPath(); ctx.moveTo(...Pp(pE)); ctx.lineTo(...Pp(vadd(pE, vmul(tE, 1.6)))); ctx.stroke();
    arrow(ctx, ...Pp(vsub(pS, vmul(tS, 1.0))), ...Pp(vsub(pS, vmul(tS, 1.7))), C.vel, 2.5); arrow(ctx, ...Pp(vadd(pE, vmul(tE, 1.0))), ...Pp(vadd(pE, vmul(tE, 1.7))), C.acc, 2.5);
    lbl(ctx, `T₂ = ${n2(T2, 0)} N (slack)`, ...vadd(Pp(vsub(pS, vmul(tS, 1.75))), [0, -14]), C.vel, 'center'); lbl(ctx, `T₁ = ${n2(P.T1, 0)} N (tight)`, ...vadd(Pp(vadd(pE, vmul(tE, 1.75))), [0, 16]), C.acc, 'center');
    const am = PI, pm = [Math.cos(am), Math.sin(am)], nrm = [Math.cos(am), Math.sin(am)], tm = tg(am);
    arrow(ctx, ...Pp(vadd(pm, vmul(nrm, 0.5))), ...Pp(vadd(pm, vmul(nrm, 0.05))), C.force, 2); lbl(ctx, 'R_N on the belt', ...vadd(Pp(vadd(pm, vmul(nrm, 0.62))), [0, -12]), C.force, 'center');
    arrow(ctx, ...Pp(vadd(pm, vmul(nrm, 0.12))), ...Pp(vadd(vadd(pm, vmul(nrm, 0.12)), vmul(tm, -0.45))), C.rod, 2); lbl(ctx, 'μR_N', ...vadd(Pp(vadd(vadd(pm, vmul(nrm, 0.12)), vmul(tm, -0.55))), [-14, 0]), C.rod, 'right');
    ctx.strokeStyle = alpha(C.muted, 0.6); ctx.lineWidth = 1; ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(...Pp([0, 0])); ctx.lineTo(...Pp(pS)); ctx.moveTo(...Pp([0, 0])); ctx.lineTo(...Pp(pE)); ctx.stroke(); ctx.setLineDash([]);
    lbl(ctx, `θ = ${P.th}°`, ...vadd(Pp([0, 0]), [12, 0]), C.ink);
    const X = [], Y = [], Y2 = []; for (let i = 0; i <= 100; i++){ const a = th*i/100; X.push(a/DEG); Y.push(T2*Math.exp(P.mu*a)); Y2.push(T2 + (P.T1 - T2)*i/100); }
    chart($('tr_g'), { x:X, series:[{ y:Y, color:C.acc, label:'T = T₂ e^μθ round the arc' }, { y:Y2, color:C.muted, dash:[4, 4], label:'straight line, for comparison' }], ylab:'T [N]', xlab:'angle from where the belt arrives [°]', value:`T₁/T₂ = ${n2(ratio, 3)}` });
    $('tr_st').innerHTML = statsHTML([['μθ', n2(P.mu*th, 4)], ['T₁/T₂ = e^μθ', n2(ratio, 3)], ['T₂', `${n2(T2, 0)} <small>N</small>`], ['Driving force T₁ − T₂', `${n2(P.T1 - T2, 0)} <small>N</small>`], ['Power (T₁ − T₂) v', `${n2((P.T1 - T2)*P.v/1000, 2)} <small>kW</small>`]]);
    $('tr_sub').textContent = 'belt thickness drawn in proportion to its tension'; }
  [cv, $('tr_g')].forEach(c => watchSize(c, draw)); onRedraw(draw); draw(); }

example($('ex4'), { title:'Example 11.4: power from the tensions', src:'notes, Example 11.4 (Khurmi)',
  q:'Find the power transmitted by a belt running over a pulley of 600 mm diameter at 200 rpm. The coefficient of friction between belt and pulley is 0.25, the angle of lap 160° and the maximum tension in the belt 2500 N.',
  inputs:[['d', 'Pulley diameter', 600, 'mm'], ['N', 'Speed', 200, 'rpm'], ['mu', 'μ', 0.25, ''], ['th', 'Angle of lap', 160, '°'], ['T1', 'T₁', 2500, 'N']],
  solve:v => { const vv = PI*v.d/1000*v.N/60, th = v.th*DEG, r = Math.exp(v.mu*th), T2 = v.T1/r;
    return [['v = π d N/60 =', `${n2(vv, 3)} m/s`], ['θ =', `${n2(th, 3)} rad`], ['2.3 log(T₁/T₂) = μθ =', n2(v.mu*th, 4)], ['T₁/T₂ =', n2(r, 3)], ['T₂ = T₁/(T₁/T₂) =', `${n2(T2, 0)} N`, true], ['P = (T₁ − T₂) v =', `${n2((v.T1 - T2)*vv/1000, 2)} kW`, true]]; } });
example($('ex5'), { title:'Example 11.6: crossed belt, length, contact and power', src:'notes, Example 11.6 (Khurmi)',
  q:'Two pulleys, 450 mm and 200 mm in diameter, are on parallel shafts 1.95 m apart and connected by a crossed belt. Find the length of belt needed and the angle of contact on each pulley. What power can it carry when the larger pulley turns at 200 rpm, if the maximum tension is 1 kN and μ = 0.25?',
  inputs:[['d1', 'd₁', 450, 'mm'], ['d2', 'd₂', 200, 'mm'], ['x', 'x', 1.95, 'm'], ['N1', 'N₁', 200, 'rpm'], ['T1', 'T₁', 1000, 'N'], ['mu', 'μ', 0.25, '']],
  solve:v => { const r1 = v.d1/2000, r2 = v.d2/2000, vv = PI*v.d1/1000*v.N1/60, L = PI*(r1 + r2) + 2*v.x + (r1 + r2)**2/v.x, sa = (r1 + r2)/v.x, al = Math.asin(sa), th = PI + 2*al, r = Math.exp(v.mu*th), T2 = v.T1/r;
    return [['v = π d₁ N₁/60 =', `${n2(vv, 3)} m/s`], ['L = π(r₁ + r₂) + 2x + (r₁ + r₂)²/x =', `${n2(L, 3)} m`, true], ['sin α = (r₁ + r₂)/x =', `${n2(sa, 4)}, α = ${n2(al/DEG, 2)}°`],
      ['θ = 180° + 2α =', `${n2(th/DEG, 1)}° = ${n2(th, 3)} rad, the same on both pulleys`, true], ['T₁/T₂ = e<sup>μθ</sup> =', n2(r, 3)], ['T₂ =', `${n2(T2, 0)} N`], ['P = (T₁ − T₂) v =', `${n2((v.T1 - T2)*vv/1000, 2)} kW`, true]]; } });

/* ================= 4.10 maximum power ================= */
{ const P = {};
  controls($('mp_c'), [['T', 'Maximum tension T', 200, 5000, 50, 1200, 'N'], ['m', 'Mass per metre m', 0.1, 3, 0.05, 0.6, 'kg/m'], ['mu', 'μ', 0.1, 0.6, 0.01, 0.3, ''], ['th', 'Angle of lap θ', 90, 270, 1, 120, '°']], P, () => draw());
  function draw(){ const Cc = 1 - Math.exp(-P.mu*P.th*DEG), vmax = Math.sqrt(P.T/P.m), vo = Math.sqrt(P.T/(3*P.m)), X = [], Y = [], Tc = [];
    for (let i = 0; i <= 300; i++){ const v = vmax*1.05*i/300; X.push(v); Y.push(Math.max(-1e9, (P.T - P.m*v*v)*Cc*v/1000)); Tc.push(P.m*v*v); }
    const Pm = (P.T - P.m*vo*vo)*Cc*vo/1000;
    chart($('mp'), { x:X, series:[{ y:Y, color:C.acc, label:'P = (T − m v²) C v [kW]' }], ylab:'P [kW]', xlab:'belt speed v [m/s]', markers:[{ x:vo, y:Pm, color:C.ink, label:`max ${n2(Pm, 2)} kW at ${n2(vo, 2)} m/s` }], vlines:[{ v:vo, color:C.ink, label:'v = √(T/3m)' }, { v:vmax, color:C.bad, label:'v = √(T/m): no power' }], zero:true });
    const T1 = P.T - P.T/3, T2 = T1*Math.exp(-P.mu*P.th*DEG);
    $('mp_st').innerHTML = statsHTML([['C = 1 − 1/e^μθ', n2(Cc, 4)], ['Speed for maximum power', `${n2(vo, 2)} <small>m/s</small>`], ['T_C = m v² = T/3', `${n2(P.T/3, 0)} <small>N</small>`], ['T₁ = 2T/3', `${n2(T1, 0)} <small>N</small>`], ['T₂', `${n2(T2, 0)} <small>N</small>`], ['Maximum power', `${n2(Pm, 2)} <small>kW</small>`], ['Initial tension (T₁ + T₂ + 2T_C)/2', `${n2((T1 + T2 + 2*P.T/3)/2, 0)} <small>N</small>`]]);
    $('mp_sub').textContent = 'centrifugal tension takes over as the belt speeds up'; }
  watchSize($('mp'), draw); onRedraw(draw); draw(); }

example($('ex6'), { title:'Example 11.10: greatest power of a flat belt', src:'notes, Example 11.10 (Khurmi)',
  q:'A pulley is driven by a flat belt, the angle of lap being 120°. The belt is 100 mm wide, 6 mm thick, of density 1000 kg/m³. If μ = 0.3 and the stress in the belt must not exceed 2 MPa, find the greatest power the belt can carry and the corresponding belt speed.',
  inputs:[['th', 'Angle of lap', 120, '°'], ['b', 'Width b', 100, 'mm'], ['t', 'Thickness t', 6, 'mm'], ['rho', 'Density ρ', 1000, 'kg/m³'], ['mu', 'μ', 0.3, ''], ['sig', 'Allowable stress σ', 2, 'MPa']],
  solve:v => { const T = v.sig*1e6*v.b/1000*v.t/1000, m = v.b/1000*v.t/1000*v.rho, vo = Math.sqrt(T/(3*m)), th = v.th*DEG, T1 = 2*T/3, r = Math.exp(v.mu*th), T2 = T1/r;
    return [['Maximum tension T = σ b t =', `${n2(T, 0)} N`], ['Mass per metre m = b t ρ =', `${n2(m, 3)} kg/m`], ['Speed for greatest power v = √(T/3m) =', `${n2(vo, 2)} m/s`, true], ['T_C = T/3 =', `${n2(T/3, 0)} N`], ['T₁ = T − T_C =', `${n2(T1, 0)} N`],
      ['θ =', `${n2(th, 4)} rad`], ['T₁/T₂ = e<sup>μθ</sup> =', n2(r, 3)], ['T₂ =', `${n2(T2, 1)} N`], ['P = (T₁ − T₂) v =', `${n2((T1 - T2)*vo/1000, 2)} kW`, true]]; } });

/* ================= 4.11 V-belt ================= */
{ const P = {};
  controls($('vb_c'), [['b', 'Half groove angle β', 10, 25, 0.5, 15, '°', v => `groove 2β = ${2*v}°`], ['mu', 'μ', 0.1, 0.5, 0.01, 0.15, ''], ['th', 'Angle of lap θ', 90, 220, 1, 140, '°']], P, () => draw());
  function draw(){ const b = P.b*DEG, V = fitView($('vb'), { xl:-1.6, xr:1.6, yb:-1.4, yt:1.3 }), { ctx } = V, Pp = V.P, h = 0.9, top = 0.55, tw = 0.55;
    // groove
    const gb = -1.2, gl = [-(top - gb)*Math.tan(b) - 0.05, top], gr = [(top - gb)*Math.tan(b) + 0.05, top];
    ctx.fillStyle = alpha(C.muted, 0.18); ctx.strokeStyle = C.muted; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(...Pp([-1.5, top])); ctx.lineTo(...Pp(gl)); ctx.lineTo(...Pp([-0.05, gb])); ctx.lineTo(...Pp([0.05, gb])); ctx.lineTo(...Pp(gr)); ctx.lineTo(...Pp([1.5, top])); ctx.lineTo(...Pp([1.5, -1.35])); ctx.lineTo(...Pp([-1.5, -1.35])); ctx.closePath(); ctx.fill(); ctx.stroke();
    // belt: trapezoid touching the sides, clear of the bottom
    const yt = top + 0.25, yb = yt - h, xt = (yt - gb)*Math.tan(b) + 0.05, xb = (yb - gb)*Math.tan(b) + 0.05;
    ctx.fillStyle = '#3b3f45'; ctx.beginPath(); ctx.moveTo(...Pp([-xt, yt])); ctx.lineTo(...Pp([xt, yt])); ctx.lineTo(...Pp([xb, yb])); ctx.lineTo(...Pp([-xb, yb])); ctx.closePath(); ctx.fill();
    lbl(ctx, `2β = ${n2(2*P.b, 0)}°`, ...Pp([0, gb - 0.12]), C.ink, 'center'); lbl(ctx, 'clearance', ...Pp([0.55, gb + 0.15]), C.muted, 'left', 11);
    // reactions R1 normal to each side, and R
    const ym = (yt + yb)/2, xm = (xt + xb)/2, nL = [Math.cos(b), Math.sin(b)];
    const pl = [-xm, ym], pr = [xm, ym]; arrow(ctx, ...Pp(pl), ...Pp(vadd(pl, vmul(nL, 0.7))), C.force, 2.5); arrow(ctx, ...Pp(pr), ...Pp(vadd(pr, vmul([-nL[0], nL[1]], 0.7))), C.force, 2.5);
    lbl(ctx, 'R₁', ...vadd(Pp(vadd(pl, vmul(nL, 0.8))), [0, 0]), C.force, 'center'); lbl(ctx, 'R₁', ...vadd(Pp(vadd(pr, vmul([-nL[0], nL[1]], 0.8))), [0, 0]), C.force, 'center');
    arrow(ctx, ...Pp([0, yt]), ...Pp([0, yt + 0.4]), C.acc, 2.8); lbl(ctx, 'R = 2R₁ sin β', ...vadd(Pp([0, yt + 0.48]), [0, 0]), C.acc, 'center');
    const cosec = 1/Math.sin(b), X = [], F = [], Vv = []; for (let i = 0; i <= 100; i++){ const a = 270*i/100*DEG; X.push(270*i/100); F.push(Math.exp(P.mu*a)); Vv.push(Math.exp(P.mu*a*cosec)); }
    chart($('vb_g'), { x:X, series:[{ y:Vv, color:C.acc, label:'V-belt: e^(μθ cosec β)' }, { y:F, color:C.crank, label:'flat belt: e^μθ' }], ylab:'T₁/T₂', xlab:'angle of lap θ [°]', vlines:[{ v:P.th, color:C.ink, label:`θ = ${P.th}°` }] });
    const th = P.th*DEG;
    $('vb_st').innerHTML = statsHTML([['cosec β', n2(cosec, 3)], ['Effective μ cosec β', n2(P.mu*cosec, 3)], ['Flat belt T₁/T₂', n2(Math.exp(P.mu*th), 3)], ['V-belt T₁/T₂', n2(Math.exp(P.mu*th*cosec), 3)], ['Friction force for the same R', `${n2(cosec, 2)} × a flat belt`]]);
    $('vb_sub').textContent = 'the belt wedges against the sides, not the bottom'; }
  ['vb', 'vb_g'].forEach(c => watchSize($(c), draw)); onRedraw(draw); draw(); }

example($('ex7'), { title:'Example 11.18: maximum power of a V-belt', src:'notes, Example 11.18 (Khurmi)',
  q:'Power is carried by a V-belt drive. The included angle of the groove is 30°, the belt is 20 mm deep with a maximum width of 20 mm, its mass is 0.35 kg per metre and the maximum allowable stress 1.4 MPa. Find the maximum power when the angle of lap is 140° and μ = 0.15.',
  inputs:[['g', 'Groove angle 2β', 30, '°'], ['t', 'Depth t', 20, 'mm'], ['b', 'Width b', 20, 'mm'], ['m', 'Mass m', 0.35, 'kg/m'], ['sig', 'Allowable stress', 1.4, 'MPa'], ['th', 'Angle of lap', 140, '°'], ['mu', 'μ', 0.15, '']],
  solve:v => { const T = v.sig*1e6*v.b/1000*v.t/1000, vo = Math.sqrt(T/(3*v.m)), be = v.g/2*DEG, th = v.th*DEG, x = v.mu*th/Math.sin(be), r = Math.exp(x), Tc = T/3, T1 = T - Tc, T2 = T1/r;
    return [['Maximum tension T = σ b t =', `${n2(T, 0)} N`], ['For maximum power v = √(T/3m) =', `${n2(vo, 2)} m/s`], ['2.3 log(T₁/T₂) = μ θ cosec β =', n2(x, 4)], ['T₁/T₂ =', n2(r, 3)], ['T_C = T/3 =', `${n2(Tc, 1)} N`], ['T₁ = T − T_C =', `${n2(T1, 1)} N`], ['T₂ =', `${n2(T2, 1)} N`],
      ['Maximum power P = (T₁ − T₂) v =', `${n2((T1 - T2)*vo/1000, 2)} kW`, true]]; } });

/* ================= quiz ================= */
quiz($('qz'), [
  { q:'A 600 mm pulley at 300 rpm drives a 200 mm pulley by an open belt, with no slip. The driven speed is…', o:['100 rpm', '600 rpm', '900 rpm', '1800 rpm'], a:2, why:'N₂ = N₁ d₁/d₂ = 300 × 600/200 = 900 rpm.' },
  { q:'With 3% total slip, the same drive gives about…', o:['873 rpm', '900 rpm', '927 rpm', '870 rpm'], a:0, why:'N₂ = 900 × (1 − 0.03) = 873 rpm.' },
  { q:'Which drive makes the two shafts turn in opposite directions?', o:['Open belt', 'Crossed belt', 'Idler pulley drive', 'Fast and loose pulley'], a:1, why:'In a crossed belt the strands cross, reversing the direction of the driven pulley.' },
  { q:'For stepped pulleys with a crossed belt, the belt fits every step if…', o:['r₁ − r₂ is the same for each pair', 'r₁ + r₂ is the same for each pair', 'r₁/r₂ is the same', 'the centre distance is large'], a:1, why:'The crossed-belt length depends only on r₁ + r₂, so equal sums give equal lengths.' },
  { q:'In an open belt drive, on which pulley is the angle of contact smaller, and what is it?', o:['The larger, 180° − 2α', 'The smaller, 180° − 2α', 'The smaller, 180° + 2α', 'Both are 180°'], a:1, why:'sin α = (r₁ − r₂)/x; the smaller pulley has the arc 180° − 2α, and it limits the drive.' },
  { q:'μ = 0.3 and θ = π rad. The ratio T₁/T₂ is about…', o:['1.35', '2.57', '3.14', '0.94'], a:1, why:'T₁/T₂ = e<sup>μθ</sup> = e<sup>0.942</sup> ≈ 2.57.' },
  { q:'For maximum power, the centrifugal tension equals…', o:['T/2', 'T/3', '2T/3', 'T'], a:1, why:'dP/dv = 0 gives T = 3mv², so T_C = mv² = T/3 and T₁ = 2T/3.' },
  { q:'A belt may carry T = 900 N and has m = 1 kg/m. Its speed for maximum power is…', o:['10 m/s', '17.3 m/s', '30 m/s', '54.8 m/s'], a:1, why:'v = √(T/3m) = √(900/3) = √300 ≈ 17.3 m/s.' },
  { q:'Why does a V-belt grip better than a flat belt with the same μ?', o:['It is heavier', 'The wedge action multiplies the normal reaction by cosec β', 'It runs faster', 'It has no creep'], a:1, why:'The friction is μR cosec β, so the ratio of tensions becomes e<sup>μθ cosec β</sup>.' },
  { q:'If T₁ = 1500 N and T₂ = 500 N, the initial tension (neglecting centrifugal tension) is…', o:['500 N', '1000 N', '1500 N', '2000 N'], a:1, why:'T₀ = (T₁ + T₂)/2 = 1000 N.' } ]);
})();
