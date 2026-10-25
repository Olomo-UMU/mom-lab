/* BME 2102 interactive notes, Chapter 2: kinematics of simple mechanisms */
(() => {
'use strict';
const PI = Math.PI, TAU = 2*PI;
const lbl = (ctx, t, x, y, col, al = 'left', size = 12) => label(ctx, t, x, y, col, al, size, true);
const LC = () => ({ 1:C.muted, 2:C.crank, 3:C.rod, 4:C.vel, 5:C.force, 6:C.acc, 7:C.mass, 8:C.crank, 9:C.rod, 10:C.vel, 11:C.force });
function tag(ctx, txt, x, y, col){ ctx.fillStyle = col; ctx.beginPath(); ctx.arc(x, y, 10, 0, 7); ctx.fill(); ctx.fillStyle = '#fff'; ctx.font = `600 11.5px ${C.mono}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(txt, x, y + 0.5); }
function arcM(ctx, Pp, c, r, a0, a1, col, w = 6){ ctx.strokeStyle = col; ctx.lineWidth = w; ctx.beginPath(); for (let i = 0; i <= 60; i++){ const a = a0 + (a1 - a0)*i/60, q = Pp([c[0] + r*Math.cos(a), c[1] + r*Math.sin(a)]); i ? ctx.lineTo(...q) : ctx.moveTo(...q); } ctx.stroke(); }
const mid = (p, q, k = 0.5) => [p[0] + (q[0] - p[0])*k, p[1] + (q[1] - p[1])*k];
function circX(P1, r1, P2, r2, br = 1){ const d = vsub(P2, P1), L = vlen(d); if (L > r1 + r2 || L < Math.abs(r1 - r2) || L === 0) return null;
  const k = (r1*r1 - r2*r2 + L*L)/(2*L), h = Math.sqrt(Math.max(0, r1*r1 - k*k)), u = vmul(d, 1/L); return vadd(vadd(P1, vmul(u, k)), vmul(vperp(u), br*h)); }
const G3 = () => THREE;
function ring3(ri, ro, h, m){ const T = G3(); const pts = [new T.Vector2(ri, -h/2), new T.Vector2(ro, -h/2), new T.Vector2(ro, h/2), new T.Vector2(ri, h/2), new T.Vector2(ri, -h/2)]; return new T.Mesh(new T.LatheGeometry(pts, 48), m); }

/* ================= 2.1 constrained motion (3D) ================= */
{ let mode = 'square', t = 0; const parts = {};
  const CAP = { square:'<b>Completely constrained.</b> The square bar can only slide along the hole, whichever way it is pushed: one definite motion.',
    collar:'<b>Completely constrained.</b> The collars stop the shaft sliding, so it can only turn in the hole.',
    round:'<b>Incompletely constrained.</b> A plain round shaft can turn, slide, or do both: the motion depends on how it is pushed.',
    foot:'' };
  const v = view3d($('cm3'), { cam:[2.2, 1.5, 2.6], target:[0, 0.1, 0], fov:34,
    build(v){ const T = THREE; readColors(); const steel = mat('#9aa6b2', { metalness:0.7, roughness:0.3 }), body = mat('#7f8c99', { metalness:0.2, roughness:0.7 }), mark = mat(C.acc), load = mat(C.rod, { metalness:0.2 });
      const base = new T.Mesh(new T.BoxGeometry(1.4, 0.06, 1.0), body); base.position.y = -0.62;
      // square hole housing
      const sq = new T.Group(); for (const [y, z, w, h] of [[0.27, 0, 0.5, 0.18], [-0.27, 0, 0.5, 0.18]]){ const b = new T.Mesh(new T.BoxGeometry(0.5, h, 0.72), body); b.position.set(0, y, z); sq.add(b); }
      for (const z of [0.27, -0.27]){ const b = new T.Mesh(new T.BoxGeometry(0.5, 0.36, 0.18), body); b.position.set(0, 0, z); sq.add(b); }
      const leg = new T.Mesh(new T.BoxGeometry(0.4, 0.3, 0.5), body); leg.position.y = -0.47; sq.add(leg);
      const bar = new T.Mesh(new T.BoxGeometry(1.9, 0.355, 0.355), steel); sq.add(bar); sq.add(base.clone()); parts.square = { g:sq, mv:bar };
      // round housing, used twice
      const house = () => { const g = new T.Group(); const r = ring3(0.16, 0.3, 0.5, body); r.rotation.z = PI/2; g.add(r); const l = new T.Mesh(new T.BoxGeometry(0.4, 0.32, 0.5), body); l.position.y = -0.46; g.add(l); g.add(base.clone()); return g; };
      const shaft = (collars) => { const s = new T.Group(); const c = new T.Mesh(new T.CylinderGeometry(0.155, 0.155, 1.9, 40), steel); c.rotation.z = PI/2; s.add(c);
        const m = new T.Mesh(new T.BoxGeometry(0.5, 0.03, 0.06), mark); m.position.set(0.62, 0.155, 0); s.add(m); const m2 = m.clone(); m2.position.x = -0.62; s.add(m2);
        if (collars) for (const x of [0.29, -0.29]){ const k = new T.Mesh(new T.CylinderGeometry(0.24, 0.24, 0.07, 40), steel); k.rotation.z = PI/2; k.position.x = x; s.add(k); } return s; };
      const cg = house(), cs = shaft(true); cg.add(cs); parts.collar = { g:cg, mv:cs };
      const rg = house(), rs = shaft(false); rg.add(rs); parts.round = { g:rg, mv:rs };
      // foot-step bearing: vertical shaft in a cup
      const fg = new T.Group(); const cup = ring3(0.16, 0.3, 0.4, body); cup.position.y = -0.3; fg.add(cup); const fl = new T.Mesh(new T.CylinderGeometry(0.3, 0.34, 0.1, 40), body); fl.position.y = -0.55; fg.add(fl); fg.add(base.clone());
      const fs = new T.Group(); const fc = new T.Mesh(new T.CylinderGeometry(0.155, 0.155, 1.1, 40), steel); fs.add(fc); const fm = new T.Mesh(new T.BoxGeometry(0.06, 0.4, 0.03), mark); fm.position.set(0, 0.25, 0.155); fs.add(fm);
      fs.position.y = 0.05; fg.add(fs); const wt = new T.Mesh(new T.BoxGeometry(0.5, 0.18, 0.5), load); fg.add(wt); parts.foot = { g:fg, mv:fs, wt };
      for (const k in parts){ parts[k].g.visible = k === mode; v.root.add(parts[k].g); } },
    update(dt){ t += dt; pose(); } });
  let roundState = { kind:0, tt:0 };
  function pose(){ if (!parts.square) return; const s = Math.sin(t*1.6);
    parts.square.mv.position.x = 0.35*s;
    parts.collar.mv.rotation.x = t*2;
    roundState.tt -= 1/60; if (roundState.tt <= 0){ roundState = { kind:(roundState.kind + 1) % 3, tt:2.2 }; }
    const rs = parts.round.mv; if (roundState.kind === 0) rs.rotation.x += 0.04; else if (roundState.kind === 1) rs.position.x = 0.3*Math.sin(t*2); else { rs.rotation.x += 0.04; rs.position.x = 0.3*Math.sin(t*2); }
    const f = parts.foot, ld = $('cm_load').checked; f.mv.rotation.y = t*2; f.mv.position.y = ld ? 0.05 : 0.05 + 0.22*Math.max(0, Math.sin(t*1.3)); f.wt.visible = ld; f.wt.position.y = f.mv.position.y + 0.64;
    const cap = mode === 'foot' ? (ld ? '<b>Successfully constrained.</b> On its own the shaft could turn or lift out of the bearing; the load on it stops the lift, so only turning is left.' : '<b>Incompletely constrained without the load.</b> The shaft can turn and also rise out of the cup. Tick the load to complete the constraint.') : mode === 'round' ? CAP.round + ` Now: ${['turning', 'sliding', 'turning and sliding'][roundState.kind]}.` : CAP[mode];
    if ($('cm_cap')._last !== cap){ $('cm_cap').innerHTML = cap; $('cm_cap')._last = cap; } }
  segGroup($('cm_seg'), m => { mode = m; for (const k in parts) parts[k].g.visible = k === m; $('cm_loadl').hidden = m !== 'foot'; pose(); if (v) v.render(); });
  $('cm_loadl').hidden = true; $('cm_load').addEventListener('change', () => { pose(); if (v) v.render(); });
  $('cm_sub').textContent = 'what motions can the pair allow?'; pose(); }

/* ================= 2.1.2 pairs (3D) ================= */
{ let mode = 'slide', t = 0; const P = {};
  const INFO = { slide:['Sliding', 'Lower (surface contact)', 'Self-closed', 'Piston and cylinder, cross-head and guides, shaper ram, tailstock on the lathe bed. Completely constrained.'],
    turn:['Turning', 'Lower (surface contact)', 'Self-closed', 'Shaft with collars in a bearing, crankshaft in a journal bearing, lathe spindle, cycle wheel on its axle. Completely constrained.'],
    roll:['Rolling', 'Higher (point contact at each ball)', 'Self-closed (held by the rings)', 'Ball and roller bearings.'],
    screw:['Screw', 'Lower (surface contact on the threads)', 'Self-closed', 'Lead screw of a lathe with its nut, bolt and nut. Turning and sliding are tied together by the lead.'],
    sphere:['Spherical', 'Lower (surface contact)', 'Self-closed', 'Ball and socket joint, car mirror mount, pen stand.'],
    cam:['Partly rolling, partly sliding', 'Higher (line contact)', 'Force-closed: a spring (or gravity) keeps the follower on the cam', 'Cam and follower; also toothed gears and belt drives are higher pairs.'] };
  const v = view3d($('kp3'), { cam:[1.9, 1.3, 2.3], target:[0, 0.1, 0], fov:34,
    build(v){ const T = THREE; readColors(); const steel = mat('#9aa6b2', { metalness:0.7, roughness:0.3 }), body = mat('#7f8c99', { metalness:0.2, roughness:0.7 }), blue = mat(C.crank), orange = mat(C.rod), mark = mat(C.acc);
      // sliding
      const sl = new T.Group(); const rail = new T.Mesh(new T.BoxGeometry(2, 0.12, 0.3), body); sl.add(rail); const blk = new T.Mesh(new T.BoxGeometry(0.4, 0.3, 0.42), blue); blk.position.y = 0.06; sl.add(blk);
      for (const z of [0.19, -0.19]){ const lip = new T.Mesh(new T.BoxGeometry(0.4, 0.06, 0.04), blue); lip.position.set(0, -0.07, z); blk.add(lip); } P.slide = { g:sl, a:blk };
      // turning
      const tu = new T.Group(); const r = ring3(0.16, 0.32, 0.45, body); r.rotation.z = PI/2; tu.add(r); const ped = new T.Mesh(new T.BoxGeometry(0.4, 0.4, 0.55), body); ped.position.y = -0.48; tu.add(ped);
      const sh = new T.Group(); const c = new T.Mesh(new T.CylinderGeometry(0.155, 0.155, 1.6, 40), steel); c.rotation.z = PI/2; sh.add(c); for (const x of [0.27, -0.27]){ const k = new T.Mesh(new T.CylinderGeometry(0.24, 0.24, 0.06, 40), blue); k.rotation.z = PI/2; k.position.x = x; sh.add(k); }
      const mk = new T.Mesh(new T.BoxGeometry(0.4, 0.03, 0.05), mark); mk.position.set(0.55, 0.155, 0); sh.add(mk); tu.add(sh); P.turn = { g:tu, a:sh };
      // rolling: ball bearing
      const ro = new T.Group(); const outer = ring3(0.46, 0.6, 0.2, body); outer.rotation.x = PI/2; ro.add(outer); const inner = ring3(0.18, 0.3, 0.2, blue); inner.rotation.x = PI/2; ro.add(inner);
      const im = new T.Mesh(new T.BoxGeometry(0.04, 0.08, 0.21), mark); im.position.set(0, 0.25, 0); inner.add(im); im.rotation.x = -PI/2; im.position.set(0, 0, -0.25);
      const cage = new T.Group(); for (let i = 0; i < 9; i++){ const b = new T.Mesh(new T.SphereGeometry(0.08, 20, 14), steel); const a = i/9*TAU; b.position.set(0.38*Math.cos(a), 0.38*Math.sin(a), 0); cage.add(b); } ro.add(cage); P.roll = { g:ro, a:inner, b:cage };
      // screw
      const sc = new T.Group(); const core = new T.Mesh(new T.CylinderGeometry(0.12, 0.12, 1.9, 32), steel); core.rotation.z = PI/2; sc.add(core);
      class Helix extends T.Curve { getPoint(u){ const a = u*TAU*14; return new T.Vector3(-0.95 + 1.9*u, 0.14*Math.cos(a), 0.14*Math.sin(a)); } }
      const th = new T.Mesh(new T.TubeGeometry(new Helix(), 700, 0.028, 6, false), steel); sc.add(th);
      const nut = new T.Mesh(new T.CylinderGeometry(0.3, 0.3, 0.26, 6), blue); nut.rotation.z = PI/2; const ng = new T.Group(); ng.add(nut); const nm = new T.Mesh(new T.BoxGeometry(0.27, 0.05, 0.05), mark); nm.position.set(0, 0.27, 0); ng.add(nm); sc.add(ng); P.screw = { g:sc, a:ng };
      // spherical
      const sp = new T.Group(); const sock = new T.Mesh(new T.SphereGeometry(0.3, 40, 20, 0, TAU, PI*0.45, PI*0.55), body); sock.material = body.clone(); sock.material.side = T.DoubleSide; sp.add(sock);
      const stem = new T.Mesh(new T.CylinderGeometry(0.1, 0.1, 0.5, 20), body); stem.position.y = -0.52; sp.add(stem);
      const arm = new T.Group(); const ball = new T.Mesh(new T.SphereGeometry(0.26, 32, 20), blue); arm.add(ball); const rod = new T.Mesh(new T.CylinderGeometry(0.06, 0.06, 0.9, 16), steel); rod.position.y = 0.5; arm.add(rod);
      const pad = new T.Mesh(new T.BoxGeometry(0.5, 0.05, 0.3), orange); pad.position.y = 0.95; arm.add(pad); sp.add(arm); P.sphere = { g:sp, a:arm };
      // cam and follower
      const cm = new T.Group(); const camShape = new T.Shape(); for (let i = 0; i <= 64; i++){ const a = i/64*TAU, rr = 0.3 + 0.16*Math.max(0, Math.cos(a))**2; const x = rr*Math.cos(a), y = rr*Math.sin(a); i ? camShape.lineTo(x, y) : camShape.moveTo(x, y); }
      const cam = new T.Mesh(new T.ExtrudeGeometry(camShape, { depth:0.2, bevelEnabled:false }), blue); cam.position.z = -0.1; const cg = new T.Group(); cg.add(cam); cg.position.y = -0.3; cm.add(cg);
      const fol = new T.Group(); const fr = new T.Mesh(new T.CylinderGeometry(0.05, 0.05, 0.9, 16), steel); fr.position.y = 0.45; fol.add(fr); const ff = new T.Mesh(new T.BoxGeometry(0.42, 0.04, 0.24), orange); fol.add(ff); cm.add(fol);
      const guide = new T.Mesh(new T.BoxGeometry(0.16, 0.18, 0.22), body); guide.position.y = 0.55; cm.add(guide);
      class Spring extends T.Curve { constructor(){ super(); this.h = 0.3; } getPoint(u){ const a = u*TAU*7; return new T.Vector3(0.1*Math.cos(a), 0.66 + u*this.h, 0.1*Math.sin(a)); } }
      const spc = new Spring(); const spm = new T.Mesh(new T.TubeGeometry(spc, 200, 0.012, 6, false), mat(C.acc)); cm.add(spm); const top = new T.Mesh(new T.BoxGeometry(0.4, 0.04, 0.24), body); top.position.y = 1.02; cm.add(top);
      P.cam = { g:cm, a:cg, b:fol, s:spm, sc:spc };
      for (const k in P){ P[k].g.visible = k === mode; v.root.add(P[k].g); } },
    update(dt){ t += dt; pose(); } });
  function pose(){ if (!P.slide) return;
    P.slide.a.position.x = 0.65*Math.sin(t*1.4); P.turn.a.rotation.x = t*2.2; P.roll.a.rotation.z = t*2.4; P.roll.b.rotation.z = t*0.9;
    const turn = t*3; P.screw.a.rotation.x = turn; P.screw.a.position.x = -0.6 + ((turn/TAU*0.136) % 1.2 + 1.2) % 1.2;
    P.sphere.a.rotation.set(0.45*Math.sin(t*1.3), t*0.8, 0.45*Math.cos(t*1.3));
    const a = t*1.8; P.cam.a.rotation.z = a; let lift = 0; for (let i = 0; i < 90; i++){ const q = i/90*TAU, rr = 0.3 + 0.16*Math.max(0, Math.cos(q))**2; lift = Math.max(lift, rr*Math.sin(q + a)); }
    P.cam.b.position.y = -0.3 + lift + 0.02; const sp = P.cam.s; sp.scale.y = 1; const k = (1.0 - (P.cam.b.position.y + 0.9 + 0.02))/0.3; }
  function table(){ const [m, c, cl, ex] = INFO[mode]; $('kp_t').innerHTML = `<tr><th>Relative motion</th><td>${m}</td></tr><tr><th>Contact</th><td>${c}</td></tr><tr><th>Closure</th><td>${cl}</td></tr><tr><th>Examples</th><td>${ex}</td></tr>`; }
  segGroup($('kp_seg'), m => { mode = m; for (const k in P) P[k].g.visible = k === m; table(); pose(); if (v) v.render(); });
  $('kp_sub').textContent = 'choose a pair'; table(); }

/* ================= 2.1.3 chains (Figure 2.3) ================= */
{ const cv = $('ch'); let n = '3', t = 0;
  const DATA = { 3:{ l:3, p:3, j:3 }, 4:{ l:4, p:4, j:4 }, 5:{ l:5, p:5, j:5 }, 6:{ l:6, p:5, j:7 } };
  function geom(){ const A = [0, 0], B = [160, 0], L = LC();
    if (n === '3'){ const jit = [Math.sin(t*37)*0.8, Math.cos(t*29)*0.8]; return { links:[[1, [A, B]], [2, [B, [80, 125]]], [3, [[80, 125], A]]], pins:{ A, B, C:vadd([80, 125], jit) }, ground:[A, B] }; }
    if (n === '4'){ const r = fourbar(A, B, 60, 150, 110, PI*0.6 + t*1.2, 1); const D = r.A, Cc = r.B; return { links:[[1, [A, B]], [2, [B, Cc]], [3, [Cc, D]], [4, [D, A]]], pins:{ A, B, C:Cc, D }, ground:[A, B] }; }
    if (n === '5'){ const E = vadd(A, vmul([Math.cos(1.9 + 0.5*Math.sin(t*1.1)), Math.sin(1.9 + 0.5*Math.sin(t*1.1))], 80)), Cc = vadd(B, vmul([Math.cos(1.3 + 0.5*Math.sin(t*1.7 + 1)), Math.sin(1.3 + 0.5*Math.sin(t*1.7 + 1))], 80));
      const D = circX(E, 95, Cc, 95, -1) || mid(E, Cc); return { links:[[1, [A, B]], [2, [B, Cc]], [3, [Cc, D]], [4, [D, E]], [5, [E, A]]], pins:{ A, B, C:Cc, D, E }, ground:[A, B] }; }
    const Ee = [250, 0], r = fourbar(A, B, 55, 150, 115, PI*0.62 + 0.9*Math.sin(t*1.1), 1), D = r.A, Cc = r.B, Gp = mid(B, Cc, 0.55), F = circX(Gp, 100, Ee, 110, -1) || Gp;
    return { links:[[1, [A, Ee]], [2, [B, Cc]], [3, [Cc, D]], [4, [D, A]], [5, [F, Ee]], [6, [Gp, F]]], pins:{ A, B, C:Cc, D, E:Ee, F, G:Gp }, ground:[A, Ee] }; }
  function draw(){ const g = geom(), L = LC(), V = fitView(cv, { xl:-60, xr:300, yb:-40, yt:190 }), { ctx } = V, P = V.P;
    ctx.strokeStyle = C.muted; ctx.lineWidth = 1; groundAt(ctx, ...P(mid(g.ground[0], g.ground[1])), (g.ground[1][0] - g.ground[0][0])*V.sc + 40, 0);
    for (const [k, [p, q]] of g.links){ if (k === 1) continue; barAt(ctx, ...P(p), ...P(q), L[k], 6); tag(ctx, String(k), ...vadd(P(mid(p, q)), [0, 0]), L[k]); }
    barAt(ctx, ...vadd(P(g.ground[0]), [0, 4]), ...vadd(P(g.ground[1]), [0, 4]), alpha(C.muted, 0.6), 4);
    for (const [nm, p] of Object.entries(g.pins)){ pinAt(ctx, ...P(p)); lbl(ctx, nm, P(p)[0] + 8, P(p)[1] - 12, C.ink); }
    const d = DATA[n], lr = 2*d.p - 4, jr = 1.5*d.l - 2, cmp = (a, b) => a > b ? '>' : a < b ? '<' : '=';
    const verdict = d.j > jr ? 'locked chain: a structure' : d.j === jr ? 'kinematic chain: one input moves it all' : 'unconstrained chain: the motion is not definite';
    const col = d.j > jr ? C.bad : d.j === jr ? C.ok : C.rod;
    lbl(ctx, verdict, 8, 14, col, 'left', 13);
    $('ch_st').innerHTML = statsHTML([['Links l, pairs p, joints j', `${d.l}, ${d.p}, ${d.j}`], ['l and 2p − 4', `${d.l} ${cmp(d.l, lr)} ${lr}`], ['j and (3/2)l − 2', `${d.j} ${cmp(d.j, jr)} ${jr}`], ['Result', verdict.split(':')[0]]]);
    $('ch_sub').textContent = n === '5' ? 'two inputs needed: the angles at A and at B change independently' : n === '3' ? 'nothing can move' : 'driven by one link'; }
  segGroup($('ch_seg'), v => { n = v; draw(); }); watchSize(cv, draw); onRedraw(draw); animate(cv, dt => { t += dt; draw(); }); draw(); }

/* Figure 2.4: joints */
{ const cv = $('jt'); let sel = 'C'; let V;
  const PTS = { A:[0, 0], B:[188, -1], C:[137, 40], D:[113, 232], E:[69, 147], F:[108, 140], G:[30, 101] };
  const LINKS = [[1, 'A', 'B'], [2, 'B', 'C'], [3, 'A', 'C'], [4, 'B', 'D'], [5, 'D', 'E'], [6, 'E', 'F'], [7, 'E', 'G'], [8, 'G', 'F'], [9, 'F', 'C'], [10, 'G', 'C'], [11, 'A', 'G']];
  const at = j => LINKS.filter(l => l[1] === j || l[2] === j).map(l => l[0]);
  const KIND = n => ['', '', 'binary', 'ternary', 'quaternary', 'quinary'][n];
  function draw(){ V = fitView(cv, { xl:-40, xr:230, yb:-25, yt:255 }); const { ctx } = V, P = V.P, L = LC(), on = at(sel);
    for (const [k, a, b] of LINKS){ const hl = on.includes(k); barAt(ctx, ...P(PTS[a]), ...P(PTS[b]), hl ? L[k] : alpha(C.muted, 0.45), hl ? 6 : 3.5);
      const m = P(mid(PTS[a], PTS[b])); if (hl) tag(ctx, String(k), ...m, L[k]); else lbl(ctx, String(k), m[0], m[1], C.muted, 'center', 11); }
    for (const [nm, p] of Object.entries(PTS)){ const q = P(p), n = at(nm).length; ctx.fillStyle = nm === sel ? C.ink : C.sheet; ctx.strokeStyle = C.ink; ctx.lineWidth = 1.6; ctx.beginPath(); ctx.arc(q[0], q[1], nm === sel ? 7 : 5, 0, 7); ctx.fill(); ctx.stroke();
      lbl(ctx, `${nm} (${n - 1})`, q[0] + 10, q[1] - 12, nm === sel ? C.ink : C.muted, 'left', 12); }
    const sum = Object.keys(PTS).reduce((s, j) => s + at(j).length - 1, 0), rhs = 1.5*11 - 2;
    $('jt_st').innerHTML = statsHTML([[`Joint ${sel}`, `${KIND(on.length)}: links ${on.join(', ')}`], ['Counts as binary joints', `${on.length - 1}`], ['Total j (binary-equivalent)', `${sum}`], ['(3/2)l − 2 with l = 11', `${rhs}`], ['Result', `${sum} > ${rhs}: locked chain`, 'bad']]);
    $('jt_sub').textContent = 'number in brackets: binary joints it counts as'; }
  dragOn(cv, (x, y) => { if (!V) return null; for (const [nm, p] of Object.entries(PTS)) if (Math.hypot(x - V.PX(p[0]), y - V.PY(p[1])) < 16) return nm; return null; }, () => {}, h => { sel = h; draw(); });
  cv.addEventListener('pointerdown', e => { const r = cv.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top; for (const [nm, p] of Object.entries(PTS)) if (Math.hypot(x - V.PX(p[0]), y - V.PY(p[1])) < 16){ sel = nm; draw(); } });
  watchSize(cv, draw); onRedraw(draw); draw(); }

/* ================= 2.2 degrees of freedom as joints are added (Figure 2.6) ================= */
{ const cv = $('df'); let stage = 0, t = 0;
  const A = [0, 0], B = [170, 0], a = 70, b = 150, c = 110;
  function draw(){ const L = LC(), V = fitView(cv, { xl:-70, xr:260, yb:-35, yt:200 }), { ctx } = V, P = V.P;
    const base = fourbar(A, B, a, b, c, PI*0.55, 1), D0 = base.A, C0 = base.B;
    const ang = (p, q) => Math.atan2(q[1] - p[1], q[0] - p[0]), wob = (k, s = 1) => s*Math.sin(t*(0.9 + 0.23*k) + k);
    let l4, l3, l2, joins = [];
    if (stage === 4){ const r = fourbar(A, B, a, b, c, PI*0.55 + t*1.2, 1); l4 = [A, r.A]; l3 = [r.A, r.B]; l2 = [B, r.B]; joins = ['A', 'D', 'B', 'C']; }
    else { const fl = (p0, ph0, len, k, pinned) => { const p = pinned ? p0 : vadd(p0, [14*wob(k), 10*wob(k + 3)]), ph = ph0 + 0.22*wob(k + 7); return [p, vadd(p, vmul([Math.cos(ph), Math.sin(ph)], len))]; };
      l4 = fl(A, ang(A, D0), a, 1, stage >= 1); if (stage >= 1) joins.push('A');
      l3 = stage >= 2 ? fl(l4[1], ang(D0, C0), b, 2, true) : fl(vadd(D0, [-6, 18]), ang(D0, C0), b, 2, false); if (stage >= 2) joins.push('D');
      l2 = stage >= 3 ? fl(B, ang(B, C0), c, 3, true) : fl(vadd(B, [28, 10]), ang(B, C0), c, 3, false); if (stage >= 3) joins.push('B'); }
    groundAt(ctx, ...P(mid(A, B)), (B[0] - A[0])*V.sc + 60, 0); barAt(ctx, ...vadd(P(A), [0, 4]), ...vadd(P(B), [0, 4]), alpha(C.muted, 0.6), 4);
    for (const [k, [p, q]] of [[4, l4], [3, l3], [2, l2]]){ barAt(ctx, ...P(p), ...P(q), L[k], 6); tag(ctx, String(k), ...P(mid(p, q)), L[k]); for (const e of [p, q]) dotAt(ctx, ...P(e), L[k], 4.5); }
    const pins = { A, B, D:l4[1], C:l3[1] }; for (const j of joins) pinAt(ctx, ...P(pins[j]), 6);
    lbl(ctx, '1 (fixed)', ...vadd(P(mid(A, B)), [0, 22]), C.muted, 'center');
    const n = 3*3 - 2*stage;
    $('df_st').innerHTML = statsHTML([['Moving links (l − 1)', '3'], ['Freedom before joining 3(l − 1)', '9'], ['Turning pairs j', String(stage)], ['n = 3(l − 1) − 2j', String(n), n === 1 ? '' : ''], ['Inputs needed', n === 1 ? '1: a constrained mechanism' : String(n)]]);
    $('df_sub').textContent = stage === 4 ? 'n = 1: one input (link 4) drives the rest' : `n = ${n}: each pin takes away 2`; }
  segGroup($('df_seg'), v => { stage = +v; draw(); }); watchSize(cv, draw); onRedraw(draw); const an = animate(cv, dt => { t += dt; draw(); }); playButton($('df_play'), an); draw(); }

/* Kutzbach example mechanisms */
{ const cv = $('kz'); let m = 'a', t = 0;
  const INFO = { a:{ l:3, j:3, h:0, w:'n = 0: a structure. Three links pinned together cannot move.' }, b:{ l:4, j:4, h:0, w:'n = 1: a constrained mechanism. One input (link 4 or link 2) fixes the motion.' },
    c:{ l:5, j:5, h:0, w:'n = 2: two inputs are needed, for example link 5 and link 2 turned independently.' },
    d:{ l:5, j:6, h:0, w:'n = 0: a structure. Joints A and C are ternary and count as 2 each, so j = 6.' },
    e:{ l:6, j:8, h:0, w:'n = −1: a statically indeterminate (redundant) structure: one link more than needed to make it rigid. Each corner is a ternary joint.' } };
  function draw(){ const L = LC(), V = fitView(cv, { xl:-50, xr:220, yb:-35, yt:190 }), { ctx } = V, P = V.P, A = [0, 0], B = [150, 0]; let links = [], pins = {};
    if (m === 'a'){ const Cc = [75, 120]; links = [[2, B, Cc], [3, Cc, A]]; pins = { A, B, C:Cc }; }
    if (m === 'b'){ const r = fourbar(A, B, 60, 140, 110, PI*0.6 + t*1.2, 1); links = [[4, A, r.A], [3, r.A, r.B], [2, B, r.B]]; pins = { A, B, C:r.B, D:r.A }; }
    if (m === 'c'){ const E = vadd(A, vmul([Math.cos(1.9 + 0.5*Math.sin(t*1.1)), Math.sin(1.9 + 0.5*Math.sin(t*1.1))], 75)), Cc = vadd(B, vmul([Math.cos(1.25 + 0.5*Math.sin(t*1.7 + 1)), Math.sin(1.25 + 0.5*Math.sin(t*1.7 + 1))], 80));
      const D = circX(E, 90, Cc, 90, -1) || mid(E, Cc); links = [[5, A, E], [4, E, D], [3, D, Cc], [2, Cc, B]]; pins = { A, B, C:Cc, D, E }; }
    if (m === 'd'){ const Cc = [150, 140], D = [20, 120]; links = [[4, A, D], [3, D, Cc], [2, B, Cc], [5, A, Cc]]; pins = { A, B, C:Cc, D }; }
    if (m === 'e'){ const Cc = [150, 140], D = [0, 140]; links = [[4, A, D], [3, D, Cc], [2, B, Cc], [5, B, D], [6, A, Cc]]; pins = { A, B, C:Cc, D }; }
    groundAt(ctx, ...P(mid(A, B)), 150*V.sc + 40, 0); barAt(ctx, ...vadd(P(A), [0, 4]), ...vadd(P(B), [0, 4]), alpha(C.muted, 0.6), 4);
    for (const [k, p, q] of links){ barAt(ctx, ...P(p), ...P(q), L[k], 6); tag(ctx, String(k), ...P(mid(p, q, k === 5 || k === 6 ? 0.3 : 0.5)), L[k]); }
    for (const [nm, p] of Object.entries(pins)){ pinAt(ctx, ...P(p)); lbl(ctx, nm, P(p)[0] + 8, P(p)[1] - 12, C.ink); }
    tag(ctx, '1', ...vadd(P(mid(A, B)), [0, 20]), C.muted);
    const d = INFO[m], n = 3*(d.l - 1) - 2*d.j - d.h;
    $('kz_st').innerHTML = statsHTML([['Links l', d.l], ['Binary joints j', d.j], ['Higher pairs h', d.h], ['n = 3(l − 1) − 2j − h', n, n === 1 ? '' : n < 0 ? 'bad' : ''], ['Meaning', d.w.split(':')[0].replace(/^n = -?\d+/, '').trim() || d.w]]);
    $('kz_sub').textContent = d.w; }
  segGroup($('kz_seg'), v => { m = v; draw(); }); watchSize(cv, draw); onRedraw(draw); animate(cv, dt => { t += dt; if (m === 'b' || m === 'c') draw(); }); draw(); }

example($('ex1'), { title:'Kutzbach criterion', src:'example in the notes, section 2.2.2',
  q:'Find the degrees of freedom of a plane mechanism and say what they mean. The numbers start with the four-bar (b); try (a) 3, 3, 0; (c) 5, 5, 0; (d) 5, 6, 0; (e) 6, 8, 0.',
  inputs:[['l', 'Links l', 4], ['j', 'Binary joints j', 4], ['h', 'Higher pairs h', 0]],
  solve:v => { const n = 3*(v.l - 1) - 2*v.j - v.h;
    return [['Moving links start with 3(l − 1) =', `${3*(v.l - 1)}`], ['Lower pairs remove 2j =', `${2*v.j}`], ['Higher pairs remove h =', `${v.h}`], ['n = 3(l − 1) − 2j − h =', `${n}`, true],
      [n < 0 ? 'n < 0: a statically indeterminate structure (redundant link).' : n === 0 ? 'n = 0: a structure; no motion is possible.' : n === 1 ? 'n = 1: a constrained mechanism; one input defines the motion.' : `n = ${n}: ${n} independent inputs are needed for definite motion.`, '']]; } });

/* ================= 2.2.3 four-bar chain, Grashof, inversions ================= */
{ const cv = $('fb'), P = {}; let fix = 0, ph = 0, trace = [], V, H;
  const NAMES = ['AB', 'BC', 'CD', 'DA'], JN = ['A', 'B', 'C', 'D'];
  function roles(L){ const k = fix, s = Math.min(...L), lmax = Math.max(...L), sum = L.reduce((a, b) => a + b, 0), gr = s + lmax <= sum - s - lmax + 1e-9, si = L.indexOf(s);
    const km = (k + 3) % 4, kp = (k + 1) % 4, kc = (k + 2) % 4; let input = km, type;
    if (!gr) type = 'triple rocker (non-Grashof): no link turns fully';
    else if (Math.abs(s + lmax - (sum - s - lmax)) < 1e-9) type = 'change point: s + l = p + q, the links can line up';
    else if (si === k) type = 'double crank: both side links turn fully';
    else if (si === kc) type = 'double rocker: the coupler turns fully, the side links rock';
    else { type = 'crank and rocker'; input = si; }
    return { gr, type, input, other:input === km ? kp : km, coupler:kc, s, lmax, sum }; }
  function solve(L, r, th){ const k = fix, Jk = [0, 0], Jk1 = [L[k], 0]; const fromK = r.input === (k + 3) % 4;
    const Pp = fromK ? Jk : Jk1, Q = fromK ? Jk1 : Jk, a = L[r.input], b = L[r.coupler], c = L[r.other];
    const sgn = fromK ? 1 : -1; const A = vadd(Pp, vmul([Math.cos(th), Math.sin(th)], a));
    let Bq; const para = Math.abs(a - c) < 1e-6 && Math.abs(b - L[k]) < 1e-6;
    if (para) Bq = vadd(Q, vsub(A, Pp)); else { const fbr = fourbar(Pp, Q, a, b, c, th, sgn); if (!fbr) return null; Bq = fbr.B; }
    const J = []; J[k] = Jk; J[(k + 1) % 4] = Jk1; if (fromK){ J[(k + 3) % 4] = A; J[(k + 2) % 4] = Bq; } else { J[(k + 2) % 4] = A; J[(k + 3) % 4] = Bq; } return J; }
  let range = null;
  function prepare(){ const L = [P.L1, P.L2, P.L3, P.L4], r = roles(L); const N = 720, ok = [];
    for (let i = 0; i < N; i++) ok.push(!!solve(L, r, i/N*TAU));
    if (ok.every(Boolean)) range = { full:true };
    else { let best = null; for (let i = 0; i < N; i++){ if (!ok[i] || ok[(i + N - 1) % N]) continue; let j = i; while (ok[(j + 1) % N] && j - i < N) j++; if (!best || j - i > best[1] - best[0]) best = [i, j]; }
      range = best ? { full:false, a:(best[0] + 0.5)/N*TAU, b:(best[1] - 0.5)/N*TAU } : null; }
    trace = []; let bb = [1e9, -1e9, 1e9, -1e9]; const grow = q => { bb = [Math.min(bb[0], q[0]), Math.max(bb[1], q[0]), Math.min(bb[2], q[1]), Math.max(bb[3], q[1])]; };
    if (range) for (let i = 0; i <= 240; i++){ const J = solve(L, r, angle(i/240*TAU)); if (J){ const c = cpt(J, r); trace.push(c); grow(c); J.forEach(grow); } }
    const pad = 30 + (cv._preset === 'loco' ? Math.max(L[1], L[3])*1.5 : 0); H = { L, r, box:{ xl:bb[0] - pad, xr:bb[1] + pad, yb:bb[2] - pad - 10, yt:bb[3] + pad } }; }
  const angle = p => !range ? 0 : range.full ? p : (range.a + range.b)/2 + (range.b - range.a)/2*Math.sin(p);
  const cpt = (J, r) => { const p = J[r.coupler], q = J[(r.coupler + 1) % 4], u = vsub(q, p), n = vperp(u); return vadd(vadd(p, vmul(u, 0.5)), vmul(n, 0.35)); };
  function draw(){ if (!H) prepare(); const { L, r } = H, k = fix; const Lc = LC(), COL = [Lc[1], Lc[2], Lc[3], Lc[4]];
    V = fitView(cv, range ? H.box : { xl:-100, xr:100, yb:-100, yt:100 }); const { ctx } = V, Pp = V.P;
    if (!range){ lbl(ctx, 'These lengths cannot be assembled: one link is longer than the other three together.', 10, 16, C.bad, 'left', 13);
      $('fb_st').innerHTML = statsHTML([['Assembly', 'impossible', 'bad']]); return; }
    const J = solve(L, r, angle(ph)); if (!J) return;
    if (trace.length > 2){ ctx.strokeStyle = alpha(C.force, 0.55); ctx.lineWidth = 1.3; ctx.setLineDash([3, 3]); ctx.beginPath(); trace.forEach((q, i) => i ? ctx.lineTo(...Pp(q)) : ctx.moveTo(...Pp(q))); ctx.stroke(); ctx.setLineDash([]); }
    const loco = $('fb')._preset === 'loco' && k === 0; if (loco){ for (const q of [J[0], J[1]]){ ctx.strokeStyle = alpha(C.muted, 0.8); ctx.lineWidth = 3; ctx.beginPath(); ctx.arc(...Pp(q), L[3]*1.45*V.sc, 0, 7); ctx.stroke(); ctx.lineWidth = 1.2; ctx.beginPath(); ctx.arc(...Pp(q), L[3]*1.2*V.sc, 0, 7); ctx.stroke(); } }
    groundAt(ctx, ...Pp(mid(J[k], J[(k + 1) % 4])), L[k]*V.sc + 50, 0);
    for (let i = 0; i < 4; i++){ const p = J[i], q = J[(i + 1) % 4]; if (i === k){ barAt(ctx, ...vadd(Pp(p), [0, 4]), ...vadd(Pp(q), [0, 4]), alpha(C.muted, 0.7), 5); continue; } barAt(ctx, ...Pp(p), ...Pp(q), COL[i], 7); }
    for (let i = 0; i < 4; i++){ const m = Pp(mid(J[i], J[(i + 1) % 4])); tag(ctx, String(i + 1), m[0], m[1] + (i === k ? 18 : 0), COL[i]); }
    const cp = cpt(J, r); ctx.strokeStyle = alpha(C.force, 0.6); ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(...Pp(J[r.coupler])); ctx.lineTo(...Pp(cp)); ctx.lineTo(...Pp(J[(r.coupler + 1) % 4])); ctx.stroke(); dotAt(ctx, ...Pp(cp), C.force, 4);
    for (let i = 0; i < 4; i++){ const q = Pp(J[i]); if (i === k || i === (k + 1) % 4) pivotAt(ctx, ...q); else pinAt(ctx, ...q); lbl(ctx, JN[i], q[0] + 9, q[1] - 12, C.ink); }
    const role = i => i === k ? 'frame (fixed)' : i === r.coupler ? 'coupler' : (r.type.startsWith('double crank') || (i === r.input && range.full)) ? 'crank' : 'rocker';
    let swing = ''; if (range && !range.full) swing = `input rocks through ${n2((range.b - range.a)/DEG, 1)}°`;
    $('fb_st').innerHTML = statsHTML([['s + l', `${n2(r.s + r.lmax, 0)}`], ['p + q', `${n2(r.sum - r.s - r.lmax, 0)}`], ["Grashof's law", r.gr ? 'satisfied' : 'not satisfied', r.gr ? '' : 'bad'], ['This inversion', r.type.split(':')[0]],
      ['Driven link', `${r.input + 1} (${NAMES[r.input]})${swing ? ', ' + swing : ''}`]]);
    $('fb_sub').textContent = r.type; }
  const S = controls($('fb_c'), [['L1', 'Link 1, AB', 20, 220, 1, 100, 'mm'], ['L2', 'Link 2, BC', 20, 220, 1, 75, 'mm'], ['L3', 'Link 3, CD', 20, 220, 1, 110, 'mm'], ['L4', 'Link 4, DA', 20, 220, 1, 40, 'mm']], P, () => { cv._preset = ''; H = null; draw(); });
  const fixSeg = $('fb_fix'); segGroup(fixSeg, v => { fix = +v; H = null; draw(); });
  const PRE = { beam:{ L:[100, 75, 110, 40], fix:0 }, loco:{ L:[130, 32, 130, 32], fix:0 }, watt:{ L:[100, 70, 35, 80], fix:0 } };
  $('fb_pre').querySelectorAll('button').forEach(b => b.addEventListener('click', () => { const p = PRE[b.dataset.v]; ['L1', 'L2', 'L3', 'L4'].forEach((k, i) => S[k].set(p.L[i])); fix = p.fix;
    fixSeg.querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', x.dataset.v === String(p.fix) ? 'true' : 'false')); $('fb_pre').querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', x === b ? 'true' : 'false')); cv._preset = b.dataset.v; H = null; draw(); }));
  watchSize(cv, draw); onRedraw(draw); const an = animate(cv, dt => { ph += dt*1.5; draw(); }); playButton($('fb_play'), an); draw(); }

/* ================= single slider-crank chain inversions ================= */
{ const cv = $('sc'); let mode = 'frame', th = 0.4; const r = 50, l = 140; let box = null;
  const CAP = { frame:'<b>Frame fixed: the reciprocating engine.</b> The crank (2) turns about O, the connecting rod (3) swings and the cross-head (4) slides in the fixed guides (1).',
    slider:'<b>Link 4 fixed: the pendulum pump or bull engine.</b> The block becomes the fixed cylinder. The connecting rod (3) rocks about its pin on the fixed link, the crank (2) turns, and link 1 slides back and forth through the cylinder as the piston rod.',
    rod:'<b>Connecting rod fixed: the oscillating cylinder engine.</b> Link 3 is still. The block (4) becomes the cylinder, rocking on its trunnion at the fixed pin, while link 1, the piston and piston rod, slides in and out of it, driven by the crank (2). The crank and slotted lever quick return comes from this same inversion.',
    crank:'<b>Crank fixed: the rotary (Gnome) engine.</b> Link 2 is still. The frame (the cylinders, in the Gnome engine seven of them) turns about O, the piston slides in it, and the rod turns about the fixed crank pin. The Whitworth quick return also comes from this inversion.' };
  function pose(th){ const O = [0, 0], B = [r*Math.cos(th), r*Math.sin(th)], Px = B[0] - Math.sqrt(l*l - B[1]*B[1]), Pt = [Px, 0];
    return { frame:{ o:O, a:0 }, crank:{ o:O, a:th }, rod:{ o:B, a:Math.atan2(Pt[1] - B[1], Pt[0] - B[0]) }, slider:{ o:Pt, a:0 }, O, B, P:Pt }; }
  const xf = (q, f) => { const c = Math.cos(-f.a), s = Math.sin(-f.a), d = vsub(q, f.o); return [c*d[0] - s*d[1], s*d[0] + c*d[1]]; };
  const unx = (q, f, g) => { const c = Math.cos(f.a), s = Math.sin(f.a); return vadd(f.o, [c*q[0] - s*q[1], s*q[0] + c*q[1]]); };
  function mapper(ps){ const f = ps[mode]; return q => xf(q, f); }
  function bounds(){ let xl = 1e9, xr = -1e9, yb = 1e9, yt = -1e9; for (let i = 0; i < 72; i++){ const ps = pose(i/72*TAU), M = mapper(ps);
      for (const q of [[-250, 22], [-250, -22], [-50, 22], [-50, -22], [r + 10, 0], [-r - 10, 0], [0, r + 10], [0, -r - 10], ps.B, vadd(ps.P, [-25, 18]), vadd(ps.P, [25, -18])]){ const m = M(unx(q, ps.frame) && q); xl = Math.min(xl, m[0]); xr = Math.max(xr, m[0]); yb = Math.min(yb, m[1]); yt = Math.max(yt, m[1]); } }
    const p = 20; box = { xl:xl - p, xr:xr + p, yb:yb - p, yt:yt + p }; }
  function draw(){ if (!box) bounds(); const ps = pose(th), M = mapper(ps), L = LC(), V = fitView(cv, box), { ctx } = V, Pp = q => V.P(M(q));
    const poly = (pts, fill, stroke, w = 1.5) => { ctx.beginPath(); pts.forEach((q, i) => i ? ctx.lineTo(...Pp(q)) : ctx.moveTo(...Pp(q))); ctx.closePath(); if (fill){ ctx.fillStyle = fill; ctx.fill(); } if (stroke){ ctx.strokeStyle = stroke; ctx.lineWidth = w; ctx.stroke(); } };
    // link 1: guide (cylinder) and crank bearing, in frame coordinates (frame pose is the identity)
    const fc = mode === 'frame' ? alpha(C.muted, 0.25) : alpha(L[1], 0.22);
    poly([[-250, 26], [-50, 26], [-50, 18], [-250, 18]], fc, C.muted); poly([[-250, -26], [-50, -26], [-50, -18], [-250, -18]], fc, C.muted); poly([[-258, 26], [-250, 26], [-250, -26], [-258, -26]], fc, C.muted);
    poly([[-50, -26], [-10, -26], [0, -6], [0, 6], [-10, 26], [-50, 26]], null, alpha(C.muted, 0.5), 1);
    // link 4: slider (piston) at P
    const pc = alpha(L[4], 0.85), P0 = ps.P; poly([vadd(P0, [-22, 17]), vadd(P0, [22, 17]), vadd(P0, [22, -17]), vadd(P0, [-22, -17])], pc, null);
    // links 2, 3
    barAt(ctx, ...Pp(ps.O), ...Pp(ps.B), L[2], 8); barAt(ctx, ...Pp(ps.B), ...Pp(ps.P), L[3], 7);
    pinAt(ctx, ...Pp(ps.O), 6); pinAt(ctx, ...Pp(ps.B)); pinAt(ctx, ...Pp(ps.P));
    tag(ctx, '1', ...Pp([-150, 40]), L[1]); tag(ctx, '2', ...Pp(mid(ps.O, ps.B)), L[2]); tag(ctx, '3', ...Pp(mid(ps.B, ps.P)), L[3]); tag(ctx, '4', ...vadd(Pp(P0), [0, 0]), L[4]);
    const fixedAt = { frame:[-150, -34], crank:mid(ps.O, ps.B), rod:mid(ps.B, ps.P), slider:P0 }[mode]; lbl(ctx, 'fixed', ...vadd(Pp(fixedAt), [0, mode === 'frame' ? 18 : 22]), C.ink, 'center', 11.5);
    $('sc_sub').textContent = { frame:'link 1 fixed', slider:'link 4 fixed', rod:'link 3 fixed', crank:'link 2 fixed' }[mode]; }
  segGroup($('sc_seg'), m => { mode = m; box = null; $('sc_cap').innerHTML = CAP[m]; draw(); }); $('sc_cap').innerHTML = CAP.frame;
  watchSize(cv, draw); onRedraw(draw); const an = animate(cv, dt => { th -= dt*1.6; draw(); }); playButton($('sc_play'), an); draw(); }

/* ================= crank and slotted lever QRM ================= */
{ const cv = $('qr'), P = {}; let psi = PI/2;
  controls($('qr_c'), [['AC', 'Fixed centres AC', 100, 400, 5, 240, 'mm'], ['CB', 'Crank CB', 30, 300, 5, 120, 'mm'], ['AP', 'Slotted lever AP', 150, 700, 5, 450, 'mm']], P, () => draw());
  function geo(psi){ const cb = Math.min(P.CB, P.AC*0.95), A = [0, 0], Cc = [0, P.AC], B = vadd(Cc, [cb*Math.cos(psi), cb*Math.sin(psi)]), u = vmul(B, 1/vlen(B)), Pt = vmul(u, P.AP);
    const b0 = Math.asin(cb/P.AC), ys = P.AP*Math.cos(b0), d = 0.22*P.AP, R = [Pt[0] + Math.sqrt(Math.max(0, d*d - (Pt[1] - ys)**2)), ys]; return { A, C:Cc, B, P:Pt, R, ys, b0, cb, d }; }
  function draw(){ const g = geo(psi), L = LC(), ext = Math.max(P.AP, P.AC + g.cb) ; const V = fitView(cv, { xl:-P.AP*0.75, xr:P.AP*0.75 + g.d, yb:-40, yt:Math.max(ext, g.ys + 60) }), { ctx } = V, Pp = V.P;
    const alpha_ = PI - 2*g.b0, beta = TAU - alpha_;
    ctx.strokeStyle = alpha(C.muted, 0.6); ctx.setLineDash([4, 4]); ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(...Pp(g.C), g.cb*V.sc, 0, 7); ctx.stroke();
    const P1 = [-P.AP*Math.sin(g.b0), g.ys], P2 = [P.AP*Math.sin(g.b0), g.ys]; ctx.beginPath(); ctx.moveTo(...Pp(g.A)); ctx.lineTo(...Pp(P1)); ctx.moveTo(...Pp(g.A)); ctx.lineTo(...Pp(P2)); ctx.stroke();
    ctx.beginPath(); ctx.arc(...Pp(g.A), P.AP*V.sc, -PI/2 - g.b0, -PI/2 + g.b0); ctx.stroke(); ctx.setLineDash([]);
    ctx.strokeStyle = C.muted; ctx.lineWidth = 1.4; ctx.beginPath(); ctx.moveTo(...Pp([P1[0] - 30, g.ys])); ctx.lineTo(...Pp([P2[0] + g.d + 60, g.ys])); ctx.stroke(); lbl(ctx, 'line of stroke', ...vadd(Pp([P2[0] + g.d + 55, g.ys]), [0, -12]), C.muted, 'right', 11);
    // tangent points B1, B2 and the angles
    const B1 = vadd(g.C, vmul([-Math.cos(g.b0), -Math.sin(g.b0)], g.cb)), B2 = vadd(g.C, vmul([Math.cos(g.b0), -Math.sin(g.b0)], g.cb));
    for (const [q, n] of [[B1, 'B₁'], [B2, 'B₂']]){ dotAt(ctx, ...Pp(q), C.muted, 3); lbl(ctx, n, ...vadd(Pp(q), [n === 'B₁' ? -10 : 10, 10]), C.muted, n === 'B₁' ? 'right' : 'left', 11); }
    arcM(ctx, Pp, g.C, g.cb + 8/V.sc, -g.b0, PI + g.b0, alpha(C.vel, 0.4)); arcM(ctx, Pp, g.C, g.cb + 8/V.sc, PI + g.b0, TAU - g.b0, alpha(C.acc, 0.4));
    lbl(ctx, `β = ${n2(beta/DEG, 1)}° (cut)`, ...vadd(Pp(vadd(g.C, [0, g.cb])), [0, -20]), C.vel, 'center', 11.5); lbl(ctx, `α = ${n2(alpha_/DEG, 1)}° (return)`, ...vadd(Pp(vadd(g.C, [0, -g.cb])), [0, 22]), C.acc, 'center', 11.5);
    // ram
    ctx.fillStyle = alpha(L[5], 0.85); ctx.fillRect(Pp(g.R)[0] - 10, Pp(g.R)[1] - 26, 90, 18); lbl(ctx, 'ram', Pp(g.R)[0] + 35, Pp(g.R)[1] - 36, C.force, 'center', 11);
    barAt(ctx, ...Pp(g.A), ...Pp(g.P), L[4], 8); barAt(ctx, ...Pp(g.C), ...Pp(g.B), L[2], 7); barAt(ctx, ...Pp(g.P), ...Pp(g.R), L[3], 4);
    ctx.save(); ctx.translate(...Pp(g.B)); ctx.rotate(-Math.atan2(g.B[1], g.B[0])); ctx.fillStyle = C.ink; ctx.fillRect(-9, -6, 18, 12); ctx.restore();
    pivotAt(ctx, ...Pp(g.A)); pivotAt(ctx, ...Pp(g.C)); pinAt(ctx, ...Pp(g.B), 3.5); pinAt(ctx, ...Pp(g.P)); pinAt(ctx, ...Pp(g.R));
    for (const [q, n, dx] of [[g.A, 'A', -12], [g.C, 'C', -12], [g.P, 'P', -12], [g.R, 'R', 12]]) lbl(ctx, n, Pp(q)[0] + dx, Pp(q)[1] - 4, C.ink, dx < 0 ? 'right' : 'left');
    // graph: ram position against crank angle turned from B1 (clockwise)
    const X = [], Y = []; const a0 = Math.atan2(B1[1] - g.C[1], B1[0] - g.C[0]); let cur = 0;
    for (let i = 0; i <= 360; i++){ const q = geo(a0 - i*DEG); X.push(i); Y.push(q.R[0]); }
    cur = ((a0 - psi)/DEG % 360 + 360) % 360;
    chart($('qr_g'), { x:X, series:[{ y:Y, color:C.force, label:'ram position x_R' }], ylab:'x_R [mm]', xlab:'crank angle from CB₁, clockwise [°]', shade:[{ a:0, b:beta/DEG, color:C.vel, label:'cutting' }, { a:beta/DEG, b:360, color:C.acc, label:'return' }], cursor:cur, cursorIndex:Math.round(cur), value:`x_R = ${n2(g.R[0], 0)} mm` });
    const stroke = Math.max(...Y) - Math.min(...Y);
    $('qr_st').innerHTML = statsHTML([['Lever at the extremes, ∠CAB₁ = sin⁻¹(CB/AC)', `${n2(g.b0/DEG, 2)}°`], ['Return angle α', `${n2(alpha_/DEG, 1)}°`], ['Cutting angle β = 360° − α', `${n2(beta/DEG, 1)}°`], ['Time ratio β/α', n2(beta/alpha_, 3)], ['Stroke 2 AP × CB/AC', `${n2(2*P.AP*g.cb/P.AC, 1)} <small>mm</small>`], ['Stroke of R measured in the model', `${n2(stroke, 1)} <small>mm</small>`]]);
    $('qr_sub').textContent = cur < beta/DEG ? 'cutting stroke: slow, left to right' : 'return stroke: quick, right to left'; }
  watchSize(cv, draw); watchSize($('qr_g'), draw); onRedraw(draw); const an = animate(cv, dt => { psi -= dt*1.2; draw(); }); playButton($('qr_play'), an); draw(); }

example($('ex2'), { title:'Example 5.2: crank and slotted lever', src:'notes, Example 5.2 (Khurmi)',
  q:'In a crank and slotted lever quick return mechanism the distance between the fixed centres is 240 mm and the driving crank is 120 mm long. Find the inclination of the slotted bar to the vertical in the extreme position and the time ratio of cutting to return stroke. If the slotted bar is 450 mm long, find the length of the stroke if the line of stroke passes through the extreme positions of the free end of the lever.',
  inputs:[['AC', 'AC', 240, 'mm'], ['CB', 'CB₁', 120, 'mm'], ['AP', 'AP₁', 450, 'mm']],
  solve:v => { const inc = Math.asin(v.CB/v.AC)/DEG, al = 2*(90 - inc);
    return [['sin ∠CAB₁ = sin(90° − α/2) = B₁C/AC =', n2(v.CB/v.AC, 3)], ['Inclination of the slotted bar ∠CAB₁ = 90° − α/2 =', `${n2(inc, 2)}°`, true], ['α/2 = 90° − ∠CAB₁, so α =', `${n2(al, 2)}°`],
      ['Time of cutting / time of return = (360° − α)/α =', n2((360 - al)/al, 3), true], ['Stroke R₁R₂ = 2 AP₁ sin(90° − α/2) =', `${n2(2*v.AP*Math.sin((90 - al/2)*DEG), 1)} mm`, true]]; } });
example($('ex3'), { title:'Example 5.3: designing the oscillating link', src:'notes, Example 5.3 (Khurmi)',
  q:'A quick return mechanism of the oscillating link type has a driving crank BC 30 mm long. The time ratio of working stroke to return stroke is to be 1.7, and the working stroke of R is 120 mm. Find the lengths AC and AP.',
  inputs:[['BC', 'Crank BC', 30, 'mm'], ['k', 'Time ratio', 1.7, ''], ['S', 'Stroke R₁R₂', 120, 'mm']],
  solve:v => { const al = 360/(v.k + 1), h = al/2*DEG;
    return [['(360° − α)/α = ratio, so α = 360°/(ratio + 1) =', `${n2(al, 2)}°`], ['α/2 =', `${n2(al/2, 2)}°`], ['From triangle AB₁C: sin(90° − α/2) = B₁C/AC, so AC = BC/cos(α/2) =', `${n2(v.BC/Math.cos(h), 1)} mm`, true],
      ['Stroke = 2 AP cos(α/2), so AP = stroke/(2 cos(α/2)) =', `${n2(v.S/(2*Math.cos(h)), 1)} mm`, true]]; } });

/* ================= Whitworth QRM ================= */
{ const cv = $('ww'), P = {}; let psi = 0.3;
  controls($('ww_c'), [['CD', 'Fixed link CD', 10, 140, 1, 50, 'mm'], ['CA', 'Driving crank CA', 40, 200, 1, 100, 'mm'], ['DP', 'Lever end DP', 40, 200, 1, 110, 'mm'], ['PR', 'Connecting rod PR', 120, 400, 5, 260, 'mm']], P, () => draw());
  function geo(psi){ const c = Math.min(P.CD, P.CA*0.95), Cc = [0, 0], D = [0, c], A = [P.CA*Math.cos(psi), P.CA*Math.sin(psi)], u = vsub(A, D), ul = vlen(u), Pt = vsub(D, vmul(u, P.DP/ul));
    const pr = Math.max(P.PR, P.DP*1.05), R = [Pt[0] + Math.sqrt(Math.max(0, pr*pr - (Pt[1] - c)**2)), c]; return { C:Cc, D, A, P:Pt, R, c, pr }; }
  function draw(){ const g = geo(psi), L = LC(), V = fitView(cv, { xl:-Math.max(P.CA, P.DP) - 20, xr:P.DP + g.pr + 80, yb:-P.CA - 20, yt:Math.max(P.CA, P.DP + g.c) + 20 }), { ctx } = V, Pp = V.P;
    const s = Math.asin(g.c/P.CA), beta = PI - 2*s, al = PI + 2*s;
    ctx.strokeStyle = alpha(C.muted, 0.6); ctx.setLineDash([4, 4]); ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(...Pp(g.C), P.CA*V.sc, 0, 7); ctx.stroke(); ctx.beginPath(); ctx.arc(...Pp(g.D), P.DP*V.sc, 0, 7); ctx.stroke(); ctx.setLineDash([]);
    ctx.strokeStyle = C.muted; ctx.lineWidth = 1.4; ctx.beginPath(); ctx.moveTo(...Pp([-Math.max(P.CA, P.DP), g.c])); ctx.lineTo(...Pp([P.DP + g.pr + 70, g.c])); ctx.stroke();
    arcM(ctx, Pp, g.C, P.CA + 8/V.sc, s, PI - s, alpha(C.acc, 0.4)); arcM(ctx, Pp, g.C, P.CA + 8/V.sc, PI - s, TAU + s, alpha(C.vel, 0.4));
    lbl(ctx, `β = ${n2(beta/DEG, 1)}° (return)`, ...vadd(Pp([0, P.CA]), [0, -20]), C.acc, 'center', 11.5); lbl(ctx, `α = ${n2(al/DEG, 1)}° (cut)`, ...vadd(Pp([0, -P.CA]), [0, 22]), C.vel, 'center', 11.5);
    ctx.fillStyle = alpha(L[5], 0.85); ctx.fillRect(Pp(g.R)[0] - 10, Pp(g.R)[1] - 9, 90, 18); lbl(ctx, 'ram', Pp(g.R)[0] + 35, Pp(g.R)[1] - 20, C.force, 'center', 11);
    const far = vadd(g.D, vmul(vsub(g.A, g.D), (P.CA + g.c + 10)/vlen(vsub(g.A, g.D)))); barAt(ctx, ...Pp(g.P), ...Pp(far), L[1], 8); barAt(ctx, ...Pp(g.C), ...Pp(g.A), L[3], 7); barAt(ctx, ...Pp(g.P), ...Pp(g.R), L[4], 4);
    barAt(ctx, ...Pp(g.C), ...Pp(g.D), alpha(C.muted, 0.7), 5);
    ctx.save(); ctx.translate(...Pp(g.A)); ctx.rotate(-Math.atan2(g.A[1] - g.D[1], g.A[0] - g.D[0])); ctx.fillStyle = C.ink; ctx.fillRect(-9, -6, 18, 12); ctx.restore();
    pivotAt(ctx, ...Pp(g.C)); pinAt(ctx, ...Pp(g.D), 5); pinAt(ctx, ...Pp(g.A), 3.5); pinAt(ctx, ...Pp(g.P)); pinAt(ctx, ...Pp(g.R));
    for (const [q, n, dx] of [[g.C, 'C', 14], [g.D, 'D', -12], [g.P, 'P', -12], [g.R, 'R', 12], [g.A, 'A', 12]]) lbl(ctx, n, Pp(q)[0] + dx, Pp(q)[1] - 12, C.ink, dx < 0 ? 'right' : 'left');
    const X = [], Y = [], a1 = s; for (let i = 0; i <= 360; i++){ X.push(i); Y.push(geo(a1 - i*DEG).R[0]); }   // from CA1 (right end of the line) clockwise
    const cur = ((a1 - psi)/DEG % 360 + 360) % 360;
    chart($('ww_g'), { x:X, series:[{ y:Y, color:C.force, label:'ram position x_R' }], ylab:'x_R [mm]', xlab:'crank angle from CA₁, clockwise [°]', shade:[{ a:0, b:al/DEG, color:C.vel, label:'cutting (α)' }, { a:al/DEG, b:360, color:C.acc, label:'return (β)' }], cursor:cur, cursorIndex:Math.round(cur), value:`x_R = ${n2(g.R[0], 0)} mm` });
    $('ww_st').innerHTML = statsHTML([['Cutting angle α', `${n2(al/DEG, 1)}°`], ['Return angle β', `${n2(beta/DEG, 1)}°`], ['Time ratio α/β', n2(al/beta, 3)], ['Stroke 2 PD', `${n2(2*P.DP, 0)} <small>mm</small>`], ['Stroke in the model', `${n2(Math.max(...Y) - Math.min(...Y), 1)} <small>mm</small>`]]);
    $('ww_sub').textContent = cur < al/DEG ? 'cutting stroke: slow' : 'return stroke: quick'; }
  watchSize(cv, draw); watchSize($('ww_g'), draw); onRedraw(draw); const an = animate(cv, dt => { psi -= dt*1.2; draw(); }); playButton($('ww_play'), an); draw(); }

/* ================= elliptical trammels ================= */
{ const cv = $('et'), P = {}; let th = 0.6, tr = [];
  controls($('et_c'), [['AB', 'Bar AB', 60, 200, 1, 150, 'mm'], ['AP', 'Distance AP', 0, 260, 1, 100, 'mm', (v, Q) => v > Q.AB ? 'P beyond B' : v === Q.AB/2 ? 'midpoint' : '']], P, () => { tr = []; draw(); });
  function draw(){ const AB = P.AB, AP = P.AP, BP = Math.abs(AB - AP), R = Math.max(AB, AP, BP) + 30, V = fitView(cv, { xl:-R, xr:R, yb:-R*0.75, yt:R*0.75 }), { ctx } = V, Pp = V.P, L = LC();
    const A = [0, AB*Math.sin(th)], B = [-AB*Math.cos(th), 0], Pt = vadd(A, vmul(vsub(B, A), AP/AB));
    ctx.fillStyle = alpha(C.muted, 0.12); ctx.fillRect(...Pp([-R + 10, 12]), (2*R - 20)*V.sc, 24*V.sc); ctx.fillRect(...Pp([-12, R*0.75 - 5]), 24*V.sc, (1.5*R - 10)*V.sc);
    ctx.strokeStyle = alpha(C.muted, 0.7); ctx.lineWidth = 1; ctx.setLineDash([5, 4]); ctx.beginPath(); ctx.ellipse(...Pp([0, 0]), AP*V.sc, BP*V.sc, 0, 0, 7); ctx.stroke(); ctx.setLineDash([]);
    tr.push(Pt); if (tr.length > 900) tr.shift(); ctx.strokeStyle = C.force; ctx.lineWidth = 2; ctx.beginPath(); tr.forEach((q, i) => i ? ctx.lineTo(...Pp(q)) : ctx.moveTo(...Pp(q))); ctx.stroke();
    const ext = AP > AB ? Pt : B; barAt(ctx, ...Pp(A), ...Pp(ext), L[2], 7);
    ctx.fillStyle = alpha(L[3], 0.9); ctx.fillRect(Pp(A)[0] - 11, Pp(A)[1] - 15, 22, 30); ctx.fillStyle = alpha(L[1], 0.9); ctx.fillRect(Pp(B)[0] - 15, Pp(B)[1] - 11, 30, 22);
    pinAt(ctx, ...Pp(A)); pinAt(ctx, ...Pp(B)); dotAt(ctx, ...Pp(Pt), C.force, 6);
    lbl(ctx, 'A', Pp(A)[0] + 16, Pp(A)[1], C.ink); lbl(ctx, 'B', Pp(B)[0], Pp(B)[1] + 22, C.ink, 'center'); lbl(ctx, 'P', Pp(Pt)[0] + 10, Pp(Pt)[1] - 12, C.force);
    $('et_st').innerHTML = statsHTML([['Semi-axis along x = AP', `${n2(AP, 0)} <small>mm</small>`], ['Semi-axis along y = BP', `${n2(BP, 0)} <small>mm</small>`], ['x²/AP² + y²/BP²', AP && BP ? n2(Pt[0]**2/AP**2 + Pt[1]**2/BP**2, 4) : 'n/a'], ['Path of P', AP === BP ? 'a circle' : AP === 0 || BP === 0 ? 'a straight line' : 'an ellipse']]);
    $('et_sub').textContent = `bar at θ = ${n2(((th/DEG) % 360 + 360) % 360, 0)}°`; }
  $('et_clear').addEventListener('click', () => { tr = []; draw(); });
  watchSize(cv, draw); onRedraw(draw); const an = animate(cv, dt => { th += dt*0.9; draw(); }); playButton($('et_play'), an); draw(); }

/* ================= Scotch yoke ================= */
{ const cv = $('sy'); let th = 0.5; const r = 60;
  function draw(){ const L = LC(), V = fitView(cv, { xl:-170, xr:210, yb:-110, yt:110 }), { ctx } = V, Pp = V.P, B = [0, 0], A = [r*Math.cos(th), r*Math.sin(th)], x = A[0];
    ctx.strokeStyle = alpha(C.muted, 0.6); ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.arc(...Pp(B), r*V.sc, 0, 7); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle = alpha(C.muted, 0.2); for (const y of [16, -16]) ctx.fillRect(...Pp([90, y + 6]), 110*V.sc, 6*V.sc);
    ctx.strokeStyle = L[4]; ctx.lineWidth = 6; ctx.beginPath(); ctx.moveTo(...Pp([x - 8, 90])); ctx.lineTo(...Pp([x - 8, -90])); ctx.moveTo(...Pp([x + 8, 90])); ctx.lineTo(...Pp([x + 8, -90])); ctx.moveTo(...Pp([x - 8, 90])); ctx.lineTo(...Pp([x + 8, 90])); ctx.moveTo(...Pp([x - 8, -90])); ctx.lineTo(...Pp([x + 8, -90])); ctx.stroke();
    barAt(ctx, ...Pp([x + 8, 0]), ...Pp([x + 150, 0]), L[4], 8);
    ctx.fillStyle = L[3]; ctx.fillRect(Pp(A)[0] - 8, Pp(A)[1] - 10, 16, 20);
    barAt(ctx, ...Pp(B), ...Pp(A), L[2], 8); pivotAt(ctx, ...Pp(B)); pinAt(ctx, ...Pp(A), 4);
    lbl(ctx, 'B', Pp(B)[0] - 12, Pp(B)[1] - 10, C.ink, 'right'); lbl(ctx, 'crank 2', ...vadd(Pp(mid(B, A)), [0, -14]), L[2], 'center', 11); lbl(ctx, 'yoke 4', Pp([x + 80, 0])[0], Pp([x, 0])[1] - 16, L[4], 'center', 11);
    const X = [], Y = []; for (let i = 0; i <= 360; i++){ X.push(i); Y.push(r*Math.cos(i*DEG)); } const cur = ((th/DEG) % 360 + 360) % 360;
    chart($('sy_g'), { x:X, series:[{ y:Y, color:L[4], label:'yoke displacement x = r cos θ' }], ylab:'x [mm]', xlab:'crank angle θ [°]', cursor:cur, cursorIndex:Math.round(cur), value:`x = ${n2(x, 1)} mm` });
    $('sy_sub').textContent = 'simple harmonic motion of the yoke'; }
  watchSize(cv, draw); watchSize($('sy_g'), draw); onRedraw(draw); const an = animate(cv, dt => { th += dt*1.3; draw(); }); playButton($('sy_play'), an); draw(); }

/* ================= Oldham's coupling (3D) ================= */
{ const P = {}; let th = 0, parts = null, trail;
  controls($('ol_c'), [['e', 'Distance between the shaft axes r', 0, 60, 1, 30, 'mm'], ['N', 'Shaft speed', 10, 600, 10, 60, 'rpm']], P, () => { if (v) v.render(); stats(); });
  const S = 1/200;   // scene units per mm
  const v = view3d($('ol3'), { cam:[2.1, 1.1, 1.9], target:[0, 0.05, 0], fov:34,
    build(v){ const T = THREE; readColors(); const steel = mat('#9aa6b2', { metalness:0.7, roughness:0.3 }), f1m = mat(C.crank), f2m = mat(C.vel), dm = mat(C.rod), dark = mat('#39424c'), body = mat('#7f8c99', { metalness:0.2, roughness:0.7 });
      const R = 0.5, flange = (m, slotDir) => { const g = new T.Group(); const d = new T.Mesh(new T.CylinderGeometry(R, R, 0.12, 48), m); d.rotation.z = PI/2; g.add(d);
          const slot = new T.Mesh(new T.BoxGeometry(0.03, slotDir === 'y' ? 2*R + 0.01 : 0.14, slotDir === 'y' ? 0.14 : 2*R + 0.01), dark); g.add(slot); slot.position.x = 0; return g; };
      const sh1 = new T.Mesh(new T.CylinderGeometry(0.09, 0.09, 1, 24), steel); sh1.rotation.z = PI/2; const sh2 = sh1.clone();
      const g1 = new T.Group(), f1 = flange(f1m, 'y'); f1.position.x = -0.18; g1.add(f1); sh1.position.x = -0.75; g1.add(sh1); const k1 = new T.Mesh(new T.BoxGeometry(0.3, 0.03, 0.06), mat(C.acc)); k1.position.set(-0.8, 0.09, 0); g1.add(k1);
      const g2 = new T.Group(), f2 = flange(f2m, 'z'); f2.position.x = 0.18; g2.add(f2); sh2.position.x = 0.75; g2.add(sh2); const k2 = k1.clone(); k2.position.x = 0.8; g2.add(k2);
      const disc = new T.Group(); const dd = new T.Mesh(new T.CylinderGeometry(R*0.92, R*0.92, 0.16, 48), dm); dd.rotation.z = PI/2; disc.add(dd);
      const t1 = new T.Mesh(new T.BoxGeometry(0.1, 2*R*0.9, 0.13), dm); t1.position.x = -0.1; disc.add(t1); const t2 = new T.Mesh(new T.BoxGeometry(0.1, 0.13, 2*R*0.9), dm); t2.position.x = 0.1; disc.add(t2);
      const pts = []; for (let i = 0; i <= 64; i++) pts.push(new T.Vector3(0, 0, 0)); trail = new T.Line(new T.BufferGeometry().setFromPoints(pts), new T.LineBasicMaterial({ color:new T.Color(C.acc) }));
      const cdot = new T.Mesh(new T.SphereGeometry(0.025, 12, 8), mat(C.acc)); disc.add(cdot); cdot.position.x = 0.19;
      const bear = x => { const b = new T.Mesh(new T.BoxGeometry(0.16, 0.5, 0.36), body); b.position.set(x, -0.3, 0); return b; };
      const b1 = bear(-0.95), b2 = bear(0.95); const base = new T.Mesh(new T.BoxGeometry(2.3, 0.04, 0.8), body); base.position.y = -0.57;
      v.root.add(g1, g2, disc, trail, b1, b2, base); parts = { g1, g2, disc, b2, f1, f2 }; },
    update(dt){ th += dt*P.N*TAU/60*Math.min(1, 1.5/(P.N*TAU/60)); place(); } });
  function place(){ if (!parts) return; const e = P.e*S, ex = $('ol_ex').checked ? 0.35 : 0;
    // shaft 1 axis at y = 0, shaft 2 axis at y = e; slot 1 along the flange's local y, slot 2 along its local z
    parts.g1.rotation.x = th; parts.g1.position.set(-ex, 0, 0); parts.g2.rotation.x = th; parts.g2.position.set(ex, e, 0); parts.b2.position.y = -0.3 + e;
    const u1 = [Math.cos(th), Math.sin(th)], O2 = [e, 0], s = O2[0]*u1[0] + O2[1]*u1[1], M = [s*u1[0], s*u1[1]];
    parts.disc.position.set(0, M[0], M[1]); parts.disc.rotation.x = th;
    const p = trail.geometry.attributes.position; for (let i = 0; i < p.count; i++){ const a = i/(p.count - 1)*TAU; p.setXYZ(i, 0.2, e/2 + e/2*Math.cos(a), e/2*Math.sin(a)); } p.needsUpdate = true; stats(s); }
  function stats(s){ const w = P.N*TAU/60, e = P.e/1000; const sl = s != null ? Math.abs(-e*w*Math.sin(th + PI/2 - PI/2)) : 0;
    $('ol_st').innerHTML = statsHTML([['Angular speed ω', `${n2(w, 2)} <small>rad/s</small>`], ['Path of the disc centre', `circle of diameter ${P.e} <small>mm</small>`], ['Greatest sliding speed v = rω', `${n2(e*w*1000, 1)} <small>mm/s</small>`], ['Sliding speed now', `${n2(sl*1000, 1)} <small>mm/s</small>`]]); }
  $('ol_ex').addEventListener('change', () => { place(); if (v) v.render(); });
  $('ol_sub').textContent = 'both shafts turn at the same speed'; if (v){ playButton($('ol_play'), v.anim); place(); } else stats(); }

/* ================= quiz ================= */
quiz($('qz'), [
  { q:'A plain round shaft in a round hole can turn or slide. What kind of constrained motion is this?', o:['Completely constrained', 'Incompletely constrained', 'Successfully constrained', 'Unconstrained chain'], a:1, why:'Two unrelated motions are possible, so the motion depends on the force: incompletely constrained.' },
  { q:'An engine valve is held on its seat by a spring. Its motion is…', o:['completely constrained', 'incompletely constrained', 'successfully constrained', 'a higher pair'], a:2, why:'The pair alone would allow more motion; the spring (an outside means) completes the constraint.' },
  { q:'Which of these is a higher pair?', o:['Piston and cylinder', 'Lead screw and nut', 'Ball and socket', 'Cam and follower'], a:3, why:'The cam touches the follower along a line; the others have surface contact (lower pairs).' },
  { q:'A chain has l = 5 and j = 5. Using j = (3/2)l − 2, it is…', o:['a locked chain', 'a kinematic chain', 'an unconstrained chain', 'impossible to say'], a:2, why:'(3/2)(5) − 2 = 5.5 and j = 5 < 5.5, so the chain is unconstrained.' },
  { q:'Four links meet at one pin. How many binary joints is that equivalent to?', o:['1', '2', '3', '4'], a:2, why:'A joint of n links counts as n − 1 binary joints: a quaternary joint is 3.' },
  { q:'What does the Kutzbach criterion give for l = 6, j = 8, h = 0?', o:['1', '0', '−1', '2'], a:2, why:'3(6 − 1) − 2×8 = 15 − 16 = −1: a statically indeterminate structure.' },
  { q:'A four-bar has links 40, 75, 100 and 110 mm. Which link, if fixed, gives a double crank?', o:['The 40 mm link', 'The 75 mm link', 'The 100 mm link', 'The 110 mm link'], a:0, why:"40 + 110 = 150 ≤ 175, so Grashof's law holds; fixing the shortest link lets both side links turn fully." },
  { q:'In a crank and slotted lever quick return, AC = 200 mm and CB = 100 mm. The time ratio is…', o:['1.5', '2', '2.5', '3'], a:1, why:'cos(α/2) = 100/200, so α/2 = 60° and α = 120°. Ratio = (360 − 120)/120 = 2.' },
  { q:'In the elliptical trammels, P is the midpoint of AB. P traces…', o:['an ellipse', 'a circle', 'a straight line', 'a parabola'], a:1, why:'AP = BP, so x² + y² = AP²: a circle.' },
  { q:"Two shafts 20 mm apart are joined by an Oldham's coupling turning at 10 rad/s. The greatest sliding speed of a tongue is…", o:['0.1 m/s', '0.2 m/s', '0.4 m/s', '2 m/s'], a:1, why:'v = r ω = 0.02 × 10 = 0.2 m/s.' } ]);
})();
