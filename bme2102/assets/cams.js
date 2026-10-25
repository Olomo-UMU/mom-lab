/* BME 2102 interactive notes, Chapter 6: cam mechanisms */
(() => {
'use strict';
const PI = Math.PI, TAU = 2*PI;
/* ---------- follower motions: f(u) on 0..1 with first and second derivatives in u ---------- */
const MOT = {
  uv:  { name:'uniform velocity', f:u => u, d1:() => 1, d2:() => 0, cv:1, ca:Infinity },
  shm: { name:'simple harmonic', f:u => (1 - Math.cos(PI*u))/2, d1:u => PI/2*Math.sin(PI*u), d2:u => PI*PI/2*Math.cos(PI*u), cv:PI/2, ca:PI*PI/2 },
  uarm:{ name:'uniform acceleration and retardation', f:u => u < 0.5 ? 2*u*u : 1 - 2*(1 - u)**2, d1:u => u < 0.5 ? 4*u : 4*(1 - u), d2:u => u < 0.5 ? 4 : -4, cv:2, ca:4 },
  cyc: { name:'cycloidal', f:u => u - Math.sin(TAU*u)/TAU, d1:u => 1 - Math.cos(TAU*u), d2:u => TAU*Math.sin(TAU*u), cv:2, ca:TAU } };
/* cam program: rise over `out`, dwell `d1`, return over `ret`, dwell to 360. Angles in degrees, S in mm. Returns s, ds/dθ, d²s/dθ² (θ in rad) */
function program(P){ const o = P.out*DEG, d = P.d1*DEG, r = P.ret*DEG, S = P.S, mo = MOT[P.mo], mr = MOT[P.mr];
  return th => { th = ((th % TAU) + TAU) % TAU;
    if (th < o){ const u = th/o; return [S*mo.f(u), S*mo.d1(u)/o, S*mo.d2(u)/(o*o), 'out', u]; }
    if (th < o + d) return [S, 0, 0, 'dwell', 0];
    if (th < o + d + r){ const u = (th - o - d)/r; return [S*(1 - mr.f(u)), -S*mr.d1(u)/r, -S*mr.d2(u)/(r*r), 'ret', u]; }
    return [0, 0, 0, 'rest', 0]; }; }
const rot = (x, y, a) => [x*Math.cos(a) - y*Math.sin(a), x*Math.sin(a) + y*Math.cos(a)];
/* cam profile by inversion. Cam turns clockwise, so in the cam's own frame the follower goes round anticlockwise.
   Follower line of motion is vertical, offset e to the right; trace point starts at (e, √(Rp² − e²)). */
function camModel(P){
  const prog = program(P), type = P.type, rr = type === 'roller' ? P.rr : 0, Rp = P.Rb + rr, e = type === 'flat' ? 0 : P.e;
  const ok = Math.abs(e) < Rp*0.98; const y0 = ok ? Math.sqrt(Rp*Rp - e*e) : Rp;
  const N = 1440, th = new Float64Array(N + 1), pitch = [], prof = [], s = [], v = [], a = [], pa = new Float64Array(N + 1), rho = new Float64Array(N + 1);
  for (let i = 0; i <= N; i++){ const t = i/N*TAU; th[i] = t; const [si, s1, s2] = prog(t); s.push(si); v.push(s1); a.push(s2); }
  const pitchAt = t => { const [si] = prog(t); return rot(e, y0 + si, t); };
  for (let i = 0; i <= N; i++){ const t = th[i];
    if (type === 'flat'){ const h = P.Rb + s[i], h1 = v[i], n = [-Math.sin(t), Math.cos(t)], n1 = [-Math.cos(t), -Math.sin(t)];
      prof.push([h*n[0] + h1*n1[0], h*n[1] + h1*n1[1]]); pitch.push(rot(0, P.Rb + s[i], t)); rho[i] = P.Rb + s[i] + a[i]; pa[i] = 0; continue; }
    const p = pitchAt(t), dt = 1e-4, p1 = pitchAt(t + dt), p0 = pitchAt(t - dt), T = [(p1[0] - p0[0])/(2*dt), (p1[1] - p0[1])/(2*dt)], T2 = [(p1[0] - 2*p[0] + p0[0])/(dt*dt), (p1[1] - 2*p[1] + p0[1])/(dt*dt)];
    const L = Math.hypot(T[0], T[1]) || 1; let n = [T[1]/L, -T[0]/L]; if (n[0]*p[0] + n[1]*p[1] < 0) n = [-n[0], -n[1]];
    pitch.push(p); prof.push([p[0] - rr*n[0], p[1] - rr*n[1]]);
    const d = rot(0, 1, t); pa[i] = Math.acos(Math.min(1, Math.abs(n[0]*d[0] + n[1]*d[1])))/DEG;
    const cr = T[0]*T2[1] - T[1]*T2[0]; rho[i] = cr !== 0 ? L*L*L/cr : Infinity; }
  let paMax = 0, paAt = 0; for (let i = 0; i <= N; i++) if (pa[i] > paMax){ paMax = pa[i]; paAt = i; }
  let rhoMin = Infinity; for (let i = 0; i <= N; i++) if (rho[i] > 0) rhoMin = Math.min(rhoMin, rho[i]);
  let rhoNeg = Infinity; for (let i = 0; i <= N; i++) rhoNeg = Math.min(rhoNeg, rho[i]);
  const w = (P.N || 0)*TAU/60;
  const cut = type === 'roller' ? rhoMin < rr : type === 'flat' ? rhoNeg <= 0 : false;
  let flatW = 0; if (type === 'flat') for (let i = 0; i <= N; i++) flatW = Math.max(flatW, Math.abs(v[i]));
  let Rmax = 0; for (const p of prof) Rmax = Math.max(Rmax, Math.hypot(p[0], p[1]));
  return { P, prog, type, rr, Rp, e, y0, N, th, s, v, a, pitch, prof, pa, paMax, paAt, rho, rhoMin, rhoNeg, cut, flatW, w, Rmax, ok }; }

/* ---------- canvas helper ---------- */
function view(cv, b, pad = 14){ const { ctx, W, H } = prep(cv); const sc = Math.min((W - 2*pad)/(b.xr - b.xl), (H - 2*pad)/(b.yt - b.yb));
  const ox = pad + (W - 2*pad - (b.xr - b.xl)*sc)/2 - b.xl*sc, oy = pad + (H - 2*pad - (b.yt - b.yb)*sc)/2 + b.yt*sc;
  return { ctx, W, H, sc, PX:x => ox + x*sc, PY:y => oy - y*sc }; }
function poly(ctx, V, pts, close, fill, stroke, lw = 1.4, a = 0){ ctx.beginPath(); pts.forEach((p, i) => { const [x, y] = a ? rot(p[0], p[1], a) : p; i ? ctx.lineTo(V.PX(x), V.PY(y)) : ctx.moveTo(V.PX(x), V.PY(y)); }); if (close) ctx.closePath();
  if (fill){ ctx.fillStyle = fill; ctx.fill(); } if (stroke){ ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); } }
function circ(ctx, V, cx, cy, r, col, dash = [], lw = 1){ ctx.strokeStyle = col; ctx.lineWidth = lw; ctx.setLineDash(dash); ctx.beginPath(); ctx.arc(V.PX(cx), V.PY(cy), r*V.sc, 0, 7); ctx.stroke(); ctx.setLineDash([]); }
const dot = (ctx, x, y, col, r = 3.5) => { ctx.fillStyle = col; ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); };
const lbl = (ctx, t, x, y, col, al = 'left', size = 12) => label(ctx, t, x, y, col, al, size, true);

/* ================= 3D helpers ================= */
function camGeo(prof, depth, bore, scale){ const s = new THREE.Shape(); const step = 4;
  for (let i = 0; i < prof.length - 1; i += step){ const [x, y] = prof[i]; i ? s.lineTo(x*scale, y*scale) : s.moveTo(x*scale, y*scale); } s.closePath();
  const h = new THREE.Path(); for (let i = 0; i <= 40; i++){ const t = -i/40*TAU; i ? h.lineTo(bore*Math.cos(t), bore*Math.sin(t)) : h.moveTo(bore, 0); } s.holes.push(h);
  const g = new THREE.ExtrudeGeometry(s, { depth, bevelEnabled:true, bevelThickness:depth*0.06, bevelSize:depth*0.05, bevelSegments:2, curveSegments:4 }); g.translate(0, 0, -depth/2); return g; }
const cyl = (r, h, m, axis = 'y', seg = 28) => { const g = new THREE.CylinderGeometry(r, r, h, seg); if (axis === 'z') g.rotateX(PI/2); if (axis === 'x') g.rotateZ(PI/2); return new THREE.Mesh(g, m); };
const box = (x, y, z, m) => new THREE.Mesh(new THREE.BoxGeometry(x, y, z), m);
function lightMats(v){ readColors(); return { cam:mat(C.crank, { metalness:0.5, roughness:0.35 }), fol:mat(C.rod, { metalness:0.45, roughness:0.4 }), steel:mat('#9aa6b2', { metalness:0.65, roughness:0.32 }), dark:mat('#3b4652', { metalness:0.3, roughness:0.6 }),
  body:mat('#b9c3cc', { metalness:0.15, roughness:0.75, transparent:true, opacity:0.55 }), spring:mat(C.vel, { metalness:0.5, roughness:0.35 }), guide:mat('#7f8c99', { metalness:0.3, roughness:0.6 }) }; }

/* ================= 6.1 valve train ================= */
{ const P = { S:9, out:115, d1:0, ret:115, mo:'cyc', mr:'cyc', Rb:19, type:'flat', e:0, rr:0, N:0 }, M = camModel(P), k = 0.001;
  const v = view3d($('vt'), { cam:[0.17, 0.02, 0.24], target:[0, -0.055, 0], fov:32,
    build(v){ const T = THREE, m = v.m = lightMats(v), g = new T.Group(); v.root.add(g); g.rotation.z = PI;   // cam on top, valve below
      const camM = new T.Mesh(camGeo(M.prof, 0.016, 0.006, k), m.cam); const shaft = cyl(0.009, 0.12, m.steel, 'z'); const cam = new T.Group(); cam.add(camM); cam.add(shaft); g.add(cam); v.cam3 = cam;
      const fol = new T.Group(); g.add(fol); v.fol = fol;
      const bucket = cyl(0.015, 0.02, m.fol); bucket.position.y = 0.010; fol.add(bucket);
      const stem = cyl(0.0035, 0.09, m.steel); stem.position.y = 0.02 + 0.045; fol.add(stem);
      const head = new T.Mesh(new T.CylinderGeometry(0.016, 0.006, 0.006, 32), m.steel); head.position.y = 0.02 + 0.09 + 0.003; head.rotation.x = PI; fol.add(head);
      const retainer = cyl(0.011, 0.003, m.fol); retainer.position.y = 0.024; fol.add(retainer);
      class Hx extends T.Curve { getPoint(u){ const t = u*7*TAU; return new T.Vector3(0.009*Math.cos(t), u, 0.009*Math.sin(t)); } }
      const sp = new T.Mesh(new T.TubeGeometry(new Hx(), 400, 0.0014, 8, false), m.spring); g.add(sp); v.spring = sp;
      // cylinder head block (cut away) and valve seat
      const blk = box(0.07, 0.05, 0.05, m.body); blk.position.y = 0.1; g.add(blk); v.blk = blk;
      const seat = new T.Mesh(new T.TorusGeometry(0.0165, 0.0022, 10, 40), m.dark); seat.rotation.x = PI/2; seat.position.y = 0.1285; g.add(seat); },
    update(dt, v){ v.t = (v.t || 0) + dt*1.4; const th = v.t, [s] = M.prog(th); v.cam3.rotation.z = -th; v.fol.position.y = 0.019 + s*k;
      const top = 0.0255 + 0.019 + s*k, bottom = 0.075; v.spring.position.y = top; v.spring.scale.y = bottom - top; } });
  if (v){ playButton($('vt_play'), v.anim); $('vt_cut').addEventListener('change', e => { v.blk.visible = e.target.checked; v.render(); }); }
  $('vt_cap').textContent = 'The camshaft turns at half crankshaft speed. Its lobe pushes the bucket tappet (a flat-faced follower) and the valve down against the spring; the spring closes the valve and keeps the tappet on the cam. Lift 9 mm, cycloidal opening and closing over 115° each.'; }

/* ================= 6.2 followers in 3D ================= */
const FOL = {
  knife:['Knife-edge follower', 'radial, reciprocating', 'The sharp edge slides on the cam. Its pitch curve is the cam profile itself. Wears quickly and pushes sideways on the guide, so it is rarely used for real loads.'],
  roller:['Roller follower', 'radial, reciprocating', 'A roller turns on a pin at the end of the follower, so the contact rolls. The cam profile lies one roller radius inside the pitch curve traced by the roller centre.'],
  flat:['Flat-faced (mushroom) follower', 'radial, reciprocating', 'A flat face touches the cam. The contact moves across the face (it is offset from the axis by ds/dθ), so the face must be wide enough. Side thrust is small, so engine valves use it.'],
  sphere:['Spherical-faced follower', 'radial, reciprocating', 'The flat face is ground to a large sphere to reduce contact stress; for the cam it acts like a big roller.'],
  offset:['Offset roller follower', 'offset, reciprocating', 'The line of motion is moved 15 mm to one side of the cam centre. Offset reduces the pressure angle on the outstroke (and increases it on the return).'],
  osc:['Oscillating roller follower', 'oscillating', 'The follower is an arm swinging about a fixed pivot; the cam sets its angle instead of its height.'] };
{ let ftype = 'knife'; const base = { S:30, out:120, d1:30, ret:120, mo:'shm', mr:'shm', Rb:40, rr:10, e:0, N:0 }, k = 0.001;
  function modelFor(t){ const P = { ...base, type:t === 'sphere' ? 'roller' : t === 'offset' ? 'roller' : t === 'osc' ? 'roller' : t, rr:t === 'sphere' ? 30 : 10, e:t === 'offset' ? 15 : 0 };
    if (t !== 'osc') return camModel(P);
    // oscillating: arm pivot at (−70, 60) mm, arm length 70 mm, roller swings through ψ = s/L
    const L = 70, piv = [-70, 0], Rp = P.Rb + P.rr, prog = program(P);
    const g0 = Math.acos(((Rp*Rp) - (piv[0]*piv[0] + piv[1]*piv[1]) - L*L)/(-2*L*Math.hypot(piv[0], piv[1]))) ;
    const A0 = Math.atan2(-piv[1], -piv[0]) + g0;   // arm angle that puts the roller centre on the prime circle
    const world = t2 => { const [s] = prog(t2), ang = A0 + s/L; return [piv[0] + L*Math.cos(ang), piv[1] + L*Math.sin(ang)]; };
    const N = 1440, prof = []; for (let i = 0; i <= N; i++){ const t2 = i/N*TAU, p = rot(...world(t2), t2), dt = 1e-4, p1 = rot(...world(t2 + dt), t2 + dt), p0 = rot(...world(t2 - dt), t2 - dt);
      const T = [p1[0] - p0[0], p1[1] - p0[1]], Ln = Math.hypot(...T) || 1; let n = [T[1]/Ln, -T[0]/Ln]; if (n[0]*p[0] + n[1]*p[1] < 0) n = [-n[0], -n[1]]; prof.push([p[0] - P.rr*n[0], p[1] - P.rr*n[1]]); }
    return { prof, prog, osc:{ L, piv, A0 }, P, rr:P.rr, Rp, e:0, y0:Rp, type:'roller' }; }
  const v = view3d($('cf'), { cam:[0.07, 0.1, 0.36], target:[0, 0.05, 0], fov:32,
    build(v){ v.m = lightMats(v); build(v); },
    update(dt, v){ v.t = (v.t || 0) + dt*0.9; if (v.step) v.step(v.t); } });
  function build(v){ const T = THREE, m = v.m; while (v.root.children.length) v.root.remove(v.root.children[0]);
    const M = modelFor(ftype), w = 0.022, camM = new T.Mesh(camGeo(M.prof, w, 0.008, k), m.cam), cam = new T.Group(); cam.add(camM); cam.add(cyl(0.008, 0.1, m.steel, 'z')); v.root.add(cam);
    const fol = new T.Group(); v.root.add(fol); let step;
    if (ftype === 'osc'){ const { L, piv, A0 } = M.osc, arm = new T.Group(); arm.position.set(piv[0]*k, piv[1]*k, 0); v.root.add(arm);
      const bar = box(L*k, 0.008, 0.008, m.fol); bar.position.x = L*k/2; arm.add(bar); const roll = cyl(M.rr*k, w*0.8, m.fol, 'z'); roll.position.x = L*k; arm.add(roll); arm.add(cyl(0.005, 0.04, m.steel, 'z'));
      const pin = cyl(0.003, w*1.2, m.steel, 'z'); pin.position.x = L*k; arm.add(pin);
      step = t => { const [s] = M.prog(t); cam.rotation.z = -t; arm.rotation.z = A0 + s/70; };
    } else {
      const guide = new T.Group(); v.root.add(guide); const gy = (M.Rp + base.S + 40)*k; const g1 = box(0.004, 0.03, 0.024, m.guide), g2 = g1.clone(); g1.position.set(M.e*k - 0.0085, gy, 0); g2.position.set(M.e*k + 0.0085, gy, 0); guide.add(g1); guide.add(g2);
      const rod = cyl(0.0045, 0.11, m.steel); fol.add(rod);
      if (ftype === 'knife'){ const tip = new T.Mesh(new T.ConeGeometry(0.009, 0.02, 4), m.fol); tip.rotation.x = PI; tip.rotation.y = PI/4; tip.scale.z = 0.4; tip.position.y = 0.01; fol.add(tip); rod.position.y = 0.02 + 0.055; }
      else if (ftype === 'flat'){ const face = cyl(0.03, 0.005, m.fol); face.position.y = 0.0025; fol.add(face); rod.position.y = 0.005 + 0.055; }
      else if (ftype === 'sphere'){ const sp = new T.Mesh(new T.SphereGeometry(M.rr*k, 32, 16, 0, TAU, PI*0.62, PI*0.38), m.fol); sp.position.y = M.rr*k; fol.add(sp);
        const disc = cyl(Math.sin(PI*0.38)*M.rr*k, 0.002, m.fol); disc.position.y = M.rr*k - Math.cos(PI*0.38)*M.rr*k; fol.add(disc); rod.position.y = disc.position.y + 0.055; }
      else { const roll = cyl(M.rr*k, w*0.75, m.fol, 'z'); fol.add(roll); const fork = box(0.004, 0.02, w*1.05, m.steel); fork.position.y = 0.012; fol.add(fork); rod.position.y = 0.012 + 0.055; fol.add(cyl(0.002, w*1.15, m.steel, 'z')); }
      const yOff = ftype === 'knife' || ftype === 'flat' ? 0 : ftype === 'sphere' ? 0 : 0;
      step = t => { const [s] = M.prog(t); cam.rotation.z = -t; const y = ftype === 'knife' ? M.y0 + s : ftype === 'flat' ? base.Rb + s : (M.y0 + s) - (ftype === 'sphere' ? M.rr : 0); fol.position.set(M.e*k, y*k + yOff, 0); };
      if (ftype === 'roller' || ftype === 'offset') step = (t => { const [s] = M.prog(t); cam.rotation.z = -t; fol.position.set(M.e*k, (M.y0 + s)*k, 0); });
      if (ftype === 'sphere') step = (t => { const [s] = M.prog(t); cam.rotation.z = -t; fol.position.set(0, (M.y0 + s - M.rr)*k, 0); });
    }
    v.step = step; step(v.t || 0); const [ti, sub, cap] = FOL[ftype]; $('cf_t').textContent = ti; $('cf_sub').textContent = sub; $('cf_cap').textContent = cap; }
  if (v){ $('cf_chips').innerHTML = Object.entries(FOL).map(([kk, t]) => `<button type="button" data-f="${kk}" aria-pressed="${kk === 'knife'}">${t[0].replace(' follower', '')}</button>`).join('');
    $('cf_chips').querySelectorAll('button').forEach(b => b.addEventListener('click', () => { ftype = b.dataset.f; $('cf_chips').querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', x === b)); build(v); v.render(); }));
    playButton($('cf_play'), v.anim); } }

/* ================= 6.3 cylindrical cam ================= */
{ const P = { S:40, out:150, d1:30, ret:150, mo:'cyc', mr:'cyc' }, prog = program(P), k = 0.001, Rc = 40*k, Lc = 0.12, x0 = -0.02;
  const v = view3d($('cy'), { cam:[0.06, 0.12, 0.22], target:[0, 0.02, 0], fov:32,
    build(v){ const T = THREE, m = lightMats(v), cylG = new T.Group(); v.root.add(cylG); v.cg = cylG;
      cylG.add(cyl(Rc, Lc, m.cam, 'x', 48)); cylG.add(cyl(0.007, Lc + 0.06, m.steel, 'x'));
      class Path extends T.Curve { getPoint(u){ const f = u*TAU, [s] = prog(f); return new T.Vector3(x0 + s*k, (Rc - 0.0035)*Math.cos(f), (Rc - 0.0035)*Math.sin(f)); } }
      cylG.add(new T.Mesh(new T.TubeGeometry(new Path(), 500, 0.0055, 10, true), m.dark));
      const fol = new T.Group(); v.root.add(fol); v.fol = fol;
      const pin = cyl(0.005, 0.02, m.fol); pin.position.y = Rc + 0.006; fol.add(pin);
      const bar = box(0.1, 0.008, 0.012, m.fol); bar.position.set(0.03, Rc + 0.02, 0); fol.add(bar);
      const g1 = box(0.02, 0.016, 0.02, m.guide); g1.position.set(0.07, Rc + 0.02, 0); v.root.add(g1); },
    update(dt, v){ v.t = (v.t || 0) + dt*0.8; const [s] = prog(v.t); v.cg.rotation.x = -v.t; v.fol.position.x = x0 + s*k; } });
  if (v) playButton($('cy_play'), v.anim); }

/* ================= 6.4 terminology ================= */
{ const P = {}, cv = $('tm'); let sel = 'base';
  const TERMS = [['base', 'Base circle', 'The smallest circle that can be drawn to the cam profile.'],
    ['trace', 'Trace point', 'The point on the follower used to draw the pitch curve: the knife edge, or the centre of the roller.'],
    ['pitchc', 'Pitch curve', 'The path of the trace point as the follower goes round the (imagined stationary) cam. For a roller it lies one roller radius outside the cam profile.'],
    ['prime', 'Prime circle', 'The smallest circle from the cam centre that touches the pitch curve: base circle + roller radius.'],
    ['pa', 'Pressure angle', 'The angle between the follower’s direction of motion and the normal to the pitch curve. Too large and a sliding follower jams in its guide.'],
    ['pp', 'Pitch point', 'The point on the pitch curve where the pressure angle is greatest.'],
    ['pcirc', 'Pitch circle', 'The circle through the pitch point, centred on the cam centre.'],
    ['lift', 'Lift (stroke)', 'The greatest travel of the follower from its lowest to its highest position.']];
  $('tm_chips').innerHTML = TERMS.map(([k2, n]) => `<button type="button" data-k="${k2}" aria-pressed="${k2 === sel}">${n}</button>`).join('');
  $('tm_chips').querySelectorAll('button').forEach(b => b.addEventListener('click', () => { sel = b.dataset.k; $('tm_chips').querySelectorAll('button').forEach(x => x.setAttribute('aria-pressed', x === b)); draw(); }));
  controls($('tm_c'), [['Rb', 'Base circle radius', 15, 60, 1, 25, 'mm'], ['rr', 'Roller radius', 3, 20, 1, 10, 'mm'], ['th', 'Cam turned through', 0, 360, 1, 50, '°']], P, () => draw());
  function draw(){ const M = camModel({ S:50, out:120, d1:30, ret:60, mo:'shm', mr:'shm', Rb:P.Rb, rr:P.rr, e:0, type:'roller', N:100 });
    const R = M.Rmax + 2*P.rr + 10, V = view(cv, { xl:-R, xr:R, yb:-R*0.85, yt:R + 30 }), { ctx } = V, a = -P.th*DEG, hl = k2 => sel === k2;
    poly(ctx, V, M.prof, true, alpha(C.crank, 0.18), C.crank, 2, a);
    if (hl('pitchc') || hl('pa') || hl('pp') || hl('trace')) poly(ctx, V, M.pitch, true, null, C.force, hl('pitchc') ? 2.6 : 1.2, a); else { ctx.setLineDash([4, 4]); poly(ctx, V, M.pitch, true, null, alpha(C.force, 0.5), 1, a); ctx.setLineDash([]); }
    circ(ctx, V, 0, 0, P.Rb, hl('base') ? C.acc : alpha(C.muted, 0.6), hl('base') ? [] : [3, 3], hl('base') ? 2.6 : 1);
    circ(ctx, V, 0, 0, P.Rb + P.rr, hl('prime') ? C.acc : alpha(C.muted, 0.5), hl('prime') ? [] : [2, 4], hl('prime') ? 2.6 : 1);
    const pp = M.pitch[M.paAt], [ppx, ppy] = rot(pp[0], pp[1], a), rP = Math.hypot(pp[0], pp[1]);
    if (hl('pcirc') || hl('pp')) circ(ctx, V, 0, 0, rP, C.vel, [], hl('pcirc') ? 2.4 : 1.2);
    // follower
    const i = Math.round(((P.th % 360) + 360) % 360/360*M.N), s = M.s[i], cy0 = M.y0 + s;
    circ(ctx, V, 0, cy0, P.rr, C.rod, [], 2); ctx.fillStyle = alpha(C.rod, 0.2); ctx.beginPath(); ctx.arc(V.PX(0), V.PY(cy0), P.rr*V.sc, 0, 7); ctx.fill();
    ctx.fillStyle = alpha(C.rod, 0.35); ctx.fillRect(V.PX(-4), V.PY(cy0 + 60), 8*V.sc, 60*V.sc - P.rr*V.sc*0.2); ctx.strokeStyle = C.rod; ctx.strokeRect(V.PX(-4), V.PY(cy0 + 60), 8*V.sc, (60 - 0)*V.sc);
    dot(ctx, V.PX(0), V.PY(cy0), hl('trace') ? C.acc : C.ink, hl('trace') ? 6 : 3.5); dot(ctx, V.PX(0), V.PY(0), C.ink, 3.5); lbl(ctx, 'O', V.PX(0) + 8, V.PY(0) + 12, C.ink);
    if (hl('trace')) lbl(ctx, 'trace point', V.PX(0) + 14, V.PY(cy0), C.acc);
    if (hl('lift')){ const x = V.PX(R*0.62); ctx.strokeStyle = C.acc; ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(V.PX(0), V.PY(M.y0)); ctx.lineTo(x + 10, V.PY(M.y0)); ctx.moveTo(V.PX(0), V.PY(M.y0 + 50)); ctx.lineTo(x + 10, V.PY(M.y0 + 50)); ctx.stroke(); ctx.setLineDash([]);
      arrow(ctx, x, V.PY(M.y0 + 25), x, V.PY(M.y0), C.acc, 1.8); arrow(ctx, x, V.PY(M.y0 + 25), x, V.PY(M.y0 + 50), C.acc, 1.8); lbl(ctx, 'lift 50 mm', x + 8, V.PY(M.y0 + 25), C.acc); }
    if (hl('pa') || hl('pp')){ const j = hl('pp') ? M.paAt : i, p = M.pitch[j], [px, py] = rot(p[0], p[1], a), tt = M.th[j], d = rot(0, 1, a + tt);
      const p1 = M.pitch[Math.min(j + 1, M.N)], p0 = M.pitch[Math.max(j - 1, 0)], T = [p1[0] - p0[0], p1[1] - p0[1]], Ln = Math.hypot(...T) || 1; let n = [T[1]/Ln, -T[0]/Ln]; if (n[0]*p[0] + n[1]*p[1] < 0) n = [-n[0], -n[1]]; const [nx, ny] = rot(n[0], n[1], a);
      arrow(ctx, V.PX(px), V.PY(py), V.PX(px + nx*40), V.PY(py + ny*40), C.force, 2); arrow(ctx, V.PX(px), V.PY(py), V.PX(px + d[0]*40), V.PY(py + d[1]*40), C.vel, 2);
      lbl(ctx, `pressure angle ${n2(M.pa[j], 1)}°`, V.PX(px) + 12, V.PY(py) - 28, C.force); dot(ctx, V.PX(px), V.PY(py), C.acc, 5);
      if (hl('pp')) lbl(ctx, 'pitch point', V.PX(px) + 12, V.PY(py) + 4, C.acc); }
    const t = TERMS.find(x => x[0] === sel); $('tm_cap').innerHTML = `<b>${t[1]}.</b> ${t[2]}`; }
  watchSize(cv, draw); onRedraw(draw); draw(); }

/* ================= 6.5 displacement diagram constructions ================= */
{ const cv = $('dc'); let mo = 'shm', t = 0;
  document.querySelectorAll('[data-mo]').forEach(b => b.addEventListener('click', () => { mo = b.dataset.mo; t = 0; document.querySelectorAll('[data-mo]').forEach(x => x.setAttribute('aria-pressed', x === b)); draw(); }));
  function draw(){ const n = 8, S = 1, W0 = 2.2, V = view(cv, { xl:-0.85, xr:W0 + 0.15, yb:-0.18, yt:1.12 }, 12), { ctx } = V, f = MOT[mo].f;
    const k = Math.min(n, Math.floor(t*2.2)), X = i => V.PX(W0*i/n), Y = y => V.PY(y);
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1; for (let i = 0; i <= n; i++){ ctx.beginPath(); ctx.moveTo(X(i), Y(0)); ctx.lineTo(X(i), Y(S)); ctx.stroke(); }
    ctx.strokeStyle = C.ink; ctx.beginPath(); ctx.moveTo(X(0), Y(S)); ctx.lineTo(X(0), Y(0)); ctx.lineTo(X(n), Y(0)); ctx.stroke(); ctx.strokeStyle = C.rule; ctx.beginPath(); ctx.moveTo(X(0), Y(S)); ctx.lineTo(X(n), Y(S)); ctx.stroke();
    for (let i = 0; i <= n; i++) lbl(ctx, String(i), X(i), Y(0) + 12, C.muted, 'center', 11);
    lbl(ctx, 'cam angle during the outstroke θ_O, in 8 equal parts', X(n/2), Y(0) + 28, C.muted, 'center', 11.5); lbl(ctx, 'S', X(0) - 10, Y(S/2), C.ink, 'right', 13);
    const pts = []; for (let i = 0; i <= n; i++) pts.push([X(i), Y(S*f(i/n))]);
    if (mo === 'shm'){ const cx = V.PX(-0.45), r = (Y(0) - Y(S))/2, cy = (Y(0) + Y(S))/2; ctx.strokeStyle = C.crank; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(cx, cy, r, PI/2, 3*PI/2, false); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx, cy - r); ctx.lineTo(cx, cy + r); ctx.stroke();
      for (let i = 0; i <= n; i++){ const a = PI*i/n, px = cx - r*Math.sin(a), py = cy + r*Math.cos(a); dot(ctx, px, py, C.crank, 3); if (i <= k){ ctx.strokeStyle = alpha(C.crank, 0.6); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(px, py); ctx.lineTo(pts[i][0], py); ctx.stroke(); ctx.setLineDash([]); } }
      $('dc_cap').innerHTML = 'Draw a semicircle on the stroke as diameter and divide it into as many equal parts as the outstroke angle. Project each division across to the matching vertical line. The follower moves like the projection of a point going round the circle at constant speed.';
    } else if (mo === 'uarm'){ const h = n/2; for (let j = 1; j <= h; j++){ const yy = Y(S*j/n); dot(ctx, X(h), yy, C.rod, 3); if (j <= k){ ctx.strokeStyle = alpha(C.rod, 0.6); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(0), Y(0)); ctx.lineTo(X(h), yy); ctx.stroke(); ctx.setLineDash([]); } }
      for (let j = 1; j <= h; j++){ const yy = Y(S - S*j/n); if (j <= k - h + 4){ ctx.strokeStyle = alpha(C.rod, 0.6); ctx.setLineDash([3, 3]); ctx.beginPath(); ctx.moveTo(X(n), Y(S)); ctx.lineTo(X(h), yy); ctx.stroke(); ctx.setLineDash([]); } }
      $('dc_cap').innerHTML = 'Divide the stroke and the outstroke angle into the same even number of parts. In the first half, join the start A to the points on the middle vertical; where line Aa crosses vertical 1, Ab crosses 2 and so on are points on a parabola (uniform acceleration). The second half is the same parabola turned round (uniform retardation).';
    } else if (mo === 'cyc'){ const r = (Y(0) - Y(S))/TAU, cx = X(0), cy = Y(0) - r; ctx.strokeStyle = C.vel; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(cx - r - 6, cy, r, 0, 7); ctx.stroke();
      ctx.strokeStyle = alpha(C.vel, 0.5); ctx.setLineDash([4, 3]); ctx.beginPath(); ctx.moveTo(X(0), Y(0)); ctx.lineTo(X(n), Y(S)); ctx.stroke(); ctx.setLineDash([]);
      for (let i = 0; i <= Math.min(k, n); i++){ const u = i/n, lin = Y(S*u); ctx.strokeStyle = alpha(C.vel, 0.7); ctx.beginPath(); ctx.moveTo(X(i), lin); ctx.lineTo(pts[i][0], pts[i][1]); ctx.stroke(); }
      $('dc_cap').innerHTML = 'A circle of circumference S (radius S/2π) rolls up the stroke. The diagonal A to B is uniform velocity; the cycloid departs from it by (S/2π) sin(2πθ/θ<sub>O</sub>), the projected position of a point on the rolling circle (short green lines). The curve leaves and arrives tangent to the horizontal, with no jump in acceleration.';
    } else { $('dc_cap').innerHTML = 'The displacement is a straight line: equal rise for each equal turn of the cam. The corners at the start and end are sharp, which means an instant change of velocity and infinite acceleration. In practice the corners are rounded with short parabolas.'; }
    ctx.strokeStyle = C.acc; ctx.lineWidth = 3; ctx.beginPath(); for (let i = 0; i <= 80*Math.min(1, k/n); i++){ const u = i/80, x = V.PX(W0*u), y = Y(S*f(u)); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); } ctx.stroke();
    for (let i = 0; i <= k; i++) dot(ctx, pts[i][0], pts[i][1], C.acc, 4);
    $('dc_sub').textContent = MOT[mo].name; }
  const an = animate(cv, dt => { t += dt; if (t > 6) t = 0; draw(); }); playButton($('dc_play'), an); watchSize(cv, draw); onRedraw(draw); draw(); }

/* comparison of the four motions */
{ const P = {}, cv1 = $('mc1'), cv2 = $('mc2');
  controls($('mc_c'), [['S', 'Stroke S', 5, 100, 1, 40, 'mm'], ['b', 'Outstroke angle θ_O', 30, 270, 5, 100, '°'], ['N', 'Cam speed', 10, 3000, 10, 900, 'rpm']], P, () => draw());
  function draw(){ const w = P.N*TAU/60, b = P.b*DEG, S = P.S/1000, x = [], cols = { uv:C.muted, shm:C.crank, uarm:C.rod, cyc:C.vel };
    for (let i = 0; i <= 200; i++) x.push(P.b*i/200);
    const ser = q => Object.keys(MOT).map(k2 => ({ y:x.map(t => { const u = t/P.b; return q === 1 ? S*w/b*MOT[k2].d1(u) : (k2 === 'uv' ? 0 : S*w*w/(b*b)*MOT[k2].d2(u)); }), color:cols[k2], w:k2 === 'uv' ? 1.5 : 2 }));
    chart(cv1, { x, xlab:'cam angle [°]', ylab:'v [m/s]', series:ser(1), value:'velocity' });
    chart(cv2, { x, xlab:'cam angle [°]', ylab:'a [m/s²]', series:ser(2), value:'acceleration (uniform velocity: infinite spikes at the ends)' });
    $('mc_sub').textContent = `S = ${P.S} mm, θ_O = ${P.b}°, ${P.N} rpm (ω = ${n2(w)} rad/s)`;
    $('mc_t').innerHTML = '<thead><tr><th>Motion</th><th>Peak velocity</th><th>Peak acceleration</th><th>Acceleration at the ends</th></tr></thead><tbody>' + Object.entries(MOT).map(([k2, m]) => `<tr><td>${m.name}</td><td class="n">${n2(m.cv*S*w/b, 3)} m/s</td><td class="n">${isFinite(m.ca) ? n2(m.ca*S*w*w/(b*b), 1) + ' m/s²' : 'infinite'}</td><td>${k2 === 'cyc' ? 'zero (smooth)' : k2 === 'uv' ? 'infinite' : 'jumps'}</td></tr>`).join('') + '</tbody>'; }
  watchSize(cv1, draw); watchSize(cv2, draw); onRedraw(draw); draw(); }

/* ================= 6.6 cam profile tool ================= */
const CT = { view:'build', step:0, t:0 };
const CP = {}; let CM;
const ctCtl = controls($('ct_c'), [['Rb', 'Least radius of cam (base circle)', 10, 80, 1, 50, 'mm'], ['S', 'Stroke (lift)', 5, 80, 1, 40, 'mm'], ['out', 'Outstroke angle', 30, 200, 5, 60, '°'], ['d1', 'Dwell after the outstroke', 0, 150, 5, 30, '°'],
  ['ret', 'Return angle', 30, 200, 5, 60, '°'], ['rr', 'Roller radius', 3, 25, 1, 10, 'mm'], ['e', 'Offset of the follower', -30, 30, 1, 0, 'mm'], ['N', 'Cam speed', 10, 2000, 10, 100, 'rpm'], ['n', 'Divisions per stroke', 4, 12, 2, 6]], CP, () => ctRebuild());
['ct_f', 'ct_mo', 'ct_mr'].forEach(id => $(id).addEventListener('change', () => ctRebuild()));
document.querySelectorAll('[data-v]').forEach(b => b.addEventListener('click', () => { CT.view = b.dataset.v; document.querySelectorAll('[data-v]').forEach(x => x.setAttribute('aria-pressed', x === b)); CT.step = 0; ctDraw(); }));
$('ct_restart').addEventListener('click', () => { CT.view = 'build'; document.querySelectorAll('[data-v]').forEach(x => x.setAttribute('aria-pressed', x.dataset.v === 'build')); CT.step = 0; ctAnim.playing = true; $('ct_play').textContent = 'Pause'; });
function ctRebuild(){ if (CP.out + CP.d1 + CP.ret > 360){ CP.d1 = Math.max(0, 360 - CP.out - CP.ret); ctCtl.d1.set(CP.d1); }
  CM = camModel({ ...CP, mo:$('ct_mo').value, mr:$('ct_mr').value, type:$('ct_f').value }); ctStats(); ctDraw(); ctCharts(); }
function ctStats(){ const M = CM, P = M.P, w = M.w, S = P.S/1000, bo = P.out*DEG, br = P.ret*DEG, mo = MOT[P.mo], mr = MOT[P.mr];
  const st = [['Max velocity, outstroke', `${n2(mo.cv*S*w/bo, 3)} m/s`], ['Max velocity, return', `${n2(mr.cv*S*w/br, 3)} m/s`], ['Max acceleration, outstroke', isFinite(mo.ca) ? `${n2(mo.ca*S*w*w/bo/bo, 2)} m/s²` : 'infinite'], ['Max acceleration, return', isFinite(mr.ca) ? `${n2(mr.ca*S*w*w/br/br, 2)} m/s²` : 'infinite'],
    ['Largest pressure angle', M.type === 'flat' ? '0° (flat face)' : `${n2(M.paMax, 1)}°`], [M.type === 'flat' ? 'Face width needed' : 'Smallest radius of pitch curve', M.type === 'flat' ? `${n2(2*M.flatW + 4, 0)} mm` : `${isFinite(M.rhoMin) ? n2(M.rhoMin, 1) : '∞'} mm`], ['Largest cam radius', `${n2(M.Rmax, 1)} mm`]];
  $('ct_s').innerHTML = st.map(([k2, v2]) => `<div class="stat${(k2.startsWith('Largest pressure') && M.paMax > 30) || (M.cut && k2.startsWith('Smallest')) ? ' bad' : ''}"><div class="k">${k2}</div><div class="v">${v2}</div></div>`).join('');
  const warn = []; if (!M.ok) warn.push('The offset must be smaller than the prime circle radius.'); if (M.cut) warn.push(M.type === 'roller' ? `The roller (${P.rr} mm) is bigger than the sharpest part of the pitch curve (${n2(M.rhoMin, 1)} mm): the cam would come to a point and the follower could not follow the motion. Use a smaller roller or a bigger base circle.` : 'This motion needs a hollow in the cam, which a flat follower cannot reach. Use a bigger base circle.');
  if (M.type !== 'flat' && M.paMax > 30) warn.push(`Pressure angle reaches ${n2(M.paMax, 0)}° (above about 30°): a sliding follower may jam. Increase the base circle or spread the stroke over a larger angle.`);
  $('ct_status').textContent = warn.length ? 'check the design' : 'design is fine'; $('ct_status').style.color = warn.length ? C.bad : C.ok; CT.warn = warn; }
function ctCharts(){ const M = CM, x = [], s = [], v = [], a = [], w = M.w; for (let i = 0; i < M.N; i += 2){ x.push(M.th[i]/DEG); s.push(M.s[i]); v.push(M.v[i]*w/1000); a.push(M.a[i]*w*w/1000); }
  const cur = CT.view === 'run' ? ((CT.t/DEG) % 360) : null, ci = cur == null ? null : Math.round(cur/360*(x.length - 1));
  const shade = [{ a:0, b:M.P.out, color:C.acc, label:'out' }, { a:M.P.out + M.P.d1, b:M.P.out + M.P.d1 + M.P.ret, color:C.vel, label:'return' }];
  chart($('c1'), { x, xlab:'cam angle [°]', ylab:'s [mm]', series:[{ y:s, color:C.crank }], shade, cursor:cur, cursorIndex:ci, value:'' });
  chart($('c2'), { x, xlab:'cam angle [°]', ylab:'v [m/s]', series:[{ y:v, color:C.vel }], shade, cursor:cur, cursorIndex:ci, value:'' });
  chart($('c3'), { x, xlab:'cam angle [°]', ylab:'a [m/s²]', series:[{ y:a, color:C.acc }], shade, cursor:cur, cursorIndex:ci, value:'' });
  if (ci != null){ $('c1v').textContent = `${n2(s[ci], 2)} mm`; $('c2v').textContent = `${n2(v[ci], 3)} m/s`; $('c3v').textContent = `${n2(a[ci], 1)} m/s²`; } else ['c1v', 'c2v', 'c3v'].forEach(id => $(id).textContent = `at ${M.P.N} rpm`); }
const LET = 'ABCDEFGHJKLMNPQRSTUVWXYZ';
function ctDraw(){ const M = CM, P = M.P, cv = $('ct'); if (!M) return;
  const R = Math.max(M.Rmax, P.Rb + P.S + M.rr) + (M.type === 'flat' ? 10 : M.rr) + 18, V = view(cv, { xl:-R - 10, xr:R + 10, yb:-R, yt:R + (CT.view === 'run' ? 34 : 8) }), { ctx } = V;
  const n = P.n, o = P.out, d = P.d1, r = P.ret, rr = M.rr;
  if (CT.view === 'build'){
    const total = 2*(n + 1) + 3, k = Math.min(total, CT.step);   // steps: circles+sectors, rise points, return points, profile
    circ(ctx, V, 0, 0, P.Rb, C.ink, [], 1.4); if (rr) circ(ctx, V, 0, 0, M.Rp, alpha(C.muted, 0.9), [6, 3], 1.2); if (M.e) circ(ctx, V, 0, 0, Math.abs(M.e), alpha(C.muted, 0.9), [2, 3], 1.2);
    lbl(ctx, 'base circle', V.PX(-P.Rb*0.7), V.PY(-P.Rb*0.72), C.muted, 'center', 11); if (rr) lbl(ctx, 'prime circle', V.PX(-M.Rp*0.72), V.PY(-M.Rp*0.72) + 14, C.muted, 'center', 11);
    // follower axis and A
    const lineAt = (t, len) => { const p0 = rot(M.e, 0, t), dd = rot(0, 1, t); return [p0, [p0[0] + dd[0]*len, p0[1] + dd[1]*len]]; };
    const ptAt = (t, s) => rot(M.e, M.y0 + s, t);
    // sector marks: angles measured anticlockwise from OA (cam turns clockwise)
    const marks = [[0, 'A'], [o, 'S'], [o + d, 'T'], [o + d + r, 'P']]; const big = R*0.95;
    ctx.strokeStyle = alpha(C.muted, 0.6); ctx.lineWidth = 1;
    const bounds = [0, o, o + d, o + d + r];
    for (const b of bounds){ const [p0, p1] = lineAt(b*DEG, big); ctx.beginPath(); ctx.moveTo(V.PX(p0[0]), V.PY(p0[1])); ctx.lineTo(V.PX(p1[0]), V.PY(p1[1])); ctx.stroke(); }
    const arcLbl = (a0, a1, txt, rad) => { ctx.strokeStyle = C.ink; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.arc(V.PX(0), V.PY(0), rad*V.sc, -(PI/2 + a1*DEG), -(PI/2 + a0*DEG), false); ctx.stroke(); const am = (a0 + a1)/2*DEG + PI/2; lbl(ctx, txt, V.PX(rad*1.25*Math.cos(am)), V.PY(rad*1.25*Math.sin(am)), C.ink, 'center', 11); };
    arcLbl(0, o, `${o}°`, P.Rb*0.32); if (d) arcLbl(o, o + d, `${d}°`, P.Rb*0.22); arcLbl(o + d, o + d + r, `${r}°`, P.Rb*0.32);
    // division lines and points
    const rise = [], fall = [];
    for (let i = 0; i <= n; i++){ rise.push(o*DEG*i/n); fall.push((o + d)*DEG + r*DEG*i/n); }
    const drawSet = (angs, startIdx, numPrefix, letterOff) => { angs.forEach((t, i) => { const idx = startIdx + i; if (idx > k) return; const [s] = M.prog(t + 1e-9*(i === angs.length - 1 ? -1 : 1));
        const [p0, p1] = lineAt(t, M.y0 + P.S + rr + 12); ctx.strokeStyle = alpha(C.crank, 0.55); ctx.lineWidth = 1; ctx.beginPath(); ctx.moveTo(V.PX(p0[0]), V.PY(p0[1])); ctx.lineTo(V.PX(p1[0]), V.PY(p1[1])); ctx.stroke();
        const q0 = ptAt(t, 0), q = ptAt(t, s); dot(ctx, V.PX(q0[0]), V.PY(q0[1]), C.crank, 2.6); lbl(ctx, `${i}${numPrefix}`, V.PX(q0[0]*0.86), V.PY(q0[1]*0.86), C.crank, 'center', 10.5);
        ctx.strokeStyle = C.acc; ctx.lineWidth = 2.4; ctx.beginPath(); ctx.moveTo(V.PX(q0[0]), V.PY(q0[1])); ctx.lineTo(V.PX(q[0]), V.PY(q[1])); ctx.stroke(); dot(ctx, V.PX(q[0]), V.PY(q[1]), C.acc, 3.5);
        const L = LET[letterOff + i]; if (L) lbl(ctx, L, V.PX(q[0]*1.0) + (q[0] >= 0 ? 10 : -10), V.PY(q[1]) - 9, C.ink, q[0] >= 0 ? 'left' : 'right', 11);
        if (M.type === 'roller'){ circ(ctx, V, q[0], q[1], rr, alpha(C.rod, 0.7), [], 1.1); }
        if (M.type === 'flat'){ const dd = rot(1, 0, t), h = 22; ctx.strokeStyle = alpha(C.rod, 0.8); ctx.lineWidth = 1.4; const qq = rot(0, P.Rb + s, t); ctx.beginPath(); ctx.moveTo(V.PX(qq[0] - dd[0]*h), V.PY(qq[1] - dd[1]*h)); ctx.lineTo(V.PX(qq[0] + dd[0]*h), V.PY(qq[1] + dd[1]*h)); ctx.stroke(); } }); };
    if (k >= 1) drawSet(rise, 1, '', 0);
    if (k >= n + 2) drawSet(fall, n + 2, "′", n + 1);
    if (k >= total){ poly(ctx, V, M.prof, true, alpha(C.crank, 0.16), C.crank, 2.6); if (M.type === 'roller'){ ctx.setLineDash([6, 4]); poly(ctx, V, M.pitch, true, null, C.force, 1.4); ctx.setLineDash([]); } }
    dot(ctx, V.PX(0), V.PY(0), C.ink, 3.5); lbl(ctx, 'O', V.PX(0) + 8, V.PY(0) + 12, C.ink);
    const stepsTxt = k < 1 ? `Draw the base circle (radius ${P.Rb} mm)${rr ? ` and the prime circle (base + roller = ${M.Rp} mm)` : ''}${M.e ? `, and an offset circle of radius ${Math.abs(M.e)} mm` : ''}. From OA mark ${o}° for the outstroke, ${d}° dwell and ${r}° return, going <b>anticlockwise</b>, because the cam turns clockwise.`
      : k < n + 2 ? `Divide the outstroke angle into ${n} equal parts. ${M.e ? 'Draw tangents to the offset circle' : 'Draw radial lines'} through the points 1, 2, 3… and set off on each the displacement 1B, 2C, 3D… from the displacement diagram (${MOT[P.mo].name}).`
      : k < total ? `Do the same for the return stroke: points 0′, 1′, 2′… and the displacements H, J, K… (${MOT[P.mr].name}).`
      : M.type === 'roller' ? 'Join the points with a smooth curve: this is the <b>pitch curve</b> (dashed). Draw a roller circle at each point; the curve touching the inside of all the circles is the <b>cam profile</b>.' : M.type === 'flat' ? 'Draw the follower face at each point, square to its line. The smooth curve that touches every face is the <b>cam profile</b>.' : 'Join A, B, C … P with a smooth curve. For a knife edge this is the <b>cam profile</b>.';
    $('ct_cap').innerHTML = stepsTxt + (CT.warn && CT.warn.length ? `<br><span style="color:var(--bad)">${CT.warn.join(' ')}</span>` : '');
  } else {
    const t = CT.t, a = -t, [s, s1] = M.prog(t);
    poly(ctx, V, M.prof, true, alpha(C.crank, 0.2), C.crank, 2.2, a); circ(ctx, V, 0, 0, P.Rb*0.18, C.ink, [], 1.2);
    const mk = rot(0, P.Rb*0.8, a); ctx.strokeStyle = C.ink; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(V.PX(0), V.PY(0)); ctx.lineTo(V.PX(mk[0]), V.PY(mk[1])); ctx.stroke();
    // follower
    const fx = M.e, base = M.type === 'flat' ? P.Rb + s : M.y0 + s, top = R + 30;
    ctx.fillStyle = alpha(C.rod, 0.3); ctx.strokeStyle = C.rod; ctx.lineWidth = 1.5;
    if (M.type === 'knife'){ ctx.beginPath(); ctx.moveTo(V.PX(fx), V.PY(base)); ctx.lineTo(V.PX(fx - 6), V.PY(base + 12)); ctx.lineTo(V.PX(fx - 6), V.PY(top)); ctx.lineTo(V.PX(fx + 6), V.PY(top)); ctx.lineTo(V.PX(fx + 6), V.PY(base + 12)); ctx.closePath(); ctx.fill(); ctx.stroke(); }
    else if (M.type === 'roller'){ ctx.fillRect(V.PX(fx - 5), V.PY(top), 10*V.sc, (top - base)*V.sc); ctx.strokeRect(V.PX(fx - 5), V.PY(top), 10*V.sc, (top - base)*V.sc); ctx.beginPath(); ctx.arc(V.PX(fx), V.PY(base), rr*V.sc, 0, 7); ctx.fillStyle = alpha(C.rod, 0.45); ctx.fill(); ctx.stroke(); dot(ctx, V.PX(fx), V.PY(base), C.ink, 2.5); }
    else { const hw = Math.max(M.flatW + 4, 20); ctx.fillRect(V.PX(-hw), V.PY(base + 5), 2*hw*V.sc, 5*V.sc); ctx.strokeRect(V.PX(-hw), V.PY(base + 5), 2*hw*V.sc, 5*V.sc); ctx.fillRect(V.PX(-5), V.PY(top), 10*V.sc, (top - base - 5)*V.sc); ctx.strokeRect(V.PX(-5), V.PY(top), 10*V.sc, (top - base - 5)*V.sc);
      const cx = V.PX(-s1); dot(ctx, cx, V.PY(base), C.acc, 4); }
    ctx.fillStyle = alpha(C.muted, 0.35); ctx.fillRect(V.PX(fx - 13), V.PY(top - 8), 4*V.sc, 26*V.sc); ctx.fillRect(V.PX(fx + 9), V.PY(top - 8), 4*V.sc, 26*V.sc);
    dot(ctx, V.PX(0), V.PY(0), C.ink, 3.5);
    ctx.strokeStyle = C.muted; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(V.PX(0), V.PY(0), P.Rb*0.35*V.sc, -PI*0.15, -PI*0.6, true); ctx.stroke(); lbl(ctx, 'ω (clockwise)', V.PX(P.Rb*0.42), V.PY(P.Rb*0.42), C.muted, 'left', 11);
    const i = Math.round(((t % TAU) + TAU) % TAU/TAU*M.N), seg = M.prog(t)[3];
    $('ct_cap').innerHTML = `Cam at ${Math.round(((t/DEG) % 360 + 360) % 360)}°: ${seg === 'out' ? 'outstroke' : seg === 'ret' ? 'return stroke' : 'dwell'}, follower lifted ${n2(s, 1)} mm${M.type !== 'flat' ? `, pressure angle ${n2(M.pa[i], 1)}°` : `, contact ${n2(Math.abs(s1), 1)} mm from the follower axis`}.` + (CT.warn && CT.warn.length ? `<br><span style="color:var(--bad)">${CT.warn.join(' ')}</span>` : '');
  } }
const ctAnim = animate($('ct'), dt => { if (CT.view === 'build'){ CT.acc = (CT.acc || 0) + dt; if (CT.acc > 0.55){ CT.acc = 0; CT.step = Math.min(CT.step + 1, 2*(CP.n + 1) + 3 + 3); if (CT.step >= 2*(CP.n + 1) + 6) CT.step = 2*(CP.n + 1) + 3; ctDraw(); } }
  else { CT.t += dt*Math.min(2.5, CM.w*0.15 + 0.6); ctDraw(); ctCharts(); } });
playButton($('ct_play'), ctAnim); watchSize($('ct'), ctDraw); ['c1', 'c2', 'c3'].forEach(id => watchSize($(id), ctCharts)); onRedraw(() => { ctStats(); ctDraw(); ctCharts(); });
function loadCam(o){ ['Rb', 'S', 'out', 'd1', 'ret', 'rr', 'e', 'N'].forEach(k2 => { if (o[k2] != null) ctCtl[k2].set(o[k2]); }); $('ct_f').value = o.f; $('ct_mo').value = o.mo; $('ct_mr').value = o.mr; ctCtl.n.set(6);
  CT.view = 'build'; CT.step = 0; document.querySelectorAll('[data-v]').forEach(x => x.setAttribute('aria-pressed', x.dataset.v === 'build')); ctAnim.playing = true; $('ct_play').textContent = 'Pause'; ctRebuild(); $('s6').scrollIntoView({ behavior:reduceMotion.matches ? 'auto' : 'smooth' }); }
ctRebuild();

/* ================= worked examples ================= */
const btns = list => '<div class="btnrow" style="padding:10px 0 0;border:0">' + list.map(([id, t]) => `<button type="button" class="primary" id="${id}">${t}</button>`).join('') + '</div>';
example($('ex1'), { title:'Example 1', src:'knife-edge, uniform velocity (notes p. 118; Khurmi 20.1)',
  q:'A cam is to give a knife-edged follower: outstroke during 60° of cam rotation; dwell for the next 30°; return stroke during the next 60°; dwell for the remaining 210°. Stroke 40 mm, minimum radius of the cam 50 mm. The follower moves with uniform velocity both ways. Draw the profile when (a) the follower axis passes through the cam shaft axis, (b) it is offset by 20 mm.',
  inputs:[['S', 'Stroke', 40, 'mm'], ['out', 'Outstroke', 60, '°'], ['ret', 'Return', 60, '°'], ['N', 'Cam speed (for the velocity only)', 100, 'rpm']],
  solve:v => { const w = v.N*TAU/60; return [['Displacement grows by S/6 for each 10° of cam:', `${n2(v.S/6, 2)} mm per division`], ['Follower velocity on the outstroke = Sω/θ<sub>O</sub> =', `${n2(v.S/1000*w/(v.out*DEG), 3)} m/s at ${v.N} rpm`, true], ['Acceleration at the start and end of each stroke:', 'infinite (sharp corners)', true]]; },
  after:btns([['ld1a', 'Draw (a) radial follower'], ['ld1b', 'Draw (b) 20 mm offset']]) });
example($('ex2'), { title:'Example 2', src:'roller follower, simple harmonic (notes p. 121; Khurmi 20.3)',
  q:'A cam, minimum radius 25 mm, rotating clockwise, gives a roller follower at the end of a valve rod: raise the valve 50 mm during 120°; keep it raised for 30°; lower it during 60°; keep it closed for the rest (150°). Roller diameter 20 mm. Draw the profile (a) with the line of stroke through the cam axis, (b) offset 15 mm. Simple harmonic motion both ways. Find the maximum acceleration of the valve rod at 100 rpm.',
  inputs:[['S', 'Stroke', 50, 'mm'], ['out', 'Outstroke', 120, '°'], ['ret', 'Return', 60, '°'], ['N', 'Cam speed', 100, 'rpm']],
  solve:v => { const w = v.N*TAU/60, bo = v.out*DEG, br = v.ret*DEG, S = v.S/1000;
    return [['ω = 2πN/60 =', `${n2(w)} rad/s`], ['θ<sub>O</sub>, θ<sub>R</sub> =', `${n2(bo, 3)}, ${n2(br, 3)} rad`], ['v<sub>O</sub> = πωS/(2θ<sub>O</sub>) =', `${n2(PI*w*S/(2*bo), 3)} m/s`], ['v<sub>R</sub> = πωS/(2θ<sub>R</sub>) =', `${n2(PI*w*S/(2*br), 3)} m/s`],
      ['a<sub>O</sub> = π²ω²S/(2θ<sub>O</sub>²) =', `${n2(PI*PI*w*w*S/(2*bo*bo), 2)} m/s²`, true], ['a<sub>R</sub> = π²ω²S/(2θ<sub>R</sub>²) =', `${n2(PI*PI*w*w*S/(2*br*br), 2)} m/s²`, true]]; },
  after:btns([['ld2a', 'Draw (a) radial roller'], ['ld2b', 'Draw (b) 15 mm offset']]) });
example($('ex3'), { title:'Example 3', src:'flat-faced follower, simple harmonic (notes p. 124; Khurmi 20.4)',
  q:'A cam drives a flat reciprocating follower: during the first 120° the follower moves out 20 mm with simple harmonic motion; it dwells for 30°; during the next 120° it moves in with simple harmonic motion; it dwells for the last 90°. Minimum radius of the cam 25 mm. Draw the profile.',
  inputs:[['S', 'Stroke', 20, 'mm'], ['out', 'Outstroke', 120, '°'], ['Rb', 'Least radius', 25, 'mm']],
  solve:v => { const k2 = v.S*PI/2/(v.out*DEG); return [['Largest ds/dθ = πS/(2θ<sub>O</sub>) =', `${n2(k2, 2)} mm`], ['So the contact moves up to this far each side of the follower axis; face at least', `${n2(2*k2 + 4, 0)} mm wide`, true], ['Radius of curvature of the profile = R<sub>b</sub> + s + d²s/dθ², smallest value', `${n2(v.Rb + Math.min(...Array.from({ length:61 }, (_, i) => { const u = i/60; return v.S*(1 - Math.cos(PI*u))/2 + v.S*PI*PI/2*Math.cos(PI*u)/((v.out*DEG)**2); })), 1)} mm (must stay positive)`, true]]; },
  after:btns([['ld3', 'Draw it']]) });
example($('ex4'), { title:'Example 4', src:'knife-edge, uniform acceleration and retardation (notes p. 126; Khurmi 20.6)',
  q:'A cam, minimum radius 50 mm, rotating clockwise, gives a knife-edge follower: out 40 mm during 100°; dwell 80°; return during 90°; dwell for the remaining 90°. Uniform acceleration and retardation. Draw the profile (i) with the line of stroke through the cam centre, (ii) offset 15 mm. Find the maximum velocity and acceleration at 900 rpm.',
  inputs:[['S', 'Stroke', 40, 'mm'], ['out', 'Outstroke', 100, '°'], ['ret', 'Return', 90, '°'], ['N', 'Cam speed', 900, 'rpm']],
  solve:v => { const w = v.N*TAU/60, bo = v.out*DEG, br = v.ret*DEG, S = v.S/1000;
    return [['ω =', `${n2(w)} rad/s`], ['v<sub>O</sub> = 2ωS/θ<sub>O</sub> =', `${n2(2*w*S/bo, 2)} m/s`, true], ['v<sub>R</sub> = 2ωS/θ<sub>R</sub> =', `${n2(2*w*S/br, 2)} m/s`, true], ['a<sub>O</sub> = 4ω²S/θ<sub>O</sub>² =', `${n2(4*w*w*S/bo/bo, 0)} m/s²`, true], ['a<sub>R</sub> = 4ω²S/θ<sub>R</sub>² =', `${n2(4*w*w*S/br/br, 0)} m/s²`, true]]; },
  after:btns([['ld4a', 'Draw (i) radial'], ['ld4b', 'Draw (ii) 15 mm offset']]) });
const L = (id, o) => $(id).addEventListener('click', () => loadCam(o));
L('ld1a', { Rb:50, S:40, out:60, d1:30, ret:60, e:0, f:'knife', mo:'uv', mr:'uv', N:100 }); L('ld1b', { Rb:50, S:40, out:60, d1:30, ret:60, e:20, f:'knife', mo:'uv', mr:'uv', N:100 });
L('ld2a', { Rb:25, S:50, out:120, d1:30, ret:60, rr:10, e:0, f:'roller', mo:'shm', mr:'shm', N:100 }); L('ld2b', { Rb:25, S:50, out:120, d1:30, ret:60, rr:10, e:15, f:'roller', mo:'shm', mr:'shm', N:100 });
L('ld3', { Rb:25, S:20, out:120, d1:30, ret:120, e:0, f:'flat', mo:'shm', mr:'shm', N:100 });
L('ld4a', { Rb:50, S:40, out:100, d1:80, ret:90, e:0, f:'knife', mo:'uarm', mr:'uarm', N:900 }); L('ld4b', { Rb:50, S:40, out:100, d1:80, ret:90, e:15, f:'knife', mo:'uarm', mr:'uarm', N:900 });

/* ================= quiz ================= */
quiz($('qz'), [
  { q:'For a roller follower, the prime circle is…', o:['the same as the base circle', 'larger than the base circle by the roller radius', 'the circle through the pitch point', 'the largest circle on the cam'], a:1, why:'The trace point is the roller centre, which never comes closer than base radius + roller radius.' },
  { q:'Which follower motion gives zero acceleration at the start and end of the stroke?', o:['Uniform velocity', 'Simple harmonic', 'Uniform acceleration and retardation', 'Cycloidal'], a:3, why:'a = 2πω²S/θ² · sin(2πθ/θ<sub>O</sub>) is zero at both ends, so there is no shock.' },
  { q:'A cam turns at 300 rpm and lifts a follower 30 mm in 90° with simple harmonic motion. The maximum velocity is about…', o:['0.31 m/s', '0.94 m/s', '1.88 m/s', '0.47 m/s'], a:1, why:'v = πωS/(2θ<sub>O</sub>) = π × 31.42 × 0.03/(2 × 1.571) = 0.94 m/s.' },
  { q:'When drawing a cam profile by inversion, the follower is turned…', o:['in the same direction as the cam', 'in the opposite direction to the cam', 'only during the dwell', 'not at all; the cam is turned'], a:1, why:'Fixing the cam means giving everything an equal and opposite rotation, so the follower goes round the other way.' },
  { q:'Why are flat-faced followers used for engine valves?', o:['They give the largest lift', 'Side thrust on the guide is small and they are compact', 'They never wear', 'They allow any cam shape'], a:1, why:'The force is along the follower axis apart from friction, and the follower fits in a small space.' },
  { q:'The pressure angle of a cam gets too large. Which change helps?', o:['A larger base circle', 'A shorter outstroke angle', 'A larger stroke', 'A higher speed'], a:0, why:'A bigger cam makes the pitch curve flatter for the same rise, reducing the angle between its normal and the follower line.' },
  { q:'With uniform acceleration and retardation, the maximum velocity is reached…', o:['at the start of the stroke', 'half-way through the stroke (in angle)', 'at the end of the stroke', 'during the dwell'], a:1, why:'The follower accelerates uniformly for half the stroke, then retards uniformly.' }
]);
})();
