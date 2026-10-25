/* BME 2102 interactive notes, Chapter 7: balancing of rotating masses */
(() => {
'use strict';
const PI = Math.PI, TAU = 2*PI;
const rpm2w = N => N*TAU/60;
const angDeg = (x, y) => { let a = Math.atan2(y, x)/DEG; if (a < 0) a += 360; return a; };
const kN = F => Math.abs(F) < 1e-6 ? '0 <small>N</small>' : Math.abs(F) >= 1000 ? `${n2(F/1000, 2)} <small>kN</small>` : `${n2(F, 0)} <small>N</small>`;
function view(cv, b, pad = 14){ const { ctx, W, H } = prep(cv); const sc = Math.min((W - 2*pad)/(b.xr - b.xl), (H - 2*pad)/(b.yt - b.yb));
  const ox = pad + (W - 2*pad - (b.xr - b.xl)*sc)/2 - b.xl*sc, oy = pad + (H - 2*pad - (b.yt - b.yb)*sc)/2 + b.yt*sc;
  return { ctx, W, H, sc, PX:x => ox + x*sc, PY:y => oy - y*sc, IX:px => (px - ox)/sc, IY:py => (oy - py)/sc }; }
const dot = (ctx, x, y, col, r = 4) => { ctx.fillStyle = col; ctx.beginPath(); ctx.arc(x, y, r, 0, 7); ctx.fill(); };
const lbl = (ctx, t, x, y, col, al = 'left', size = 12) => label(ctx, t, x, y, col, al, size, true);

/* bearing reactions for masses [{x, m, r, a}] on a shaft turning at w, bearings at xA, xB.
   Returns the force each bearing feels from the shaft (vectors in the rotating frame) and the totals. */
function reactions(masses, xA, xB, w){ let Fy = 0, Fz = 0, My = 0, Mz = 0;
  for (const q of masses){ const f = q.m*q.r*w*w; Fy += f*Math.cos(q.a); Fz += f*Math.sin(q.a); My += f*Math.cos(q.a)*(q.x - xA); Mz += f*Math.sin(q.a)*(q.x - xA); }
  const L = xB - xA, RBy = My/L, RBz = Mz/L, RAy = Fy - RBy, RAz = Fz - RBz;   // forces the shaft puts on the bearings
  return { A:[RAy, RAz], B:[RBy, RBz], F:Math.hypot(Fy, Fz), C:Math.hypot(Fy*0 + My - Fy*0, Mz), Fv:[Fy, Fz], Mv:[My, Mz] }; }

/* ---------- 3D shaft viewer ---------- */
function textSprite(t, col){ const c = document.createElement('canvas'); c.width = 128; c.height = 64; const x = c.getContext('2d'); x.font = '600 40px Roboto, Arial, sans-serif'; x.fillStyle = col; x.textAlign = 'center'; x.textBaseline = 'middle'; x.fillText(t, 64, 34);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map:new THREE.CanvasTexture(c), depthTest:false, transparent:true })); s.scale.set(0.08, 0.04, 1); return s; }
function shaftViewer(cv, get){
  const v = view3d(cv, { cam:[0.25, 0.42, 0.95], target:[0.3, 0, 0], fov:30,
    build(v){ v.rebuild = () => build(v); build(v); },
    update(dt, v){ const c = v.cfg; if (!c) return; v.th = (v.th || 0) + dt*Math.min(6, 1.2 + c.w*0.004); v.rot.rotation.x = v.th;
      const R = reactions(c.masses, c.bear[0], c.bear[1], c.w), ref = c.Fref || Math.max(1, R.F, Math.hypot(...R.A), Math.hypot(...R.B));
      [['A', v.pA, v.arA], ['B', v.pB, v.arB]].forEach(([k, ped, ar]) => { const [fy, fz] = R[k], ang = v.th, gy = fy*Math.cos(ang) - fz*Math.sin(ang), gz = fy*Math.sin(ang) + fz*Math.cos(ang), mag = Math.hypot(gy, gz);
        const s = Math.min(1, mag/ref); ped.position.y = ped.userData.y + 0.012*s*(mag ? gy/mag : 0); ped.position.z = 0.012*s*(mag ? gz/mag : 0);
        if (mag > ref*0.01){ ar.visible = true; ar.setDirection(new THREE.Vector3(0, gy/mag, gz/mag)); ar.setLength(0.04 + 0.16*s, 0.03, 0.02); } else ar.visible = false; }); } });
  function build(v){ const T = THREE, c = v.cfg = get(); while (v.root.children.length) v.root.remove(v.root.children[0]); readColors();
    const ms = { shaft:mat('#9aa6b2', { metalness:0.7, roughness:0.3 }), dist:mat(C.crank, { metalness:0.4, roughness:0.4 }), bal:mat(C.acc, { metalness:0.4, roughness:0.4 }), arm:mat('#6c7884', { metalness:0.5, roughness:0.4 }),
      disc:new T.MeshStandardMaterial({ color:C.muted, transparent:true, opacity:0.12, side:T.DoubleSide, depthWrite:false }), ped:mat('#7f8c99', { metalness:0.2, roughness:0.7 }) };
    const xs = c.masses.map(q => q.x).concat(c.bear), x0 = Math.min(...xs) - 0.06, x1 = Math.max(...xs) + 0.06, rmax = Math.max(0.05, ...c.masses.map(q => q.r));
    const sc = 0.2/rmax;   // draw radii so the largest is 0.2 scene units
    const rot = v.rot = new T.Group(); v.root.add(rot);
    const sh = new T.Mesh(new T.CylinderGeometry(0.014, 0.014, x1 - x0, 24), ms.shaft); sh.rotation.z = PI/2; sh.position.x = (x0 + x1)/2; rot.add(sh);
    const mmax = Math.max(...c.masses.map(q => q.m), 1e-6), planes = new Map();
    for (const q of c.masses){ const r = q.r*sc, rad = 0.014 + 0.022*Math.cbrt(q.m/mmax), m3 = q.kind === 'bal' ? ms.bal : ms.dist;
      const s = new T.Mesh(new T.SphereGeometry(rad, 24, 16), m3); s.position.set(q.x, r*Math.cos(q.a), r*Math.sin(q.a)); rot.add(s);
      const arm = new T.Mesh(new T.CylinderGeometry(0.005, 0.005, r, 10), ms.arm); arm.position.set(q.x, r/2*Math.cos(q.a), r/2*Math.sin(q.a)); arm.rotation.x = q.a; rot.add(arm);
      if (q.label){ const sp = textSprite(q.label, q.kind === 'bal' ? C.acc : C.ink); sp.position.set(q.x, (r + rad + 0.03)*Math.cos(q.a), (r + rad + 0.03)*Math.sin(q.a)); rot.add(sp); }
      planes.set(q.x.toFixed(4), Math.max(planes.get(q.x.toFixed(4)) || 0, r + rad)); }
    for (const [x, r] of planes){ const d = new T.Mesh(new T.CircleGeometry(r + 0.02, 48), ms.disc); d.rotation.y = PI/2; d.position.x = +x; rot.add(d); }
    const ped = x => { const g = new T.Group(); const b = new T.Mesh(new T.BoxGeometry(0.05, 0.3, 0.08), ms.ped); b.position.y = -0.15 - 0.03; g.add(b);
      const ring = new T.Mesh(new T.TorusGeometry(0.028, 0.012, 10, 24), ms.ped); ring.rotation.y = PI/2; g.add(ring); g.position.x = x; g.userData.y = 0; v.root.add(g); return g; };
    v.pA = ped(c.bear[0]); v.pB = ped(c.bear[1]);
    v.arA = new T.ArrowHelper(new T.Vector3(0, 1, 0), new T.Vector3(c.bear[0], 0, 0), 0.1, new T.Color(C.force).getHex(), 0.03, 0.02); v.root.add(v.arA);
    v.arB = new T.ArrowHelper(new T.Vector3(0, 1, 0), new T.Vector3(c.bear[1], 0, 0), 0.1, new T.Color(C.force).getHex(), 0.03, 0.02); v.root.add(v.arB);
    const base = new T.Mesh(new T.BoxGeometry(x1 - x0 + 0.12, 0.02, 0.24), ms.ped); base.position.set((x0 + x1)/2, -0.34, 0); v.root.add(base);
    v.ctl.target.set((x0 + x1)/2, 0, 0); v.cam.position.set((x0 + x1)/2 - 0.35, 0.42, 0.95 + (x1 - x0)*0.4); v.ctl.update(); }
  return v; }

/* ================= 7.1 one unbalanced disc ================= */
{ const P = {}; let v;
  const cfg = () => { const w = rpm2w(P.N), r = P.r/1000, ms = [{ x:0.15, m:P.m, r, a:0, kind:'dist', label:'m' }]; if ($('u_bal').checked) ms.push({ x:0.15, m:P.m, r, a:PI, kind:'bal', label:'m₂' });
    return { masses:ms, bear:[0, 0.6], w, Fref:P.m*r*w*w }; };
  const up = () => { const c = cfg(), R = reactions(c.masses, 0, 0.6, c.w), F = P.m*P.r/1000*c.w**2;
    $('u_s').innerHTML = [['Centrifugal force of m', kN(F)], ['Force on bearing A', kN(Math.hypot(...R.A))], ['Force on bearing B', kN(Math.hypot(...R.B))], ['Weight of m, for scale', kN(P.m*9.81)], ['Times the weight', n2(F/(P.m*9.81), 1)]]
      .map(([k, x]) => `<div class="stat${k.startsWith('Force on') && Math.hypot(...R.A) > 1 ? ' bad' : ''}"><div class="k">${k}</div><div class="v">${x}</div></div>`).join('');
    $('u_sub').textContent = $('u_bal').checked ? 'balanced: no rotating load on the bearings' : 'unbalanced: the bearings carry a load that turns with the shaft';
    if (v){ v.rebuild(); v.render(); } };
  controls($('u_c'), [['m', 'Unbalanced mass m', 0.05, 5, 0.05, 0.5, 'kg'], ['r', 'Its radius r', 20, 300, 5, 150, 'mm'], ['N', 'Speed', 100, 6000, 50, 1500, 'rpm', vv => `ω = ${n2(rpm2w(vv), 1)} rad/s`]], P, up);
  v = shaftViewer($('u3'), cfg); $('u_bal').addEventListener('change', up); if (v) playButton($('u_play'), v.anim); up(); }

/* ================= 7.2 same plane, end view ================= */
{ const P = {}, cv = $('o2'); let th = 0;
  controls($('o_c'), [['m1', 'Disturbing mass m₁', 0.5, 20, 0.5, 5, 'kg'], ['r1', 'Its radius r₁', 20, 300, 5, 120, 'mm'], ['r2', 'Radius of the balancing mass r₂', 20, 300, 5, 200, 'mm'], ['N', 'Speed', 50, 3000, 50, 600, 'rpm']], P, () => draw());
  function draw(){ const m2 = P.m1*P.r1/P.r2, w = rpm2w(P.N), F = P.m1*P.r1/1000*w*w, R = 330, V = view(cv, { xl:-R, xr:R, yb:-R, yt:R }), { ctx } = V;
    ctx.fillStyle = alpha(C.muted, 0.12); ctx.strokeStyle = C.muted; ctx.beginPath(); ctx.arc(V.PX(0), V.PY(0), 310*V.sc, 0, 7); ctx.fill(); ctx.stroke();
    const p1 = [P.r1*Math.cos(th), P.r1*Math.sin(th)], p2 = [-P.r2*Math.cos(th), -P.r2*Math.sin(th)], size = m => 6 + 10*Math.cbrt(m/20);
    ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(V.PX(p2[0]), V.PY(p2[1])); ctx.lineTo(V.PX(p1[0]), V.PY(p1[1])); ctx.stroke();
    dot(ctx, V.PX(p1[0]), V.PY(p1[1]), C.crank, size(P.m1)); dot(ctx, V.PX(p2[0]), V.PY(p2[1]), C.acc, size(m2)); dot(ctx, V.PX(0), V.PY(0), C.ink, 4);
    const L = 70, u = [Math.cos(th), Math.sin(th)];
    arrow(ctx, V.PX(p1[0]), V.PY(p1[1]), V.PX(p1[0]) + u[0]*L, V.PY(p1[1]) - u[1]*L, C.crank, 2.5); arrow(ctx, V.PX(p2[0]), V.PY(p2[1]), V.PX(p2[0]) - u[0]*L, V.PY(p2[1]) + u[1]*L, C.acc, 2.5);
    lbl(ctx, `m₁ = ${n2(P.m1, 1)} kg`, V.PX(p1[0]) + u[0]*L + 6, V.PY(p1[1]) - u[1]*L, C.crank); lbl(ctx, `m₂ = ${n2(m2, 2)} kg`, V.PX(p2[0]) - u[0]*L - 6, V.PY(p2[1]) + u[1]*L, C.acc, 'right');
    $('o_sub').textContent = 'both forces turn with the disc and always cancel';
    $('o_s').innerHTML = [['m₁r₁', `${n2(P.m1*P.r1/1000, 3)} <small>kg·m</small>`], ['Balancing mass m₂ = m₁r₁/r₂', `${n2(m2, 3)} <small>kg</small>`], ['Each centrifugal force', kN(F)], ['Net force on the shaft', '0 <small>N</small>']]
      .map(([k, x]) => `<div class="stat"><div class="k">${k}</div><div class="v">${x}</div></div>`).join(''); }
  const an = animate(cv, dt => { th += dt*1.2; draw(); }); playButton($('o_play'), an); watchSize(cv, draw); onRedraw(draw); draw(); }

/* ================= 7.3 one mass, two planes ================= */
{ const P = {}; let mode = 'two', v;
  const solve = () => { const xL = 0.1, xM = 0.5, l = xM - xL, xA = P.xA, mr = P.m*P.r/1000;
    const FM = -mr*(xA - xL)/l, FL = -mr*(xM - xA)/l;   // signed m r along the direction of m (negative = opposite side)
    return { xL, xM, l, xA, mr, FL, FM, l1:Math.abs(xA - xL), l2:Math.abs(xM - xA) }; };
  const cfg = () => { const s = solve(), rb = P.rb/1000, ms = [{ x:s.xA, m:P.m, r:P.r/1000, a:0, kind:'dist', label:'A' }];
    if (mode === 'two'){ ms.push({ x:s.xL, m:Math.abs(s.FL)/rb, r:rb, a:s.FL < 0 ? PI : 0, kind:'bal', label:'L' }); ms.push({ x:s.xM, m:Math.abs(s.FM)/rb, r:rb, a:s.FM < 0 ? PI : 0, kind:'bal', label:'M' }); }
    if (mode === 'one') ms.push({ x:s.xL, m:s.mr/rb, r:rb, a:PI, kind:'bal', label:'L' });
    return { masses:ms, bear:[-0.15, 0.85], w:rpm2w(P.N), Fref:s.mr*rpm2w(P.N)**2 }; };
  const up = () => { const s = solve(), c = cfg(), R = reactions(c.masses, -0.15, 0.85, c.w), rb = P.rb/1000, between = s.xA > s.xL && s.xA < s.xM;
    const side = F => F < 0 ? 'opposite m' : 'same side as m';
    $('t_s').innerHTML = [['m r', `${n2(s.mr, 3)} <small>kg·m</small>`], ['m₁r₁ in plane L', `${n2(Math.abs(s.FL), 3)} <small>kg·m, ${side(s.FL)}</small>`], ['m₂r₂ in plane M', `${n2(Math.abs(s.FM), 3)} <small>kg·m, ${side(s.FM)}</small>`],
      ['m₁, m₂ at radius ' + P.rb + ' mm', `${n2(Math.abs(s.FL)/rb, 3)}, ${n2(Math.abs(s.FM)/rb, 3)} <small>kg</small>`], ['Force on bearing 1', kN(Math.hypot(...R.A))], ['Force on bearing 2', kN(Math.hypot(...R.B))]]
      .map(([k, x]) => `<div class="stat${k.startsWith('Force') && Math.hypot(...R.A) > 1 ? ' bad' : ''}"><div class="k">${k}</div><div class="v">${x}</div></div>`).join('');
    $('t_sub').textContent = `${between ? 'A between L and M' : 'A outside L and M'} · l = ${n2(s.l, 2)} m, l₁ = ${n2(s.l1, 2)} m, l₂ = ${n2(s.l2, 2)} m`;
    $('t_cap').innerHTML = mode === 'none' ? 'Unbalanced: both bearings carry a rotating load.' : mode === 'one' ? '<b>Static balance only.</b> The single mass in plane L cancels the force, but the two forces are in different planes, so they form a couple that rocks the shaft: the bearings now push in <b>opposite</b> directions.' : `<b>Dynamic balance.</b> Taking moments about M gives m₁r₁ = m r × ${between ? 'l₂' : 'l₂'}/l; about L gives m₂r₂ = m r × l₁/l. The bearings feel no rotating load.`;
    if (v){ v.rebuild(); v.render(); } };
  controls($('t_c'), [['xA', 'Position of plane A along the shaft', -0.1, 0.8, 0.01, 0.25, 'm', vv => vv > 0.1 && vv < 0.5 ? 'between L (0.1) and M (0.5)' : 'outside L and M'], ['m', 'Disturbing mass m', 0.5, 20, 0.5, 4, 'kg'], ['r', 'Its radius r', 20, 250, 5, 150, 'mm'], ['rb', 'Radius of both balancing masses', 20, 250, 5, 100, 'mm'], ['N', 'Speed', 100, 3000, 50, 900, 'rpm']], P, up);
  document.querySelectorAll('[data-b]').forEach(b => b.addEventListener('click', () => { mode = b.dataset.b; document.querySelectorAll('[data-b]').forEach(x => x.setAttribute('aria-pressed', x === b)); up(); }));
  v = shaftViewer($('t3'), cfg); if (v) playButton($('t_play'), v.anim); up(); }

/* ================= 7.4 several masses in one plane ================= */
const EX1 = [[200, 0.2, 0], [300, 0.15, 45], [240, 0.25, 120], [260, 0.3, 255]];
{ let rows = EX1.map(r => r.slice()); const cv1 = $('p1'), cv2 = $('p2'), NAMES = 'abcdefgh';
  function solve(){ let H = 0, Vv = 0; for (const [m, r, a] of rows){ H += m*r*Math.cos(a*DEG); Vv += m*r*Math.sin(a*DEG); } const R = Math.hypot(H, Vv), rb = +$('p_rb').value || 0.2; return { H, V:Vv, R, th:angDeg(H, Vv), bal:(angDeg(H, Vv) + 180) % 360, m:R/rb, rb }; }
  function table(){ const S = solve();
    $('p_t').innerHTML = '<thead><tr><th>Mass</th><th>m [kg]</th><th>r [m]</th><th>θ [°] anticlockwise from OX</th><th class="n">m r [kg·m]</th><th class="n">m r cos θ</th><th class="n">m r sin θ</th><th></th></tr></thead><tbody>' +
      rows.map(([m, r, a], i) => `<tr><td>${i + 1}</td><td><input type="number" step="any" value="${m}" data-i="${i}" data-k="0" aria-label="mass ${i + 1}"></td><td><input type="number" step="any" value="${r}" data-i="${i}" data-k="1" aria-label="radius ${i + 1}"></td><td><input type="number" step="any" value="${a}" data-i="${i}" data-k="2" aria-label="angle ${i + 1}"></td><td class="n">${n2(m*r, 2)}</td><td class="n">${n2(m*r*Math.cos(a*DEG), 2)}</td><td class="n">${n2(m*r*Math.sin(a*DEG), 2)}</td><td>${rows.length > 1 ? `<button type="button" data-del="${i}" aria-label="Remove mass ${i + 1}">✕</button>` : ''}</td></tr>`).join('') +
      `<tr class="hl"><td colspan="4">Sums ΣH, ΣV</td><td></td><td class="n">${n2(S.H, 2)}</td><td class="n">${n2(S.V, 2)}</td><td></td></tr></tbody>`;
    $('p_t').querySelectorAll('input').forEach(inp => inp.addEventListener('change', () => { rows[+inp.dataset.i][+inp.dataset.k] = +inp.value || 0; table(); draw(); }));
    $('p_t').querySelectorAll('[data-del]').forEach(b => b.addEventListener('click', () => { rows.splice(+b.dataset.del, 1); table(); draw(); }));
    $('p_s').innerHTML = [['Resultant R = √(ΣH² + ΣV²)', `${n2(S.R, 2)} <small>kg·m</small>`], ['Its angle θ′ from OX', `${n2(S.th, 2)}°`], ['Balancing mass at r = ' + S.rb + ' m', `${n2(S.m, 1)} <small>kg</small>`], ['Placed at', `${n2(S.bal, 2)}° <small>(θ′ + 180°)</small>`]]
      .map(([k, x]) => `<div class="stat"><div class="k">${k}</div><div class="v">${x}</div></div>`).join(''); $('p_sub').textContent = rows.length + ' masses'; }
  let drag = -1, V1;
  function draw(){ const S = solve(), rmax = Math.max(S.rb, ...rows.map(r => r[1]))*1.25; V1 = view(cv1, { xl:-rmax, xr:rmax, yb:-rmax, yt:rmax }); const { ctx } = V1;
    ctx.strokeStyle = C.grid; ctx.beginPath(); ctx.moveTo(V1.PX(-rmax), V1.PY(0)); ctx.lineTo(V1.PX(rmax), V1.PY(0)); ctx.moveTo(V1.PX(0), V1.PY(-rmax)); ctx.lineTo(V1.PX(0), V1.PY(rmax)); ctx.stroke(); lbl(ctx, 'X', V1.PX(rmax) - 6, V1.PY(0) - 10, C.muted, 'right');
    const mmax = Math.max(...rows.map(r => r[0]), S.m);
    rows.forEach(([m, r, a], i) => { const x = r*Math.cos(a*DEG), y = r*Math.sin(a*DEG); ctx.strokeStyle = C.crank; ctx.lineWidth = 1.6; ctx.beginPath(); ctx.moveTo(V1.PX(0), V1.PY(0)); ctx.lineTo(V1.PX(x), V1.PY(y)); ctx.stroke();
      dot(ctx, V1.PX(x), V1.PY(y), C.crank, 5 + 9*Math.cbrt(m/mmax)); lbl(ctx, `${m} kg`, V1.PX(x) + 12, V1.PY(y) - 12, C.ink, 'left', 11.5); });
    const bx = S.rb*Math.cos(S.bal*DEG), by = S.rb*Math.sin(S.bal*DEG); ctx.setLineDash([5, 4]); ctx.strokeStyle = C.acc; ctx.lineWidth = 1.6; ctx.beginPath(); ctx.moveTo(V1.PX(0), V1.PY(0)); ctx.lineTo(V1.PX(bx), V1.PY(by)); ctx.stroke(); ctx.setLineDash([]);
    dot(ctx, V1.PX(bx), V1.PY(by), C.acc, 5 + 9*Math.cbrt(S.m/mmax)); lbl(ctx, `m = ${n2(S.m, 1)} kg`, V1.PX(bx) + 12, V1.PY(by) + 14, C.acc, 'left', 11.5); dot(ctx, V1.PX(0), V1.PY(0), C.ink, 3.5);
    // vector polygon
    const pts = [[0, 0]]; rows.forEach(([m, r, a]) => { const [x, y] = pts[pts.length - 1]; pts.push([x + m*r*Math.cos(a*DEG), y + m*r*Math.sin(a*DEG)]); });
    const xs = pts.map(p => p[0]), ys = pts.map(p => p[1]), pad = 0.12*Math.max(...xs.map(Math.abs), ...ys.map(Math.abs), 1);
    const V2 = view(cv2, { xl:Math.min(...xs) - pad, xr:Math.max(...xs) + pad, yb:Math.min(...ys) - pad, yt:Math.max(...ys) + pad }), c2 = V2.ctx;
    for (let i = 1; i < pts.length; i++){ arrow(c2, V2.PX(pts[i - 1][0]), V2.PY(pts[i - 1][1]), V2.PX(pts[i][0]), V2.PY(pts[i][1]), C.crank, 2.2);
      const mx = (pts[i - 1][0] + pts[i][0])/2, my = (pts[i - 1][1] + pts[i][1])/2; lbl(c2, n2(rows[i - 1][0]*rows[i - 1][1], 1), V2.PX(mx) + 6, V2.PY(my) - 8, C.crank, 'left', 11); }
    pts.forEach((p, i) => lbl(c2, NAMES[i] || '', V2.PX(p[0]) - 6, V2.PY(p[1]) + 12, C.ink, 'right', 12));
    const last = pts[pts.length - 1]; if (S.R > 1e-9){ c2.setLineDash([6, 4]); arrow(c2, V2.PX(last[0]), V2.PY(last[1]), V2.PX(0), V2.PY(0), C.acc, 2.4); c2.setLineDash([]);
      lbl(c2, `balancing m r = ${n2(S.R, 1)} kg·m (${NAMES[pts.length - 1]}→a)`, V2.PX(last[0]/2), V2.PY(last[1]/2) + 16, C.acc, 'center', 11.5); }
    lbl(c2, 'vector polygon of m r (head to tail)', 10, 14, C.muted, 'left', 11.5); }
  const setFrom = e => { const b = cv1.getBoundingClientRect(), x = V1.IX(e.clientX - b.left), y = V1.IY(e.clientY - b.top); rows[drag][1] = Math.max(0.01, Math.round(Math.hypot(x, y)*100)/100); rows[drag][2] = Math.round(angDeg(x, y)); table(); draw(); };
  cv1.addEventListener('pointerdown', e => { const b = cv1.getBoundingClientRect(), px = e.clientX - b.left, py = e.clientY - b.top; let best = 18;
    rows.forEach(([m, r, a], i) => { const d = Math.hypot(V1.PX(r*Math.cos(a*DEG)) - px, V1.PY(r*Math.sin(a*DEG)) - py); if (d < best){ best = d; drag = i; } }); if (drag >= 0){ cv1.setPointerCapture(e.pointerId); setFrom(e); } });
  cv1.addEventListener('pointermove', e => { if (drag >= 0) setFrom(e); }); cv1.addEventListener('pointerup', () => drag = -1);
  $('p_add').addEventListener('click', () => { if (rows.length < 8){ rows.push([100, 0.15, (rows.length*67) % 360]); table(); draw(); } });
  $('p_ex').addEventListener('click', () => { rows = EX1.map(r => r.slice()); $('p_rb').value = 0.2; table(); draw(); });
  $('p_rb').addEventListener('input', () => { table(); draw(); });
  watchSize(cv1, draw); watchSize(cv2, draw); onRedraw(draw); table(); draw(); }

/* ================= 7.5 several planes ================= */
const EX2 = [['A', 200, 0.08, 0, -0.1], ['B', 300, 0.07, 45, 0.2], ['C', 400, 0.06, 115, 0.3], ['D', 200, 0.08, 235, 0.6]];
function solvePlanes(rows, lM, rL, rM){ let cx = 0, cy = 0; for (const [, m, r, a, l] of rows){ cx += m*r*l*Math.cos(a*DEG); cy += m*r*l*Math.sin(a*DEG); }
  const C = Math.hypot(cx, cy), mMrM = C/Math.abs(lM), aM = angDeg(-cx*Math.sign(lM), -cy*Math.sign(lM)), mM = mMrM/rM;
  let fx = mMrM*Math.cos(aM*DEG), fy = mMrM*Math.sin(aM*DEG); for (const [, m, r, a] of rows){ fx += m*r*Math.cos(a*DEG); fy += m*r*Math.sin(a*DEG); }
  const mLrL = Math.hypot(fx, fy), aL = angDeg(-fx, -fy); return { cx, cy, C, mM, aM, mMrM, mL:mLrL/rL, aL, mLrL }; }
{ let rows = EX2.map(r => r.slice()), v; const cv1 = $('m_c1'), cv2 = $('m_c2');
  const pars = () => ({ lM:+$('m_lM').value || 0.4, rL:+$('m_rL').value || 0.1, rM:+$('m_rM').value || 0.1, N:+$('m_N').value || 0 });
  const cfg = () => { const p = pars(), S = solvePlanes(rows, p.lM, p.rL, p.rM), ms = rows.map(([n, m, r, a, l]) => ({ x:l, m, r, a:a*DEG, kind:'dist', label:n }));
    if ($('m_on').checked){ ms.push({ x:0, m:S.mL, r:p.rL, a:S.aL*DEG, kind:'bal', label:'L' }); ms.push({ x:p.lM, m:S.mM, r:p.rM, a:S.aM*DEG, kind:'bal', label:'M' }); }
    const xs = ms.map(q => q.x).concat([0, p.lM]); let F0 = 0; for (const [, m, r] of rows) F0 = Math.max(F0, m*r); return { masses:ms, bear:[Math.min(...xs) - 0.12, Math.max(...xs) + 0.12], w:rpm2w(p.N), Fref:F0*rpm2w(p.N)**2*2 }; };
  function table(){ const p = pars(), S = solvePlanes(rows, p.lM, p.rL, p.rM);
    const all = rows.map(r => ({ n:r[0], m:r[1], r:r[2], a:r[3], l:r[4], ed:true })).concat([{ n:'L (R.P.)', m:S.mL, r:p.rL, a:S.aL, l:0 }, { n:'M', m:S.mM, r:p.rM, a:S.aM, l:p.lM }]).sort((x, y) => x.l - y.l);
    $('m_t').innerHTML = '<thead><tr><th>Plane</th><th>Mass m [kg]</th><th>Radius r [m]</th><th>Angle θ [°]</th><th>Distance from L, l [m]</th><th class="n">m r [kg·m]</th><th class="n">m r l [kg·m²]</th><th></th></tr></thead><tbody>' +
      all.map(q => { if (!q.ed) return `<tr class="hl"><td>${q.n}</td><td class="n">${n2(q.m, 1)}</td><td class="n">${q.r}</td><td class="n">${n2(q.a, 1)}</td><td class="n">${q.l}</td><td class="n">${n2(q.m*q.r, 2)}</td><td class="n">${n2(q.m*q.r*q.l, 2)}</td><td></td></tr>`;
        const i = rows.findIndex(r => r[0] === q.n); return `<tr><td><input value="${q.n}" data-i="${i}" data-k="0" style="width:44px" aria-label="plane name"></td>${[1, 2, 3, 4].map(k2 => `<td><input type="number" step="any" value="${rows[i][k2]}" data-i="${i}" data-k="${k2}" aria-label="${['', 'mass', 'radius', 'angle', 'distance'][k2]} of ${q.n}"></td>`).join('')}<td class="n">${n2(q.m*q.r, 2)}</td><td class="n">${n2(q.m*q.r*q.l, 2)}</td><td>${rows.length > 1 ? `<button type="button" data-del="${i}" aria-label="Remove ${q.n}">✕</button>` : ''}</td></tr>`; }).join('') + '</tbody>';
    $('m_t').querySelectorAll('input').forEach(inp => inp.addEventListener('change', () => { const k2 = +inp.dataset.k; rows[+inp.dataset.i][k2] = k2 === 0 ? inp.value : (+inp.value || 0); refresh(); }));
    $('m_t').querySelectorAll('[data-del]').forEach(b => b.addEventListener('click', () => { rows.splice(+b.dataset.del, 1); refresh(); }));
    const cw = a => `${n2(a, 1)}° anticlockwise (${n2((360 - a) % 360, 1)}° clockwise)`;
    $('m_s').innerHTML = [['Unbalanced couple Σ m r l', `${n2(S.C, 3)} <small>kg·m²</small>`], ['Balancing mass in M, m<sub>M</sub>', `${n2(S.mM, 1)} <small>kg</small>`], ['Angle of m<sub>M</sub> from mass ' + rows[0][0], cw((S.aM - rows[0][3] + 360) % 360)], ['Balancing mass in L, m<sub>L</sub>', `${n2(S.mL, 1)} <small>kg</small>`], ['Angle of m<sub>L</sub> from mass ' + rows[0][0], cw((S.aL - rows[0][3] + 360) % 360)]]
      .map(([k, x]) => `<div class="stat"><div class="k">${k}</div><div class="v" style="font-size:15px">${x}</div></div>`).join('');
    $('m_sub').textContent = `${rows.length} masses · reference plane L at 0, plane M at ${p.lM} m`; return S; }
  function polys(){ const p = pars(), S = solvePlanes(rows, p.lM, p.rL, p.rM);
    const poly = (cv, vecs, closeLbl, col, title) => { const pts = [[0, 0]]; vecs.forEach(([x, y]) => { const [a, b] = pts[pts.length - 1]; pts.push([a + x, b + y]); });
      const xs = pts.map(q => q[0]), ys = pts.map(q => q[1]), pad = 0.15*Math.max(...xs.map(Math.abs), ...ys.map(Math.abs), 1e-6);
      const V = view(cv, { xl:Math.min(...xs) - pad, xr:Math.max(...xs) + pad, yb:Math.min(...ys) - pad, yt:Math.max(...ys) + pad }), ctx = V.ctx;
      for (let i = 1; i < pts.length; i++){ const last = i === pts.length - 1; if (last) ctx.setLineDash([6, 4]); arrow(ctx, V.PX(pts[i - 1][0]), V.PY(pts[i - 1][1]), V.PX(pts[i][0]), V.PY(pts[i][1]), last ? C.acc : col, last ? 2.6 : 2); ctx.setLineDash([]);
        const mx = (pts[i - 1][0] + pts[i][0])/2, my = (pts[i - 1][1] + pts[i][1])/2; lbl(ctx, last ? closeLbl : vecs[i - 1][2], V.PX(mx) + 6, V.PY(my) - 8, last ? C.acc : col, 'left', 11); }
      lbl(ctx, title, 10, 14, C.muted, 'left', 11.5); };
    const cv = rows.map(([n, m, r, a, l]) => [m*r*l*Math.cos(a*DEG), m*r*l*Math.sin(a*DEG), `${n}: ${n2(m*r*l, 2)}`]); cv.push([S.mMrM*p.lM*Math.cos(S.aM*DEG), S.mMrM*p.lM*Math.sin(S.aM*DEG)]);
    poly(cv1, cv, `${n2(p.lM*p.rM, 3)} m_M = ${n2(S.C, 2)}`, C.mass, 'couple polygon (m r l), closing side = balancing couple');
    const fv = rows.map(([n, m, r, a]) => [m*r*Math.cos(a*DEG), m*r*Math.sin(a*DEG), `${n}: ${n2(m*r, 1)}`]); fv.push([S.mMrM*Math.cos(S.aM*DEG), S.mMrM*Math.sin(S.aM*DEG), `M: ${n2(S.mMrM, 1)}`]); fv.push([S.mLrL*Math.cos(S.aL*DEG), S.mLrL*Math.sin(S.aL*DEG)]);
    poly(cv2, fv, `${p.rL} m_L = ${n2(S.mLrL, 2)}`, C.crank, 'force polygon (m r), closing side = balancing force'); }
  function refresh(){ table(); polys(); if (v){ v.rebuild(); v.render(); } }
  ['m_lM', 'm_rL', 'm_rM', 'm_N'].forEach(id => $(id).addEventListener('change', refresh)); $('m_on').addEventListener('change', refresh);
  $('m_add').addEventListener('click', () => { if (rows.length < 7){ rows.push([String.fromCharCode(65 + rows.length), 100, 0.08, (rows.length*80) % 360, 0.1*rows.length]); refresh(); } });
  $('m_ex').addEventListener('click', () => { rows = EX2.map(r => r.slice()); $('m_lM').value = 0.4; $('m_rL').value = 0.1; $('m_rM').value = 0.1; refresh(); });
  v = shaftViewer($('m3'), cfg); if (v) playButton($('m_play'), v.anim); watchSize(cv1, polys); watchSize(cv2, polys); onRedraw(polys); refresh(); }

/* ================= worked examples ================= */
example($('ex1'), { title:'Example 1', src:'four masses in one plane (notes p. 140; Khurmi 21.1)',
  q:'Four masses m₁, m₂, m₃ and m₄ are 200 kg, 300 kg, 240 kg and 260 kg. Their radii of rotation are 0.2 m, 0.15 m, 0.25 m and 0.3 m, and the angles between successive masses are 45°, 75° and 135°. Find the position and magnitude of the balance mass required, if its radius of rotation is 0.2 m.',
  inputs:[['m1', 'm₁', 200, 'kg'], ['m2', 'm₂', 300, 'kg'], ['m3', 'm₃', 240, 'kg'], ['m4', 'm₄', 260, 'kg'], ['r1', 'r₁', 0.2, 'm'], ['r2', 'r₂', 0.15, 'm'], ['r3', 'r₃', 0.25, 'm'], ['r4', 'r₄', 0.3, 'm'], ['a12', 'Angle m₁ to m₂', 45, '°'], ['a23', 'Angle m₂ to m₃', 75, '°'], ['a34', 'Angle m₃ to m₄', 135, '°'], ['rb', 'Balance radius', 0.2, 'm']],
  solve:v => { const th = [0, v.a12, v.a12 + v.a23, v.a12 + v.a23 + v.a34], mr = [v.m1*v.r1, v.m2*v.r2, v.m3*v.r3, v.m4*v.r4];
    let H = 0, Vv = 0; mr.forEach((x, i) => { H += x*Math.cos(th[i]*DEG); Vv += x*Math.sin(th[i]*DEG); }); const R = Math.hypot(H, Vv), t1 = Math.atan2(Vv, H)/DEG;
    return [['θ₁, θ₂, θ₃, θ₄ =', th.map(t => t + '°').join(', ')], ['m r =', mr.map(x => n2(x, 1)).join(', ') + ' kg·m'], ['ΣH = Σ m r cos θ =', `${n2(H, 2)} kg·m`], ['ΣV = Σ m r sin θ =', `${n2(Vv, 2)} kg·m`], ['R = √(ΣH² + ΣV²) =', `${n2(R, 2)} kg·m`],
      ['Balance mass m = R/r =', `${n2(R/v.rb, 1)} kg`, true], ['θ′ = tan⁻¹(ΣV/ΣH) =', `${n2(t1, 2)}°`], ['Balance mass at θ′ + 180° =', `${n2(t1 + 180, 2)}° from m₁`, true]]; } });
example($('ex2'), { title:'Example 2', src:'four masses in four planes (notes p. 142; Khurmi 21.2)',
  q:'A shaft carries four masses A, B, C and D of 200 kg, 300 kg, 400 kg and 200 kg revolving at radii 80 mm, 70 mm, 60 mm and 80 mm in planes measured from A at 300 mm, 400 mm and 700 mm. The angles between the cranks measured anticlockwise are A to B 45°, B to C 70° and C to D 120°. The balancing masses are to be placed in planes X and Y. The distance between the planes A and X is 100 mm, between X and Y 400 mm, and between Y and D 200 mm. If the balancing masses revolve at a radius of 100 mm, find their magnitudes and angular positions.',
  inputs:[['mA', 'm<sub>A</sub>', 200, 'kg'], ['mB', 'm<sub>B</sub>', 300, 'kg'], ['mC', 'm<sub>C</sub>', 400, 'kg'], ['mD', 'm<sub>D</sub>', 200, 'kg'], ['rA', 'r<sub>A</sub>', 0.08, 'm'], ['rB', 'r<sub>B</sub>', 0.07, 'm'], ['rC', 'r<sub>C</sub>', 0.06, 'm'], ['rD', 'r<sub>D</sub>', 0.08, 'm'], ['AB', 'Angle A to B', 45, '°'], ['BC', 'Angle B to C', 70, '°'], ['CD', 'Angle C to D', 120, '°'], ['rb', 'Balance radius', 0.1, 'm']],
  solve:v => { const rows = [['A', v.mA, v.rA, 0, -0.1], ['B', v.mB, v.rB, v.AB, 0.2], ['C', v.mC, v.rC, v.AB + v.BC, 0.3], ['D', v.mD, v.rD, v.AB + v.BC + v.CD, 0.6]], S = solvePlanes(rows, 0.4, v.rb, v.rb);
    return [['Take X as the reference plane: l = −0.1 (A), 0.2 (B), 0.3 (C), 0.4 (Y), 0.6 m (D)', ''], ['Couples m r l =', rows.map(r => `${r[0]} ${n2(r[1]*r[2]*r[4], 2)}`).join(', ') + ' kg·m²'],
      ['Σ m r l cos θ, Σ m r l sin θ =', `${n2(S.cx, 3)}, ${n2(S.cy, 3)} kg·m²`], ['Balancing couple 0.04 m<sub>Y</sub> = √(…) =', `${n2(S.C, 3)} kg·m²`], ['m<sub>Y</sub> =', `${n2(S.mM, 1)} kg`, true],
      ['Angle of m<sub>Y</sub> =', `${n2(S.aM, 1)}° anticlockwise from A = ${n2((360 - S.aM) % 360, 1)}° clockwise`, true], ['Force polygon including 0.1 m<sub>Y</sub> gives 0.1 m<sub>X</sub> =', `${n2(S.mLrL, 3)} kg·m`], ['m<sub>X</sub> =', `${n2(S.mL, 1)} kg`, true],
      ['Angle of m<sub>X</sub> =', `${n2(S.aL, 1)}° anticlockwise from A = ${n2((360 - S.aL) % 360, 1)}° clockwise`, true]]; } });

quiz($('qz'), [
  { q:'A 2 kg mass at 100 mm radius turns at 1000 rpm. Its centrifugal force is about…', o:['209 N', '2.19 kN', '20 N', '21.9 kN'], a:1, why:'ω = 104.7 rad/s; F = m ω² r = 2 × 104.7² × 0.1 = 2193 N.' },
  { q:'If the speed doubles, the out-of-balance force…', o:['doubles', 'is four times as large', 'stays the same', 'halves'], a:1, why:'F = m ω² r grows with the square of the speed.' },
  { q:'A single balancing mass placed in a different plane from the disturbing mass gives…', o:['complete balance', 'static balance only: a couple remains', 'dynamic balance only', 'no balance at all'], a:1, why:'The forces cancel but act in different planes, so they form a rocking couple.' },
  { q:'For dynamic balance of masses in several planes…', o:['the forces must balance', 'the couples must balance', 'both the forces and the couples must balance', 'the masses must be equal'], a:2, why:'Those are the two conditions in 7.5.' },
  { q:'In the couple polygon, why are the vectors of masses on the left of the reference plane drawn inward?', o:['They are smaller', 'Their distance l is negative, so the couple m r l reverses', 'They rotate the other way', 'By convention only, with no reason'], a:1, why:'m r l changes sign with l, so the vector points the opposite way along its crank.' },
  { q:'Why is the balancing mass usually put at a large radius?', o:['So that it is easier to see', 'Because only m r matters, so a larger r needs a smaller mass', 'To increase the couple', 'To reduce the speed'], a:1, why:'m₁r₁ = m₂r₂: doubling r₂ halves m₂.' }
]);
})();
