/* BME 2102 interactive notes, Chapter 1: introduction, particle kinematics and kinetics */
(() => {
'use strict';
const PI = Math.PI, TAU = 2*PI;
const sq = x => x*x;
const lbl = (ctx, t, x, y, col, al = 'left', size = 12) => label(ctx, t, x, y, col, al, size, true);

/* ================= 1.1 a six-link mechanism ================= */
{ const cv = $('pm'); let mode = 'links', t = 0.6;
  const A = [0, 0], D = [120, 0], a = 40, b = 110, c = 80, L5 = 105, yF = 55;
  function pose(t){ const fb = fourbar(A, D, a, b, c, t, 1); if (!fb) return null; const B = fb.A, Cc = fb.B, u = vmul(vsub(Cc, B), 1/b), n = vperp(u);
    const E = vadd(vadd(B, vmul(u, 0.45*b)), vmul(n, 42)); const dy = yF - E[1]; const F = [E[0] + Math.sqrt(Math.max(0, L5*L5 - dy*dy)), yF]; return { B, C:Cc, E, F }; }
  function draw(){ const p = pose(t); if (!p) return; const V = fitView(cv, { xl:-45, xr:250, yb:-48, yt:142 }, 8), { ctx } = V, P = V.P;
    const col = { 1:C.muted, 2:C.crank, 3:C.rod, 4:C.vel, 5:C.force, 6:C.acc };
    const kind = { 1:'t', 2:'b', 3:'t', 4:'b', 5:'b', 6:'b' }, kc = k => kind[k] === 't' ? C.acc : C.crank;
    const cl = k => mode === 'kind' ? kc(k) : mode === 'pairs' ? alpha(C.muted, 0.55) : col[k];
    // ground: pivots and slide guide
    const g = mode === 'kind' ? kc(1) : C.muted;
    ctx.strokeStyle = g; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(...P([120, yF - 9])); ctx.lineTo(...P([245, yF - 9])); ctx.moveTo(...P([120, yF + 9])); ctx.lineTo(...P([245, yF + 9])); ctx.stroke();
    groundAt(ctx, ...P([182, yF + 9]), 124*V.sc, PI);
    // links
    const [Bx, By] = P(p.B), [Cx, Cy] = P(p.C), [Ex, Ey] = P(p.E), [Fx, Fy] = P(p.F), [Ax, Ay] = P(A), [Dx, Dy] = P(D);
    ctx.fillStyle = alpha(cl(3).startsWith('#') ? cl(3) : C.rod, mode === 'pairs' ? 0.12 : 0.22); ctx.strokeStyle = cl(3); ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(Bx, By); ctx.lineTo(Cx, Cy); ctx.lineTo(Ex, Ey); ctx.closePath(); ctx.fill(); ctx.stroke();
    barAt(ctx, Ax, Ay, Bx, By, cl(2), 7); barAt(ctx, Dx, Dy, Cx, Cy, cl(4), 7); barAt(ctx, Ex, Ey, Fx, Fy, cl(5), 6);
    ctx.fillStyle = mode === 'pairs' ? alpha(C.muted, 0.3) : alpha(cl(6), 0.85); ctx.fillRect(Fx - 16, Fy - 8, 32, 16);
    pivotAt(ctx, Ax, Ay); pivotAt(ctx, Dx, Dy);
    if (mode === 'kind'){ ctx.strokeStyle = kc(1); ctx.lineWidth = 2; groundAt(ctx, Ax, Ay + 13, 30); }
    for (const [x, y] of [[Bx, By], [Cx, Cy], [Ex, Ey], [Fx, Fy]]) pinAt(ctx, x, y);
    if (mode === 'links'){ const mid = (p1, p2) => [(p1[0] + p2[0])/2, (p1[1] + p2[1])/2];
      const tag = (txt, q, k) => { const [x, y] = P(q); ctx.fillStyle = col[k]; ctx.beginPath(); ctx.arc(x, y, 10, 0, 7); ctx.fill(); ctx.fillStyle = '#fff'; ctx.font = `600 12px ${C.mono}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(txt, x, y + 0.5); };
      tag('2', vadd(mid(A, p.B), [-12, 4]), 2); tag('3', [(p.B[0] + p.C[0] + p.E[0])/3, (p.B[1] + p.C[1] + p.E[1])/3], 3); tag('4', vadd(mid(D, p.C), [12, 0]), 4);
      tag('5', mid(p.E, p.F), 5); tag('6', vadd(p.F, [0, 18]), 6); tag('1', [60, -28], 1);
      $('pm_cap').innerHTML = '<b>6 links.</b> 1 frame (fixed), 2 crank, 3 coupler plate, 4 rocker, 5 connecting link, 6 slider. One link must be fixed for a chain to become a mechanism.'; }
    if (mode === 'pairs'){ const ring = (x, y, t2) => { ctx.strokeStyle = C.vel; ctx.lineWidth = 2.5; ctx.beginPath(); ctx.arc(x, y, 11, 0, 7); ctx.stroke(); lbl(ctx, t2, x + 14, y - 12, C.vel, 'left', 12.5); };
      ring(Ax, Ay, 'A'); ring(Dx, Dy, 'D'); ring(Bx, By, 'B'); ring(Cx, Cy, 'C'); ring(Ex, Ey, 'E'); ring(Fx, Fy, 'F');
      ctx.strokeStyle = C.acc; ctx.lineWidth = 2.5; ctx.strokeRect(Fx - 22, Fy - 13, 44, 26); lbl(ctx, 'sliding pair', Fx, Fy + 26, C.acc, 'center', 12);
      $('pm_cap').innerHTML = '<b>7 lower pairs.</b> Six turning pairs (A to F, green) and one sliding pair (the slider in its guide). All have surface contact. Grubler: 3(6 − 1) − 2×7 − 0 = <b>1</b> degree of freedom, so turning the crank fixes every other link.'; }
    if (mode === 'kind'){ lbl(ctx, 'ternary', ...vadd(P([(p.B[0] + p.C[0] + p.E[0])/3, (p.B[1] + p.C[1] + p.E[1])/3]), [0, 0]), C.acc, 'center', 12);
      lbl(ctx, 'ternary frame: joints at A, D and the guide', ...P([60, -40]), C.acc, 'center', 12);
      $('pm_cap').innerHTML = '<b>Binary links</b> (blue) join two other links: the crank 2, the rocker 4, the link 5 and the slider 6. <b>Ternary links</b> (pink) join three: the coupler plate 3 (at B, C and E) and the frame 1 (at A, D and the slider guide).'; }
    $('pm_sub').textContent = 'one input: turn the crank 2'; }
  segGroup($('pm_seg'), v => { mode = v; draw(); });
  watchSize(cv, draw); onRedraw(draw); const an = animate(cv, dt => { t -= dt*1.4; draw(); }); playButton($('pm_play'), an); draw(); }

example($('ex1'), { title:'Degrees of freedom of the mechanism above', src:'Grubler and Kutzbach',
  q:'Count the links and pairs of the six-link mechanism above and find its degrees of freedom. Change the counts to check other mechanisms.',
  inputs:[['N', 'Links N, including the frame', 6], ['J1', 'Lower pairs J₁', 7], ['J2', 'Higher pairs J₂', 0]],
  solve:v => { const p = 3*(v.N - 1) - 2*v.J1 - v.J2, s = 6*(v.N - 1) - 5*v.J1 - 4*v.J2;
    const what = p < 0 ? 'a statically indeterminate structure (more constraint than needed)' : p === 0 ? 'a structure: nothing can move' : p === 1 ? 'a constrained mechanism: one input fixes the motion of every link' : `a mechanism needing ${p} independent inputs`;
    return [['Freedom of the moving links before joining: 3(N − 1) =', `${3*(v.N - 1)}`], ['Each lower pair removes 2: 2J₁ =', `${2*v.J1}`], ['Each higher pair removes 1: J₂ =', `${v.J2}`],
      ['Planar DoF = 3(N − 1) − 2J₁ − J₂ =', `${p}`, true], [`So the assembly is ${what}.`, ''], ['If the same counts were a spatial linkage, Kutzbach: 6(N − 1) − 5J₁ − 4J₂ =', `${s}`]]; } });

/* ================= 1.2 particle in space (3D) ================= */
function textSprite(t, col, s = 0.22){ const c = document.createElement('canvas'); c.width = 128; c.height = 64; const x = c.getContext('2d'); x.font = '500 40px Roboto, Arial, sans-serif'; x.fillStyle = col; x.textAlign = 'center'; x.textBaseline = 'middle'; x.fillText(t, 64, 34);
  const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map:new THREE.CanvasTexture(c), depthTest:false, transparent:true })); sp.scale.set(s, s/2, 1); return sp; }
{ let mode = 'rect', t = 0.8; const P3 = t => [1.3*Math.cos(t), 1.0*Math.sin(t), 0.9 + 0.45*Math.sin(2*t)];
  const T = m => new THREE.Vector3(m[0], m[2], -m[1]);   // maths (x, y, z up) to scene (x, y up, -z)
  const lines = {}; let ball, sprites = {};
  const v = view3d($('pc3'), { cam:[3.2, 2.6, 3.6], target:[0, 0.5, 0], fov:38,
    build(v){ const G = THREE; readColors();
      const axm = new G.LineBasicMaterial({ color:new G.Color(C.muted) });
      for (const [d, nm] of [[[1.9, 0, 0], 'x'], [[0, 1.6, 0], 'y'], [[0, 0, 1.7], 'z']]){ const g = new G.BufferGeometry().setFromPoints([T([0, 0, 0]), T(d)]); v.root.add(new G.Line(g, axm));
        const s = textSprite(nm, C.muted); s.position.copy(T(d.map(q => q*1.08))); v.root.add(s); }
      const grid = new G.GridHelper(4, 8, new G.Color(C.rule), new G.Color(C.grid)); v.root.add(grid);
      const pts = []; for (let i = 0; i <= 200; i++) pts.push(T(P3(i/200*TAU)));
      v.root.add(new G.Line(new G.BufferGeometry().setFromPoints(pts), new G.LineBasicMaterial({ color:new G.Color(C.crank) })));
      ball = new G.Mesh(new G.SphereGeometry(0.06, 24, 16), mat(C.acc)); v.root.add(ball);
      const mk = (col, n = 40) => { const g = new G.BufferGeometry(); g.setAttribute('position', new G.BufferAttribute(new Float32Array(n*3), 3)); const l = new G.Line(g, new G.LineBasicMaterial({ color:new G.Color(col) })); v.root.add(l); return l; };
      lines.a = mk(C.vel); lines.b = mk(C.vel); lines.c = mk(C.vel); lines.d = mk(C.force); lines.e = mk(C.force); lines.f = mk(C.rod);
      for (const k of ['a', 'b', 'c', 'd', 'e', 'f']){ sprites[k] = textSprite('', C.ink); v.root.add(sprites[k]); } },
    update(dt){ t += dt*0.45; place(); } });
  function setLine(l, arr){ const p = l.geometry.attributes.position; for (let i = 0; i < p.count; i++){ const q = arr[Math.min(i, arr.length - 1)]; const w = T(q); p.setXYZ(i, w.x, w.y, w.z); } p.needsUpdate = true; l.geometry.computeBoundingSphere(); l.visible = arr.length > 1; }
  function setSprite(k, txt, at, col){ const s = sprites[k]; const c = s.material.map.image, x = c.getContext('2d'); x.clearRect(0, 0, 128, 64); x.font = '500 40px Roboto, Arial, sans-serif'; x.fillStyle = col || C.ink; x.textAlign = 'center'; x.textBaseline = 'middle'; x.fillText(txt, 64, 34); s.material.map.needsUpdate = true; s.position.copy(T(at)); s.visible = !!txt; }
  const arc = (r, a0, a1, f) => { const o = []; for (let i = 0; i <= 39; i++){ const a = a0 + (a1 - a0)*i/39; o.push(f(a, r)); } return o; };
  function place(){ if (!v) return; const p = P3(t), [x, y, z] = p; ball.position.copy(T(p));
    const r = Math.hypot(x, y), th = Math.atan2(y, x), R = Math.hypot(r, z), ph = Math.atan2(z, r);
    for (const k in lines) lines[k].visible = false; for (const k in sprites) sprites[k].visible = false;
    let st;
    if (mode === 'rect'){ setLine(lines.a, [[x, 0, 0], [x, y, 0]]); setLine(lines.b, [[0, y, 0], [x, y, 0]]); setLine(lines.c, [[x, y, 0], p]);
      setSprite('a', 'x', [x/2, -0.12, 0], C.vel); setSprite('b', 'y', [-0.12, y/2, 0], C.vel); setSprite('c', 'z', [x + 0.1, y, z/2], C.vel);
      st = [['x', `${n2(x)} <small>m</small>`], ['y', `${n2(y)} <small>m</small>`], ['z', `${n2(z)} <small>m</small>`]]; }
    else if (mode === 'cyl'){ setLine(lines.a, [[0, 0, 0], [x, y, 0]]); setLine(lines.c, [[x, y, 0], p]); setLine(lines.d, arc(0.45, 0, th, a => [0.45*Math.cos(a), 0.45*Math.sin(a), 0]));
      setSprite('a', 'r', [x/2, y/2, 0.08], C.vel); setSprite('c', 'z', [x + 0.1, y, z/2], C.vel); setSprite('d', 'θ', [0.6*Math.cos(th/2), 0.6*Math.sin(th/2), 0], C.force);
      st = [['r', `${n2(r)} <small>m</small>`], ['θ', `${n2(((th/DEG) + 360) % 360, 1)}°`], ['z', `${n2(z)} <small>m</small>`]]; }
    else if (mode === 'sph'){ setLine(lines.a, [[0, 0, 0], p]); setLine(lines.b, [[0, 0, 0], [x, y, 0]]); setLine(lines.f, [[x, y, 0], p]);
      setLine(lines.d, arc(0.45, 0, th, a => [0.45*Math.cos(a), 0.45*Math.sin(a), 0])); setLine(lines.e, arc(0.6, 0, ph, (a) => [0.6*Math.cos(a)*Math.cos(th), 0.6*Math.cos(a)*Math.sin(th), 0.6*Math.sin(a)]));
      setSprite('a', 'R', [x/2, y/2, z/2 + 0.1], C.vel); setSprite('d', 'θ', [0.6*Math.cos(th/2), 0.6*Math.sin(th/2), 0], C.force); setSprite('e', 'φ', [0.75*Math.cos(ph/2)*Math.cos(th), 0.75*Math.cos(ph/2)*Math.sin(th), 0.75*Math.sin(ph/2)], C.force);
      st = [['R', `${n2(R)} <small>m</small>`], ['θ', `${n2(((th/DEG) + 360) % 360, 1)}°`], ['φ (above the x–y plane)', `${n2(ph/DEG, 1)}°`]]; }
    else { const h = 1e-3, p1 = P3(t - h), p2 = P3(t + h), d1 = p2.map((q, i) => (q - p1[i])/(2*h)), d2 = p2.map((q, i) => (q - 2*p[i] + p1[i])/(h*h));
      const sp = Math.hypot(...d1), tt = d1.map(q => q/sp), dd = d2.map((q, i) => q - (d2[0]*tt[0] + d2[1]*tt[1] + d2[2]*tt[2])*tt[i]), nl = Math.hypot(...dd), nn = dd.map(q => q/nl);
      const kap = nl/(sp*sp); setLine(lines.a, [p, p.map((q, i) => q + 0.6*tt[i])]); setLine(lines.d, [p, p.map((q, i) => q + 0.45*nn[i])]);
      setSprite('a', 't', p.map((q, i) => q + 0.7*tt[i]), C.vel); setSprite('d', 'n', p.map((q, i) => q + 0.55*nn[i]), C.force);
      st = [['Speed along the path', `${n2(sp*0.45)} <small>m/s</small>`], ['Radius of curvature here', `${n2(1/kap)} <small>m</small>`], ['t: tangent; n: towards the centre of curvature', '']]; }
    $('pc_s').innerHTML = statsHTML(st); }
  $('pc_sub').textContent = 'the same point, four ways to describe it';
  segGroup($('pc_seg'), m => { mode = m; place(); if (v) v.render(); }); if (v){ playButton($('pc_play'), v.anim); place(); } }

/* ================= 1.2.1 rectilinear motion ================= */
{ const P = {}; let mode = 'const', t = 0, H;
  const M = {
    const:{ T:6, specs:[['u', 'Initial velocity u', -6, 6, 0.5, -2, 'm/s'], ['a', 'Acceleration a', -3, 3, 0.1, 1.2, 'm/s²']],
      f:t => ({ s:P.u*t + 0.5*P.a*t*t, v:P.u + P.a*t, a:P.a }) },
    trip:{ T:8, specs:[['A', 'Acceleration while starting', 0.5, 4, 0.1, 2, 'm/s²'], ['B', 'Deceleration while braking', 0.5, 4, 0.1, 2.5, 'm/s²']],
      f:t => { const t1 = 2, t2 = 5, vm = P.A*t1, tb = vm/P.B, t3 = t2 + tb;
        if (t < t1) return { s:0.5*P.A*t*t, v:P.A*t, a:P.A }; const s1 = 0.5*P.A*t1*t1;
        if (t < t2) return { s:s1 + vm*(t - t1), v:vm, a:0 }; const s2 = s1 + vm*(t2 - t1);
        if (t < t3){ const d = t - t2; return { s:s2 + vm*d - 0.5*P.B*d*d, v:vm - P.B*d, a:-P.B }; }
        return { s:s2 + vm*tb - 0.5*P.B*tb*tb, v:0, a:0 }; } },
    osc:{ T:8, specs:[['A', 'Amplitude', 0.5, 5, 0.1, 3, 'm'], ['w', 'Angular frequency ω', 0.4, 3, 0.05, 1.2, 'rad/s']],
      f:t => ({ s:P.A*Math.sin(P.w*t), v:P.A*P.w*Math.cos(P.w*t), a:-P.A*P.w*P.w*Math.sin(P.w*t) }) } };
  const W = {}; let Wc;
  function build(){ const box = $('rl_c'); box.innerHTML = ''; for (const k in P) delete P[k];
    controls(box, M[mode].specs.concat([['t1', 'Window start t₁', 0, M[mode].T, 0.1, 1, 's'], ['t2', 'Window end t₂', 0, M[mode].T, 0.1, Math.min(4, M[mode].T), 's']]), P, () => { calc(); draw(); }); calc(); draw(); }
  let X = [], S = [], Vv = [], Aa = [];
  function calc(){ const Tm = M[mode].T, n = 400; X = []; S = []; Vv = []; Aa = [];
    for (let i = 0; i <= n; i++){ const tt = i/n*Tm, q = M[mode].f(tt); X.push(tt); S.push(q.s); Vv.push(q.v); Aa.push(q.a); } }
  function area(arr, a, b){ if (b < a) [a, b] = [b, a]; let s = 0; for (let i = 1; i < X.length; i++){ const x0 = Math.max(a, X[i - 1]), x1 = Math.min(b, X[i]); if (x1 <= x0) continue;
      const f = x => arr[i - 1] + (arr[i] - arr[i - 1])*(x - X[i - 1])/(X[i] - X[i - 1]); s += (f(x0) + f(x1))/2*(x1 - x0); } return s; }
  function draw(){ const Tm = M[mode].T, q = M[mode].f(t), t1 = Math.min(P.t1, P.t2), t2 = Math.max(P.t1, P.t2), j = Math.round(t/Tm*(X.length - 1));
    const inw = arr => arr.map((y, i) => X[i] >= t1 && X[i] <= t2 ? y : 0);
    const vl = [{ v:t1, color:C.muted, label:'t₁' }, { v:t2, color:C.muted, label:'t₂' }];
    chart($('rl_s'), { x:X, series:[{ y:S, color:C.crank, label:'s' }], ylab:'s [m]', xlab:'t [s]', cursor:t, cursorIndex:j, vlines:vl, value:`s = ${n2(q.s)} m` });
    chart($('rl_v'), { x:X, series:[{ y:Vv, color:C.vel, label:'v = ds/dt' }], fill:{ y:inw(Vv), color:C.vel }, ylab:'v [m/s]', xlab:'t [s]', cursor:t, cursorIndex:j, vlines:vl, value:`v = ${n2(q.v)} m/s` });
    chart($('rl_a'), { x:X, series:[{ y:Aa, color:C.acc, label:'a = dv/dt' }], fill:{ y:inw(Aa), color:C.acc }, ylab:'a [m/s²]', xlab:'t [s]', cursor:t, cursorIndex:j, vlines:vl, value:`a = ${n2(q.a)} m/s²` });
    // track
    const smin = Math.min(...S, 0), smax = Math.max(...S, 0), pad = (smax - smin)*0.08 + 0.5; const V = fitView($('rl_track'), { xl:smin - pad, xr:smax + pad, yb:-1, yt:1 }, 10), { ctx } = V;
    const y0 = V.PY(0); ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(V.PX(smin - pad), y0); ctx.lineTo(V.PX(smax + pad), y0); ctx.stroke();
    const st = niceStep(smax - smin + 2*pad, 8); ctx.font = `11px ${C.mono}`; ctx.fillStyle = C.muted; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (let s = Math.ceil((smin - pad)/st)*st; s <= smax + pad; s += st){ ctx.beginPath(); ctx.moveTo(V.PX(s), y0 - 4); ctx.lineTo(V.PX(s), y0 + 4); ctx.stroke(); ctx.fillText(fmtTick(s, st), V.PX(s), y0 + 7); }
    lbl(ctx, 'O', V.PX(0), y0 - 14, C.ink, 'center'); const px = V.PX(q.s);
    const vs = Math.max(...Vv.map(Math.abs), 0.1), as = Math.max(...Aa.map(Math.abs), 0.1), Lm = Math.min(120, V.W*0.18);
    if (Math.abs(q.v) > 1e-3) arrow(ctx, px, y0 - 16, px + q.v/vs*Lm, y0 - 16, C.vel, 2.5);
    if (Math.abs(q.a) > 1e-3) arrow(ctx, px, y0 + 22, px + q.a/as*Lm, y0 + 22, C.acc, 2.5);
    dotAt(ctx, px, y0, C.crank, 7);
    const ds = (M[mode].f(t2).s - M[mode].f(t1).s), dv = (M[mode].f(t2).v - M[mode].f(t1).v);
    $('rl_st').innerHTML = statsHTML([['Area under v–t, t₁ to t₂', `${n2(area(Vv, t1, t2))} <small>m</small>`], ['s₂ − s₁', `${n2(ds)} <small>m</small>`], ['Area under a–t, t₁ to t₂', `${n2(area(Aa, t1, t2))} <small>m/s</small>`], ['v₂ − v₁', `${n2(dv)} <small>m/s</small>`]]);
    $('rl_sub').textContent = q.v*q.a < -1e-6 ? 'slowing down: a and v have opposite signs' : Math.abs(q.a) < 1e-6 ? 'steady speed' : 'speeding up'; }
  segGroup($('rl_seg'), m => { mode = m; t = 0; build(); });
  const cvs = ['rl_track', 'rl_s', 'rl_v', 'rl_a'].map($); cvs.forEach(c => watchSize(c, draw)); onRedraw(draw);
  const an = animate($('rl_s'), dt => { t += dt; if (t > M[mode].T) t = 0; draw(); }); playButton($('rl_play'), an); build(); }

/* a–s and v–s for a spring-like motion a = −ω² s */
{ const P = {};
  controls($('as_c'), [['A', 'Amplitude A', 1, 5, 0.1, 3, 'm'], ['w', 'ω', 0.5, 1.5, 0.05, 1, 'rad/s'], ['s1', 'From s₁', -5, 5, 0.1, -1, 'm'], ['s2', 'To s₂', -5, 5, 0.1, 2, 'm'], ['sA', 'Point A at s', -5, 5, 0.1, 1.5, 'm']], P, () => draw());
  function draw(){ const A = P.A, w = P.w, cl = s => clamp(s, -A*0.999, A*0.999), s1 = cl(P.s1), s2 = cl(P.s2), sA = cl(P.sA), vOf = s => w*Math.sqrt(Math.max(0, A*A - s*s));
    const X = [], Y = [], F = []; for (let i = 0; i <= 300; i++){ const s = -A + 2*A*i/300; X.push(s); Y.push(-w*w*s); F.push(s >= Math.min(s1, s2) && s <= Math.max(s1, s2) ? -w*w*s : 0); }
    chart($('as_a'), { x:X, series:[{ y:Y, color:C.acc, label:'a = −ω²s' }], fill:{ y:F, color:C.acc }, ylab:'a [m/s²]', xlab:'s [m]', vlines:[{ v:s1, color:C.muted, label:'s₁' }, { v:s2, color:C.muted, label:'s₂' }] });
    // v–s with equal scales and the normal construction
    const m = Math.max(A, A*w) * 1.15, V = fitView($('as_v'), { xl:-m, xr:m, yb:-0.15*m, yt:m }), { ctx } = V;
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1; for (let g = -Math.floor(m); g <= m; g++){ ctx.beginPath(); ctx.moveTo(V.PX(g), V.PY(-0.15*m)); ctx.lineTo(V.PX(g), V.PY(m)); ctx.stroke(); }
    for (let g = 0; g <= m; g++){ ctx.beginPath(); ctx.moveTo(V.PX(-m), V.PY(g)); ctx.lineTo(V.PX(m), V.PY(g)); ctx.stroke(); }
    ctx.strokeStyle = C.muted; ctx.beginPath(); ctx.moveTo(V.PX(-m), V.PY(0)); ctx.lineTo(V.PX(m), V.PY(0)); ctx.moveTo(V.PX(0), V.PY(-0.15*m)); ctx.lineTo(V.PX(0), V.PY(m)); ctx.stroke();
    ctx.strokeStyle = C.vel; ctx.lineWidth = 2; ctx.beginPath(); for (let i = 0; i <= 300; i++){ const s = -A + 2*A*i/300; const x = V.PX(s), y = V.PY(vOf(s)); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); } ctx.stroke();
    const vA = vOf(sA), dvds = -w*w*sA/vA, aA = vA*dvds, Bs = sA + aA;   // the normal meets the s axis at B: CB = v dv/ds
    const pA = V.P([sA, vA]), pC = V.P([sA, 0]), pB = V.P([Bs, 0]);
    dashLine(ctx, ...pA, ...pC, C.muted); ctx.strokeStyle = C.force; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(...pA); ctx.lineTo(...pB); ctx.stroke();
    ctx.strokeStyle = C.acc; ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(...pC); ctx.lineTo(...pB); ctx.stroke();
    dotAt(ctx, ...pA, C.vel, 5); lbl(ctx, 'A', pA[0] + 6, pA[1] - 10, C.vel); lbl(ctx, 'C', pC[0], pC[1] + 12, C.ink, 'center'); lbl(ctx, 'B', pB[0], pB[1] + 12, C.force, 'center');
    lbl(ctx, 'normal', (pA[0] + pB[0])/2 + 6, (pA[1] + pB[1])/2, C.force); lbl(ctx, 'v [m/s] against s [m], equal scales', 8, 12, C.muted, 'left', 11);
    const ar = -w*w*(s2*s2 - s1*s1)/2, dk = 0.5*(vOf(s2)**2 - vOf(s1)**2);
    $('as_st').innerHTML = statsHTML([['Area under a–s, s₁ to s₂', `${n2(ar)} <small>m²/s²</small>`], ['½(v₂² − v₁²)', `${n2(dk)} <small>m²/s²</small>`], ['CB, measured on the s axis', `${n2(aA)} <small>m</small>`], ['a at A', `${n2(-w*w*sA)} <small>m/s²</small>`]]);
    $('as_sub').textContent = 'a particle on a spring: a = −ω² s'; }
  ['as_a', 'as_v'].forEach(c => watchSize($(c), draw)); onRedraw(draw); draw(); }

/* ================= 1.2.2 plane curvilinear motion ================= */
{ const P = {}, cv = $('cv'); let path = 'ell', t = 0.6, O = [0, 0];
  const PATHS = { ell:{ f:t => [3*Math.cos(0.6*t), 2*Math.sin(0.6*t)], b:{ xl:-4.2, xr:4.2, yb:-2.8, yt:2.8 } },
    eight:{ f:t => [3*Math.sin(0.5*t), 2*Math.sin(t)], b:{ xl:-4.2, xr:4.2, yb:-2.8, yt:2.8 } },
    wave:{ f:t => { const x = -4 + ((0.9*t) % 8 + 8) % 8; return [x, 1.3*Math.sin(1.4*x)]; }, b:{ xl:-4.4, xr:4.4, yb:-2.8, yt:2.8 } } };
  controls($('cv_c'), [['dt', 'Time step Δt', 0.02, 2, 0.02, 0.8, 's']], P, () => draw());
  let V;
  function d1(f, t){ const h = 1e-3; const a = f(t - h), b = f(t + h); return [(b[0] - a[0])/(2*h), (b[1] - a[1])/(2*h)]; }
  function d2(f, t){ const h = 1e-2; const a = f(t - h), b = f(t + h), c = f(t); return [(b[0] - 2*c[0] + a[0])/(h*h), (b[1] - 2*c[1] + a[1])/(h*h)]; }
  function draw(){ const pth = PATHS[path], f = pth.f; V = fitView(cv, pth.b); const { ctx } = V; const Pp = V.P;
    if (path === 'wave' && (0.9*t) % 8 > 8 - 0.9*P.dt - 0.05) t = 0.1;
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1; for (let g = Math.ceil(pth.b.xl); g <= pth.b.xr; g++){ ctx.beginPath(); ctx.moveTo(V.PX(g), V.PY(pth.b.yb)); ctx.lineTo(V.PX(g), V.PY(pth.b.yt)); ctx.stroke(); }
    for (let g = Math.ceil(pth.b.yb); g <= pth.b.yt; g++){ ctx.beginPath(); ctx.moveTo(V.PX(pth.b.xl), V.PY(g)); ctx.lineTo(V.PX(pth.b.xr), V.PY(g)); ctx.stroke(); }
    ctx.strokeStyle = alpha(C.muted, 0.8); ctx.lineWidth = 1.5; ctx.beginPath(); const Tp = path === 'ell' ? TAU/0.6 : path === 'eight' ? TAU/0.5 : 8/0.9;
    for (let i = 0; i <= 400; i++){ const q = Pp(f(i/400*Tp*(path === 'wave' ? 0.999 : 1))); i ? ctx.lineTo(...q) : ctx.moveTo(...q); } ctx.stroke();
    const A = f(t), A2 = f(t + P.dt), v = d1(f, t), a = d2(f, t), sp = vlen(v);
    // arc length from A to A′
    let s = 0, prev = A; ctx.strokeStyle = C.rod; ctx.lineWidth = 5; ctx.globalAlpha = 0.45; ctx.beginPath(); ctx.moveTo(...Pp(A));
    for (let i = 1; i <= 60; i++){ const q = f(t + P.dt*i/60); s += vlen(vsub(q, prev)); prev = q; ctx.lineTo(...Pp(q)); } ctx.stroke(); ctx.globalAlpha = 1;
    const o = Pp(O); arrow(ctx, ...o, ...Pp(A), C.ink, 1.8); arrow(ctx, ...o, ...Pp(A2), alpha(C.ink, 0.5), 1.4); arrow(ctx, ...Pp(A), ...Pp(A2), C.crank, 2.4);
    const vs = 0.55, as = 0.35; const vt = vadd(A, vmul(v, vs)), at = vadd(A, vmul(a, as));
    arrow(ctx, ...Pp(A), ...Pp(vt), C.vel, 2.6); arrow(ctx, ...Pp(A), ...Pp(at), C.acc, 2.6);
    if ($('cv_xy').checked){ dashLine(ctx, ...Pp(A), ...Pp([vt[0], A[1]]), C.vel, [3, 3], 1.6); dashLine(ctx, ...Pp([vt[0], A[1]]), ...Pp(vt), C.vel, [3, 3], 1.6);
      lbl(ctx, 'ẋ', ...vadd(Pp([(A[0] + vt[0])/2, A[1]]), [0, 12]), C.vel, 'center'); lbl(ctx, 'ẏ', ...vadd(Pp([vt[0], (A[1] + vt[1])/2]), [10, 0]), C.vel);
      dashLine(ctx, ...Pp(A), ...Pp([at[0], A[1]]), C.acc, [3, 3], 1.6); dashLine(ctx, ...Pp([at[0], A[1]]), ...Pp(at), C.acc, [3, 3], 1.6);
      lbl(ctx, 'ẍ', ...vadd(Pp([(A[0] + at[0])/2, A[1]]), [0, -12]), C.acc, 'center'); lbl(ctx, 'ÿ', ...vadd(Pp([at[0], (A[1] + at[1])/2]), [10, 0]), C.acc); }
    dotAt(ctx, ...Pp(A), C.ink, 5); dotAt(ctx, ...Pp(A2), alpha(C.ink, 0.5), 4); pinAt(ctx, ...o, 5);
    lbl(ctx, 'O', o[0] - 10, o[1] + 12, C.ink, 'right'); lbl(ctx, 'A', Pp(A)[0] + 8, Pp(A)[1] + 12, C.ink); lbl(ctx, 'A′', Pp(A2)[0] + 8, Pp(A2)[1] + 12, C.muted);
    lbl(ctx, 'Δr', ...vadd(Pp(vmul(vadd(A, A2), 0.5)), [8, -8]), C.crank); lbl(ctx, 'v', ...vadd(Pp(vt), [6, -8]), C.vel); lbl(ctx, 'a', ...vadd(Pp(at), [6, -8]), C.acc);
    const cos = (v[0]*a[0] + v[1]*a[1])/(sp*vlen(a) || 1), ang = Math.acos(clamp(cos, -1, 1))/DEG, dr = vlen(vsub(A2, A));
    $('cv_st').innerHTML = statsHTML([['Chord |Δr|', `${n2(dr, 3)} <small>m</small>`], ['Path distance Δs', `${n2(s, 3)} <small>m</small>`], ['|Δr|/Δt', `${n2(dr/P.dt, 3)} <small>m/s</small>`], ['Speed v = ds/dt', `${n2(sp, 3)} <small>m/s</small>`], ['|a|', `${n2(vlen(a), 3)} <small>m/s²</small>`], ['Angle between a and v', `${n2(ang, 0)}°`]]);
    $('cv_sub').textContent = 'shrink Δt: |Δr|/Δt closes on the speed; drag O: Δr does not change'; }
  dragOn(cv, (x, y) => V && Math.hypot(x - V.PX(O[0]), y - V.PY(O[1])) < 14 ? 'O' : null, (h, x, y) => { O = [V.IX(x), V.IY(y)]; draw(); });
  segGroup($('cv_seg'), p => { path = p; t = 0.6; draw(); }); $('cv_xy').addEventListener('change', draw);
  watchSize(cv, draw); onRedraw(draw); const an = animate(cv, dt => { t += dt*0.7; draw(); }); playButton($('cv_play'), an); draw(); }

/* ================= 1.2.3 projectile ================= */
{ const P = {}, cv = $('pj'); let tt = 0, flying = false, ghosts = [];
  controls($('pj_c'), [['v0', 'Launch speed', 2, 40, 0.5, 20, 'm/s'], ['th', 'Launch angle', 0, 90, 1, 35, '°'], ['y0', 'Launch height y₀', 0, 30, 0.5, 1.5, 'm'], ['g', 'g', 1.6, 25, 0.01, 9.81, 'm/s²', v => v < 2 ? 'the Moon' : v < 4 ? 'Mars' : Math.abs(v - 9.81) < 0.05 ? 'Earth' : '']], P, () => { if (!flying) tt = 0; draw(); });
  const traj = q => { const vx = q.v0*Math.cos(q.th*DEG), vy = q.v0*Math.sin(q.th*DEG), tf = (vy + Math.sqrt(vy*vy + 2*q.g*q.y0))/q.g; return { vx, vy, tf, R:vx*tf, H:q.y0 + vy*vy/(2*q.g), at:t => [vx*t, q.y0 + vy*t - 0.5*q.g*t*t] }; };
  function draw(){ const tr = traj(P), all = ghosts.concat([{ ...P }]).map(traj); const xr = Math.max(10, ...all.map(q => q.R))*1.08, yt = Math.max(5, ...all.map(q => q.H))*1.15;
    const V = fitView(cv, { xl:-0.06*xr, xr, yb:-0.08*yt, yt }), { ctx } = V, Pp = V.P;
    ctx.strokeStyle = C.grid; ctx.lineWidth = 1; const gs = niceStep(xr, 8); ctx.font = `11px ${C.mono}`; ctx.fillStyle = C.muted; ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (let g = 0; g <= xr; g += gs){ ctx.beginPath(); ctx.moveTo(V.PX(g), V.PY(0)); ctx.lineTo(V.PX(g), V.PY(yt)); ctx.stroke(); ctx.fillText(fmtTick(g, gs), V.PX(g), V.PY(0) + 4); }
    ctx.textAlign = 'right'; ctx.textBaseline = 'middle'; for (let g = gs; g <= yt; g += gs){ ctx.beginPath(); ctx.moveTo(V.PX(0), V.PY(g)); ctx.lineTo(V.PX(xr), V.PY(g)); ctx.stroke(); ctx.fillText(fmtTick(g, gs), V.PX(0) - 4, V.PY(g)); }
    ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(V.PX(-0.06*xr), V.PY(0)); ctx.lineTo(V.PX(xr), V.PY(0)); ctx.stroke();
    if ($('pj_ghost').checked) ghosts.forEach(gq => { const q = traj(gq); ctx.strokeStyle = alpha(C.muted, 0.6); ctx.lineWidth = 1.2; ctx.setLineDash([4, 4]); ctx.beginPath(); for (let i = 0; i <= 80; i++){ const p = Pp(q.at(q.tf*i/80)); i ? ctx.lineTo(...p) : ctx.moveTo(...p); } ctx.stroke(); ctx.setLineDash([]); });
    ctx.strokeStyle = C.crank; ctx.lineWidth = 2; ctx.beginPath(); for (let i = 0; i <= 120; i++){ const p = Pp(tr.at(tr.tf*i/120)); i ? ctx.lineTo(...p) : ctx.moveTo(...p); } ctx.stroke();
    ctx.fillStyle = alpha(C.muted, 0.25); ctx.fillRect(V.PX(-0.05*xr), V.PY(P.y0), V.PX(0) - V.PX(-0.05*xr), V.PY(0) - V.PY(P.y0));
    const tc = Math.min(tt, tr.tf), p = tr.at(tc), vy = tr.vy - P.g*tc, k = Math.min(V.W, V.H)*0.012, pp = Pp(p);
    arrow(ctx, ...pp, pp[0] + tr.vx*k, pp[1], C.vel, 2.2); arrow(ctx, ...pp, pp[0], pp[1] - vy*k, C.acc, 2.2); arrow(ctx, ...pp, pp[0] + tr.vx*k, pp[1] - vy*k, C.ink, 1.4);
    lbl(ctx, 'vₓ', pp[0] + tr.vx*k + 6, pp[1], C.vel); lbl(ctx, 'v_y', pp[0] + 6, pp[1] - vy*k, C.acc);
    const top = tr.at(tr.vy/P.g); if (tr.vy > 0){ dotAt(ctx, ...Pp(top), C.force, 3.5); lbl(ctx, `top ${n2(tr.H, 2)} m`, Pp(top)[0], Pp(top)[1] - 12, C.force, 'center'); }
    dotAt(ctx, ...pp, C.rod, 6);
    $('pj_st').innerHTML = statsHTML([['Time of flight', `${n2(tr.tf)} <small>s</small>`], ['Range', `${n2(tr.R)} <small>m</small>`], ['Highest point', `${n2(tr.H)} <small>m</small>`], ['vₓ (constant)', `${n2(tr.vx)} <small>m/s</small>`], ['v_y now', `${n2(vy)} <small>m/s</small>`], ['Impact speed', `${n2(Math.hypot(tr.vx, tr.vy - P.g*tr.tf))} <small>m/s</small>`]]);
    $('pj_sub').textContent = flying ? `t = ${n2(tc)} s` : 'press Launch'; }
  $('pj_go').addEventListener('click', () => { if (tt > 0 && $('pj_ghost').checked) ghosts = ghosts.concat([{ ...P }]).slice(-6); tt = 0; flying = true; });
  $('pj_ghost').addEventListener('change', () => { if (!$('pj_ghost').checked) ghosts = []; draw(); });
  watchSize(cv, draw); onRedraw(draw); animate(cv, dt => { if (flying){ tt += dt*Math.max(1, traj(P).tf/4); if (tt >= traj(P).tf){ tt = traj(P).tf; flying = false; } } draw(); }); tt = 999; draw(); tt = 999; }

example($('ex2'), { title:'Practice: a ball thrown from a height', src:'practice question, not in the notes',
  q:'A ball leaves a hand 1.5 m above level ground at 20 m/s, 35° above the horizontal. Ignoring air resistance, find the highest point, the time of flight and the horizontal range.',
  inputs:[['v0', 'Speed', 20, 'm/s'], ['th', 'Angle', 35, '°'], ['y0', 'Height y₀', 1.5, 'm'], ['g', 'g', 9.81, 'm/s²']],
  solve:v => { const vx = v.v0*Math.cos(v.th*DEG), vy = v.v0*Math.sin(v.th*DEG), H = v.y0 + vy*vy/(2*v.g), tf = (vy + Math.sqrt(vy*vy + 2*v.g*v.y0))/v.g;
    return [['Components: (vₓ)₀ = v cos θ =', `${n2(vx, 3)} m/s`], ['(v_y)₀ = v sin θ =', `${n2(vy, 3)} m/s`], ['At the top v_y = 0, so from v_y² = (v_y)₀² − 2g(y − y₀): y = y₀ + (v_y)₀²/2g =', `${n2(H, 3)} m`, true],
      ['Lands when y = 0: 0 = y₀ + (v_y)₀t − ½gt², so t = [(v_y)₀ + √((v_y)₀² + 2gy₀)]/g =', `${n2(tf, 3)} s`, true], ['Range x = (vₓ)₀ t =', `${n2(vx*tf, 2)} m`, true]]; } });

/* ================= 1.3 kinetics ================= */
{ const P = {}, cv = $('nm'); let tt = 0, run = false, x = 0;
  controls($('nm_c'), [['m', 'Mass m', 1, 50, 1, 10, 'kg'], ['u', 'Initial velocity u', -3, 5, 0.5, 1, 'm/s'], ['F', 'Force F', -100, 100, 5, 40, 'N'], ['tp', 'Time the force acts t', 0.2, 4, 0.1, 2, 's']], P, () => { tt = 0; run = false; draw(); });
  const vel = t => P.u + (P.F/P.m)*Math.min(t, P.tp), Tend = () => P.tp + 2;
  function draw(){ const V = fitView(cv, { xl:0, xr:12, yb:-0.6, yt:2.6 }, 8), { ctx } = V, gx = 1.5 + ((x % 9) + 9) % 9;
    ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(V.PX(0), V.PY(0)); ctx.lineTo(V.PX(12), V.PY(0)); ctx.stroke();
    for (let k = 0; k < 12; k += 1){ ctx.beginPath(); ctx.moveTo(V.PX(k), V.PY(0)); ctx.lineTo(V.PX(k), V.PY(-0.25)); ctx.stroke(); }
    const w = 2.4, h = 1.1, x0 = V.PX(gx - w/2), y0 = V.PY(0.35 + h); ctx.fillStyle = alpha(C.crank, 0.85); ctx.fillRect(x0, y0, w*V.sc, h*V.sc);
    for (const dx of [-0.7, 0.7]){ ctx.fillStyle = C.ink; ctx.beginPath(); ctx.arc(V.PX(gx + dx), V.PY(0.18), 0.18*V.sc, 0, 7); ctx.fill(); }
    ctx.fillStyle = '#fff'; ctx.font = `600 13px ${C.mono}`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText(`${P.m} kg`, V.PX(gx), V.PY(0.35 + h/2));
    if (tt < P.tp && run && P.F){ const s = Math.sign(P.F), L = 1.2 + 2*Math.abs(P.F)/100; const ax = s > 0 ? V.PX(gx - w/2 - 0.1) : V.PX(gx + w/2 + 0.1);
      arrow(ctx, ax - s*L*V.sc, V.PY(0.35 + h/2), ax, V.PY(0.35 + h/2), C.force, 3); lbl(ctx, `F = ${P.F} N`, ax - s*L*V.sc/2, V.PY(0.35 + h) - 10, C.force, 'center'); }
    const v = vel(tt); arrow(ctx, V.PX(gx), V.PY(2), V.PX(gx) + v*V.sc*0.5, V.PY(2), C.vel, 2.2); lbl(ctx, `v = ${n2(v)} m/s`, V.PX(gx) + (v >= 0 ? 8 : -8) + v*V.sc*0.5, V.PY(2), C.vel, v >= 0 ? 'left' : 'right');
    const X = [], Y = [], Pm = []; for (let i = 0; i <= 200; i++){ const q = i/200*Tend(); X.push(q); Y.push(vel(q)); Pm.push(P.m*vel(q)); }
    chart($('nm_g'), { x:X, series:[{ y:Pm, color:C.crank, label:'momentum mv [kg·m/s]' }], ylab:'mv [kg·m/s]', xlab:'t [s]', cursor:Math.min(tt, Tend()), cursorIndex:Math.round(Math.min(tt, Tend())/Tend()*200), shade:[{ a:0, b:P.tp, color:C.force, label:'force acting' }], value:`mv = ${n2(P.m*v, 1)} kg·m/s` });
    const vf = vel(P.tp);
    $('nm_st').innerHTML = statsHTML([['a = F/m', `${n2(P.F/P.m)} <small>m/s²</small>`], ['Final velocity v = u + at', `${n2(vf)} <small>m/s</small>`], ['Change of momentum mv − mu', `${n2(P.m*(vf - P.u), 1)} <small>kg·m/s</small>`], ['Impulse F t', `${n2(P.F*P.tp, 1)} <small>N·s</small>`]]);
    $('nm_sub').textContent = run ? (tt < P.tp ? 'force acting: momentum changing at the rate F' : 'no force: first law, the velocity stays the same') : 'press Push'; }
  $('nm_go').addEventListener('click', () => { tt = 0; run = true; }); $('nm_reset').addEventListener('click', () => { tt = 0; x = 0; run = false; draw(); });
  watchSize(cv, draw); watchSize($('nm_g'), draw); onRedraw(draw);
  animate(cv, dt => { if (run){ tt += dt; x += vel(tt)*dt; if (tt > Tend() + 1.5) run = false; } draw(); }); draw(); }

{ const P = {}, cv = $('mo'); let mode = 'one', O = [-2, -1], T = [1, 0.5], V;
  controls($('mo_c'), [['F', 'Force F', 10, 200, 5, 100, 'N'], ['a', 'Direction of F', 0, 360, 5, 70, '°'], ['x', 'Arm of the couple x', 0.2, 3, 0.1, 1.5, 'm']], P, () => draw());
  function draw(){ V = fitView(cv, { xl:-4.5, xr:4.5, yb:-2.6, yt:2.6 }); const { ctx } = V; const Pp = V.P, u = [Math.cos(P.a*DEG), Math.sin(P.a*DEG)], L = 1.2 + P.F/200*1.2;
    ctx.fillStyle = alpha(C.muted, 0.12); ctx.strokeStyle = C.muted; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.roundRect ? ctx.roundRect(...Pp([-3.8, 2.1]), 7.6*V.sc, 4.2*V.sc, 8) : ctx.rect(...Pp([-3.8, 2.1]), 7.6*V.sc, 4.2*V.sc); ctx.fill(); ctx.stroke();
    const forces = mode === 'one' ? [[T, u]] : [[T, u], [vadd(T, vmul(vperp(u), -P.x)), vmul(u, -1)]];
    let M = 0;
    for (const [p, d] of forces){ const tip = vadd(p, vmul(d, L)); dashLine(ctx, ...Pp(vadd(p, vmul(d, -9))), ...Pp(vadd(p, vmul(d, 9))), alpha(C.force, 0.5), [6, 5], 1);
      arrow(ctx, ...Pp(p), ...Pp(tip), C.force, 3); dotAt(ctx, ...Pp(p), C.force, 5);
      const r = vsub(p, O), m = (r[0]*d[1] - r[1]*d[0])*P.F; M += m; const foot = vadd(p, vmul(d, -(r[0]*d[0] + r[1]*d[1])));
      if (mode === 'one'){ ctx.strokeStyle = C.vel; ctx.lineWidth = 2.5; ctx.beginPath(); ctx.moveTo(...Pp(O)); ctx.lineTo(...Pp(foot)); ctx.stroke();
        lbl(ctx, `l = ${n2(Math.abs(m/P.F))} m`, ...vadd(Pp(vmul(vadd(O, foot), 0.5)), [8, -10]), C.vel); } }
    if (mode === 'couple'){ const p2 = forces[1][0]; ctx.strokeStyle = C.vel; ctx.lineWidth = 2.5; ctx.beginPath(); ctx.moveTo(...Pp(T)); ctx.lineTo(...Pp(p2)); ctx.stroke(); lbl(ctx, `x = ${n2(P.x)} m`, ...vadd(Pp(vmul(vadd(T, p2), 0.5)), [8, 0]), C.vel); }
    pinAt(ctx, ...Pp(O), 6); lbl(ctx, 'O', Pp(O)[0] - 10, Pp(O)[1] + 12, C.ink, 'right');
    const s = M >= 0 ? 'anticlockwise' : 'clockwise', o = Pp(O); ctx.strokeStyle = C.acc; ctx.lineWidth = 2.2; ctx.beginPath(); const r0 = 22; if (M >= 0) ctx.arc(o[0], o[1], r0, 0.2*PI, 1.6*PI); else ctx.arc(o[0], o[1], r0, 1.6*PI, 0.2*PI, true); ctx.stroke();
    const ea = M >= 0 ? 1.6*PI : 0.2*PI, ex = o[0] + r0*Math.cos(ea), ey = o[1] + r0*Math.sin(ea), tg = M >= 0 ? [-Math.sin(ea), Math.cos(ea)] : [Math.sin(ea), -Math.cos(ea)];
    if (Math.abs(M) > 1e-6) arrow(ctx, ex - tg[0]*6, ey - tg[1]*6, ex + tg[0]*2, ey + tg[1]*2, C.acc, 2.2);
    $('mo_st').innerHTML = statsHTML(mode === 'one' ? [['Force F', `${P.F} <small>N</small>`], ['Perpendicular distance l', `${n2(Math.abs(M)/P.F)} <small>m</small>`], ['Moment about O, F × l', `${n2(Math.abs(M), 1)} <small>N·m</small>`], ['Sense', s]]
      : [['Each force F', `${P.F} <small>N</small>`], ['Arm x', `${n2(P.x)} <small>m</small>`], ['F × x', `${n2(P.F*P.x, 1)} <small>N·m</small>`], ['Moment about O (move O: it stays)', `${n2(Math.abs(M), 1)} <small>N·m</small>`], ['Sense', s]]);
    $('mo_sub').textContent = mode === 'one' ? 'the moment depends on where O is' : 'net force zero; the turning effect is the same about any point'; }
  dragOn(cv, (x, y) => !V ? null : Math.hypot(x - V.PX(O[0]), y - V.PY(O[1])) < 14 ? 'O' : Math.hypot(x - V.PX(T[0]), y - V.PY(T[1])) < 14 ? 'T' : null,
    (h, x, y) => { const p = [clamp(V.IX(x), -3.7, 3.7), clamp(V.IY(y), -2, 2)]; if (h === 'O') O = p; else T = p; draw(); });
  segGroup($('mo_seg'), m => { mode = m; draw(); }); watchSize(cv, draw); onRedraw(draw); draw(); }

{ const P = {}, cv = $('wh'); let th = 0, cut = null;
  controls($('wh_c'), [['m', 'Mass m', 0.05, 2, 0.05, 0.2, 'kg'], ['r', 'Radius r', 0.2, 1.5, 0.05, 0.8, 'm'], ['N', 'Speed', 10, 240, 5, 60, 'rpm', v => `ω = ${n2(v*TAU/60, 2)} rad/s`]], P, () => draw());
  function draw(){ const V = fitView(cv, { xl:-3.2, xr:3.2, yb:-2, yt:2 }), { ctx } = V, Pp = V.P, w = P.N*TAU/60, sp = w*P.r;
    ctx.strokeStyle = alpha(C.muted, 0.5); ctx.setLineDash([4, 4]); ctx.beginPath(); ctx.arc(V.PX(0), V.PY(0), P.r*V.sc, 0, 7); ctx.stroke(); ctx.setLineDash([]);
    pinAt(ctx, V.PX(0), V.PY(0), 6);
    let pos, vdir;
    if (cut){ pos = vadd(cut.p, vmul(cut.v, cut.t)); vdir = vmul(cut.v, 1/vlen(cut.v)); dashLine(ctx, ...Pp(cut.p), ...Pp(pos), C.vel, [3, 4], 1.4); }
    else { pos = [P.r*Math.cos(th), P.r*Math.sin(th)]; vdir = [-Math.sin(th), Math.cos(th)];
      ctx.strokeStyle = C.ink; ctx.lineWidth = 1.5; ctx.beginPath(); ctx.moveTo(V.PX(0), V.PY(0)); ctx.lineTo(...Pp(pos)); ctx.stroke(); }
    const pp = Pp(pos), Lv = 30 + sp*12, rad = 6 + 10*Math.cbrt(P.m/2);
    arrow(ctx, ...pp, pp[0] + vdir[0]*Lv, pp[1] - vdir[1]*Lv, C.vel, 2.5); lbl(ctx, 'v', pp[0] + vdir[0]*Lv + 6, pp[1] - vdir[1]*Lv, C.vel);
    if (!cut){ const ac = sp*sp/P.r, La = 18 + Math.min(70, ac*1.2), u = vmul(pos, -1/P.r); arrow(ctx, ...pp, pp[0] + u[0]*La, pp[1] - u[1]*La, C.acc, 2.5); lbl(ctx, 'a꜀', pp[0] + u[0]*La + 6, pp[1] - u[1]*La - 8, C.acc); }
    dotAt(ctx, ...pp, C.rod, rad);
    const ac = w*w*P.r;
    $('wh_st').innerHTML = statsHTML([['Speed v = ωr', `${n2(sp)} <small>m/s</small>`], ['Centripetal acceleration ω²r', `${n2(ac, 1)} <small>m/s²</small>`], ['Centripetal force mω²r (string tension)', `${n2(P.m*ac, 2)} <small>N</small>`], ['Times the weight', n2(ac/9.81, 1)]]);
    $('wh_sub').textContent = cut ? 'string cut: no force, straight line along the tangent' : 'the string pulls the mass inwards; the mass pulls the string outwards'; }
  $('wh_cut').addEventListener('click', () => { if (cut) return; const w = P.N*TAU/60; cut = { p:[P.r*Math.cos(th), P.r*Math.sin(th)], v:[-Math.sin(th)*w*P.r, Math.cos(th)*w*P.r], t:0 }; $('wh_cut').disabled = true; });
  watchSize(cv, draw); onRedraw(draw);
  const an = animate(cv, dt => { const w = P.N*TAU/60; if (cut){ cut.t += dt*Math.min(1, 1.2/w); const q = vadd(cut.p, vmul(cut.v, cut.t)); if (Math.abs(q[0]) > 3.4 || Math.abs(q[1]) > 2.2){ cut = null; $('wh_cut').disabled = false; } }
    th += dt*Math.min(w, 2.5); draw(); }); playButton($('wh_play'), an); draw(); }

/* ================= quiz ================= */
quiz($('qz'), [
  { q:'Which branch of theory of machines studies relative motion without asking what forces cause it?', o:['Statics', 'Kinematics', 'Kinetics', 'Dynamics'], a:1, why:'Kinematics is the geometry of motion: positions, velocities and accelerations only.' },
  { q:'A cam and its follower touch along a line. What kind of pair is this?', o:['A lower pair', 'A higher pair', 'A turning pair', 'Not a pair'], a:1, why:'Point or line contact makes a higher pair; surface contact (a pin, a slider) makes a lower pair.' },
  { q:'A planar linkage has 4 links (including the frame) joined by 4 pin joints. How many degrees of freedom?', o:['0', '1', '2', '3'], a:1, why:'3(4 − 1) − 2×4 − 0 = 9 − 8 = 1: the four-bar chain needs one input.' },
  { q:'Three links pinned into a triangle give 3(3 − 1) − 2×3 = 0. What does that mean?', o:['A mechanism with one input', 'A structure: no relative motion', 'It needs two inputs', 'The formula does not apply'], a:1, why:'Zero degrees of freedom means nothing can move: the triangle is a structure.' },
  { q:'On a v–t graph, what does the area under the curve between t₁ and t₂ give?', o:['The acceleration', 'The change in velocity', 'The displacement s₂ − s₁', 'The average speed'], a:2, why:'Area under v–t is ∫v dt = s₂ − s₁. The area under a–t gives the change in velocity.' },
  { q:'A particle has v = −4 m/s and a = +2 m/s². It is…', o:['speeding up', 'slowing down', 'moving at steady speed', 'at rest'], a:1, why:'a and v have opposite signs, so the magnitude of v is falling: it is decelerating, even though a is positive.' },
  { q:'For a particle on a curved path, the velocity vector is always…', o:['towards the centre of the path', 'along the position vector r', 'tangent to the path', 'parallel to the acceleration'], a:2, why:'v = dr/dt is the limit of Δr/Δt, and the chord Δr turns into the tangent as Δt → 0.' },
  { q:'In projectile motion (no drag), which quantity stays constant?', o:['The vertical velocity', 'The speed', 'The horizontal velocity', 'The direction of motion'], a:2, why:'aₓ = 0, so vₓ = (vₓ)₀ throughout; only the vertical component changes, at −g.' },
  { q:'A 0.2 kg mass whirls at 2 m/s on a 0.5 m string. The string tension is…', o:['0.4 N', '0.8 N', '1.6 N', '4 N'], a:2, why:'F = m v²/r = 0.2 × 4/0.5 = 1.6 N, directed inwards on the mass.' },
  { q:'The moment of a couple about a point O…', o:['is largest when O is on one of the forces', 'is the same wherever O is', 'is zero because the forces cancel', 'depends on the distance from O to the nearer force'], a:1, why:'The forces cancel, but their turning effect does not: it is F × x about every point.' } ]);
})();
