/* BME 2102 interactive notes: slide view. Turns the page into slides, one idea per slide.
   The page's own elements are moved (not copied), so every figure, control and example keeps working.
   Add ?view=page to the address to read it as one scrolling page instead. */
(() => {
'use strict';
if (new URLSearchParams(location.search).get('view') === 'page'){
  const a = document.createElement('a'); a.href = location.pathname + location.hash.replace(/^#slide-\d+/, ''); a.textContent = 'Show as slides'; a.className = 'toslides';
  const bar = document.querySelector('.sitebar'); if (bar) bar.appendChild(a); return; }
document.documentElement.classList.add('slidemode');
const main = document.querySelector('main'), header = document.querySelector('header.top'), chapter = header.querySelector('h1').textContent, eyebrow = header.querySelector('.eyebrow').textContent;
const slides = [];   // { sec, title, el, secId }
const SECOF = {}; let CURSEC = null;
const deck = document.createElement('div'); deck.className = 'deck'; deck.setAttribute('aria-live', 'polite');
function newSlide(secTitle, num, sub, kind){ const s = document.createElement('section'); s.className = 'slide' + (kind ? ' ' + kind : ''); s.setAttribute('aria-roledescription', 'slide');
  const h = document.createElement('div'); h.className = 'slidehead'; h.innerHTML = `<span class="sn">${num}</span><h2>${secTitle}</h2>${sub ? `<span class="ss">${sub}</span>` : ''}`; s.appendChild(h);
  const body = document.createElement('div'); body.className = 'slidebody'; s.appendChild(body); deck.appendChild(s); slides.push({ sec:num, title:secTitle, sub, el:s, body, secId:CURSEC }); return body; }
// title slide
{ const s = document.createElement('section'); s.className = 'slide title'; s.innerHTML = `<div class="tbox"><div class="eyebrow">${eyebrow}</div><h1>${chapter}</h1></div>`;
  s.querySelector('.tbox').appendChild(header.querySelector('p')); const ol = document.createElement('ol'); ol.className = 'tlist';
  main.querySelectorAll('section.sec').forEach(sec => { const h2 = sec.querySelector('h2'); if (!h2 || sec.id === 'refs') return; const li = document.createElement('li'); li.innerHTML = `<span>${h2.querySelector('.n').textContent}</span>${h2.childNodes[1] ? h2.childNodes[1].textContent : h2.textContent}`; li.dataset.sec = sec.id; ol.appendChild(li); });
  s.querySelector('.tbox').appendChild(ol); const hint = document.createElement('p'); hint.className = 'hint'; hint.innerHTML = 'Click, press → or Space for the next slide · ← to go back · <a href="?view=page">read as one page</a>'; s.querySelector('.tbox').appendChild(hint);
  deck.appendChild(s); slides.push({ sec:'', title:chapter, el:s, body:s }); }
// content slides
const BIG = el => el.matches('.panel, details.ex, .quiz, .tablewrap, table');
const textLen = el => (el.textContent || '').length;
main.querySelectorAll('section.sec').forEach(sec => {
  CURSEC = sec.id; SECOF[sec.id] = true;
  const h2 = sec.querySelector('h2'), num = h2.querySelector('.n').textContent, title = h2.childNodes[1] ? h2.childNodes[1].textContent : h2.textContent;
  const kids = [...sec.children].filter(c => c !== h2);
  if (sec.id === 'quiz'){ const qz = sec.querySelector('.quiz'); const qs = [...qz.querySelectorAll('.q')], score = qz.querySelector('.score');
    qs.forEach((q, i) => { const b = newSlide(title, num, `question ${i + 1} of ${qs.length}`, 'qslide'); const wrap = document.createElement('div'); wrap.className = 'quiz'; wrap.appendChild(q); b.appendChild(wrap); if (i === qs.length - 1 && score) b.appendChild(score); });
    return; }
  if (sec.id === 'refs'){ const b = newSlide(title, num, '', 'refs'); kids.forEach(k => b.appendChild(k)); return; }
  let body = null, len = 0, sub = '', n = 0;
  const flush = () => { body = null; len = 0; };
  for (const k of kids){
    if (k.tagName === 'H3'){ flush(); sub = k.textContent; body = newSlide(title, num, sub); n++; continue; }
    if (BIG(k)){ // a figure or worked example gets its own slide, with any short lead-in text kept above it
      const lead = body && len < 260 && body.children.length && !body.querySelector('.panel, details') ? body : null;
      let b; if (lead){ b = lead; b.parentElement.classList.add('withfig'); } else { b = newSlide(title, num, k.matches('details.ex') ? '' : sub); n++; }
      if (k.matches('details.ex')){ k.open = true; b.parentElement.classList.add('exslide'); const sm = k.querySelector('summary'); sm.addEventListener('click', e => e.preventDefault()); }
      b.appendChild(k); flush(); continue; }
    const L = textLen(k);
    if (!body || len + L > 900 || body.querySelector('.panel, details')){ body = newSlide(title, num, sub); n++; len = 0; }
    body.appendChild(k); len += L; }
});
// put the deck in place of the page layout
const wrap = document.querySelector('.wrap'); wrap.innerHTML = ''; wrap.appendChild(deck);
document.querySelector('.sitebar').remove();
// controls
const ui = document.createElement('div'); ui.className = 'deckbar';
const pages = document.querySelectorAll('link[rel=next], link[rel=prev]');
ui.innerHTML = `<a class="home" href="./" title="All chapters">BME 2102</a><span class="where"><b class="ch">${chapter}</b><span class="st"></span></span>
  <select class="jump" aria-label="Go to slide"></select>
  <span class="count" aria-live="polite"></span>
  <button type="button" class="prev" aria-label="Previous slide">←</button><button type="button" class="next primary" aria-label="Next slide">Next →</button>
  <button type="button" class="fs" title="Full screen (F)" aria-label="Full screen">⛶ Full screen</button>
  <a class="pagev" href="?view=page" title="Read as one scrolling page">Page view</a><div class="prog"><i></i></div>`;
document.body.appendChild(ui);
const sel = ui.querySelector('.jump'); sel.innerHTML = slides.map((s, i) => `<option value="${i}">${i + 1}. ${s.sec ? s.sec + ' ' : ''}${s.title}${s.sub ? ' – ' + s.sub : ''}</option>`).join('');
let cur = -1;
function go(i, focus){ i = Math.max(0, Math.min(slides.length - 1, i)); if (i === cur) return; const prev = slides[cur];
  if (prev) prev.el.classList.remove('on'); cur = i; const s = slides[i]; s.el.classList.add('on'); s.el.scrollTop = 0;
  ui.querySelector('.st').textContent = s.sec ? `${s.sec} ${s.title}` : ''; ui.querySelector('.count').textContent = `${i + 1} / ${slides.length}`;
  ui.querySelector('.prog i').style.width = `${100*(i + 1)/slides.length}%`; sel.value = i; ui.querySelector('.prev').disabled = i === 0;
  const nx = ui.querySelector('.next'); nx.textContent = i === slides.length - 1 ? (NEXTCH ? 'Next chapter →' : 'End') : 'Next →';
  history.replaceState(null, '', `#slide-${i + 1}`);
  window.dispatchEvent(new Event('resize')); setTimeout(() => moreCue(s.el), 60); }
const cue = document.createElement('div'); cue.className = 'morecue'; cue.textContent = 'Scroll for more ↓'; document.body.appendChild(cue);
function moreCue(el){ const more = el.scrollHeight - el.clientHeight - el.scrollTop > 30; cue.classList.toggle('show', more); }
deck.addEventListener('scroll', e => moreCue(e.target), true);
const order = ['intro.html', 'mechanisms.html', 'velocity.html', 'belts.html', 'gearing.html', 'cams.html', 'balancing.html'], me = location.pathname.split('/').pop(), NEXTCH = order[order.indexOf(me) + 1];
const next = () => { if (cur === slides.length - 1){ if (NEXTCH) location.href = NEXTCH; return; } go(cur + 1); }, prev = () => go(cur - 1);
ui.querySelector('.next').addEventListener('click', next); ui.querySelector('.prev').addEventListener('click', prev); sel.addEventListener('change', () => go(+sel.value));
// title slide: clicking a section jumps to it
slides[0].el.querySelectorAll('.tlist li').forEach(li => li.addEventListener('click', e => { e.stopPropagation(); const i = slides.findIndex(s => s.sec && s.sec === li.querySelector('span').textContent); if (i >= 0) go(i); }));
// keyboard (not while typing in a field)
document.addEventListener('keydown', e => { const t = e.target; if (t.matches && t.matches('input, select, textarea, [contenteditable]')) return; if (e.altKey || e.ctrlKey || e.metaKey) return;
  if (['ArrowRight', 'PageDown', ' '].includes(e.key)){ if (e.key === ' ' && t.matches && t.matches('button, summary, a')) return; e.preventDefault(); next(); }
  else if (['ArrowLeft', 'PageUp'].includes(e.key)){ e.preventDefault(); prev(); } else if (e.key === 'Home'){ e.preventDefault(); go(0); } else if (e.key === 'End'){ e.preventDefault(); go(slides.length - 1); } else if (e.key === 'f' || e.key === 'F'){ e.preventDefault(); toggleFS(); } });
// click on plain parts of a slide advances; anything interactive is left alone
const INTERACTIVE = 'a, button, input, select, textarea, label, summary, canvas, details .exbody, .panel, table, .q, .chips, [role=button]';
deck.addEventListener('click', e => { if (window.getSelection && String(window.getSelection()).length) return; if (e.target.closest(INTERACTIVE)) return; const r = deck.getBoundingClientRect(); if (e.clientX < r.left + r.width*0.2) prev(); else next(); });
// swipe on touch screens (not on figures)
let sx = null, sy = null; deck.addEventListener('touchstart', e => { if (e.target.closest('canvas, input, .panel')){ sx = null; return; } sx = e.touches[0].clientX; sy = e.touches[0].clientY; }, { passive:true });
deck.addEventListener('touchend', e => { if (sx == null) return; const dx = e.changedTouches[0].clientX - sx, dy = e.changedTouches[0].clientY - sy; if (Math.abs(dx) > 60 && Math.abs(dx) > 1.5*Math.abs(dy)){ dx < 0 ? next() : prev(); } sx = null; }, { passive:true });
// buttons in the page that scroll to a section (e.g. "Draw it" in the cam examples) switch to that section's first figure slide instead
const origScroll = Element.prototype.scrollIntoView;
Element.prototype.scrollIntoView = function(o){ const id = this.id && SECOF[this.id] != null ? this.id : null;
  if (id){ const list = slides.map((s, i) => [s, i]).filter(([s]) => s.secId === id), fig = list.find(([s]) => s.el.querySelector('.panel')); go((fig || list[0])[1]); return; }
  const i = slides.findIndex(s => s.el.contains(this)); if (i >= 0){ go(i); return; } return origScroll.call(this, o); };
const m = location.hash.match(/^#slide-(\d+)/); go(m ? +m[1] - 1 : 0);
})();

/* ---------- full screen ---------- */
(() => {
  const btn = document.querySelector('.deckbar .fs'), root = document.documentElement; if (!btn) return;
  const fsEl = () => document.fullscreenElement || document.webkitFullscreenElement;
  const can = !!(root.requestFullscreen || root.webkitRequestFullscreen);
  if (!can){ btn.hidden = true; }
  window.toggleFS = () => { if (!can) return; if (fsEl()) (document.exitFullscreen || document.webkitExitFullscreen).call(document); else { const p = (root.requestFullscreen || root.webkitRequestFullscreen).call(root); if (p && p.catch) p.catch(() => {}); } };
  btn.addEventListener('click', toggleFS);
  const bar = document.querySelector('.deckbar');
  // in full screen the bar is hidden at once; it comes back only while the mouse is at the very top, and the slide moves down so nothing is covered
  const hide = () => { if (fsEl()) root.classList.add('barhid'); }, show = () => root.classList.remove('barhid');
  const change = () => { const on = !!fsEl(); root.classList.toggle('fsmode', on); btn.textContent = on ? '✕ Exit full screen' : '⛶ Full screen'; if (on){ hide(); btn.blur(); } else show(); window.dispatchEvent(new Event('resize')); };
  document.addEventListener('fullscreenchange', change); document.addEventListener('webkitfullscreenchange', change);
  document.addEventListener('mousemove', e => { if (!fsEl()) return; if (e.clientY <= 6) show(); else if (!bar.contains(e.target) && e.clientY > 64) hide(); });
  bar.addEventListener('mouseleave', hide);
  document.addEventListener('keydown', hide); document.querySelector('.deck').addEventListener('click', hide);
  ['.next', '.prev'].forEach(c => bar.querySelector(c).addEventListener('click', () => { hide(); bar.querySelector(c).blur(); }));
  // stay in full screen when moving to the next chapter: remember and offer to resume
  try { if (sessionStorage.getItem('bme-fs') === '1'){ sessionStorage.removeItem('bme-fs'); const tip = document.createElement('button'); tip.type = 'button'; tip.className = 'fsresume'; tip.textContent = '⛶ Back to full screen'; tip.addEventListener('click', () => { tip.remove(); toggleFS(); }); document.body.appendChild(tip); setTimeout(() => tip.remove(), 8000); } } catch (e) {}
  window.addEventListener('beforeunload', () => { try { if (fsEl()) sessionStorage.setItem('bme-fs', '1'); } catch (e) {} });
})();
