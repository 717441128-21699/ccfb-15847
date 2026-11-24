// 页面三：一跳慢放分析——回放、角动量守恒曲线、三姿态惯量比、逐帧数据
import { DiveRenderer } from '../game/renderer';
import { app, navigate } from '../main';
import { FrameSnapshot } from '../game/session';
import { ATHLETES } from '../physics/bodyModel';
import {
  POSE_PIKE, POSE_STRAIGHT, POSE_TUCK,
} from '../physics/bodyModel';
import { buildMassMatrix } from '../physics/kinematics';
import { rad2deg } from '../physics/math';

export function renderAnalysisPage(host: HTMLElement): void {
  const result = app.lastResult;
  if (!result) {
    host.innerHTML = `<section class="card"><h2>尚无完成的一跳</h2>
      <p class="muted">先到跳台页完成一次跳水。</p>
      <button class="btn primary" style="width:auto; padding:10px 22px" id="go">去跳台</button></section>`;
    host.querySelector<HTMLButtonElement>('#go')!.onclick = () => navigate('platform');
    return;
  }
  const frames = result.snapshots;
  const splash0 = result.score?.splashSize ?? 0.5;
  const flightFrames = frames.filter((f) => f.phase !== 'contact');

  // 三姿态翻转惯量（当前运动员），给出理论周数比
  const izz = (pose: typeof POSE_TUCK) => buildMassMatrix(app.athlete, [0, 0, 0, 1], pose).Iroot[8];
  const iTuck = izz(POSE_TUCK);
  const iPike = izz(POSE_PIKE);
  const iStraight = izz(POSE_STRAIGHT);
  const inv = [1 / iTuck, 1 / iPike, 1 / iStraight];
  const theoryRatio = inv.map((x) => x / inv[0]);

  host.innerHTML = `
    <div class="grid analysis" style="flex:1; min-height:0">
      <div style="display:flex; flex-direction:column; gap:12px; min-height:0">
        <section class="card" style="padding:10px; flex:1; min-height:260px; display:flex">
          <div class="canvas-wrap" style="flex:1">
            <canvas class="gl" id="gl-canvas"></canvas>
            <div class="hud">
              <span class="chip">t <b id="r-t">0.00</b> s</span>
              <span class="chip">翻转 <b id="r-som">0.00</b></span>
              <span class="chip">转体 <b id="r-tw">0.00</b></span>
              <span class="chip">入水角 <b>${result.entryAngleDeg.toFixed(1)}°</b></span>
            </div>
          </div>
        </section>
        <section class="card">
          <div class="scrub">
            <button class="btn" id="play">▶ 播放</button>
            <input type="range" id="scrub" min="0" max="${frames.length - 1}" value="0">
            <span class="muted" id="frame-no">1/${frames.length}</span>
          </div>
          <div class="speed-pick" style="margin-top:10px">
            ${[0.15, 0.3, 0.6, 1].map((sp, i) =>
              `<button class="btn ${i === 1 ? 'on' : ''}" data-speed="${sp}">${sp}×</button>`).join('')}
          </div>
        </section>
        <section class="card">
          <h2>守恒与惯量曲线（腾空中）</h2>
          <div class="chart-box"><div class="cap">总角动量 |H|（应水平直线，守恒）</div><canvas class="chart" id="c-h"></canvas></div>
          <div class="chart-box"><div class="cap">翻转惯量 Izz（团身小→打开大，角速度反向变化）</div><canvas class="chart" id="c-izz"></canvas></div>
          <div class="chart-box"><div class="cap">翻转角速度（rad/s，由 H/Izz 决定）</div><canvas class="chart" id="c-omega"></canvas></div>
        </section>
      </div>

      <aside class="card">
        <h2>物理验收数据</h2>
        <h3>三姿态周数比（H 相同）</h3>
        <table class="kvtable">
          <tr><td>团身 Izz / 相对周数</td><td>${iTuck.toFixed(3)} / 1.000</td></tr>
          <tr><td>屈体 Izz / 相对周数</td><td>${iPike.toFixed(3)} / ${theoryRatio[1].toFixed(3)}</td></tr>
          <tr><td>直体 Izz / 相对周数</td><td>${iStraight.toFixed(3)} / ${theoryRatio[2].toFixed(3)}</td></tr>
        </table>
        <h3>本跳角动量</h3>
        <table class="kvtable">
          <tr><td>初始 |H|（kg·m²/s）</td><td>${(flightFrames[0]?.hMag ?? 0).toFixed(3)}</td></tr>
          <tr><td>入水前 |H|</td><td>${(flightFrames[flightFrames.length - 1]?.hMag ?? 0).toFixed(3)}</td></tr>
          <tr><td>守恒相对偏差</td><td id="h-dev">计算中…</td></tr>
          <tr><td>轴向 Hyaw（转体）</td><td>${(flightFrames[0]?.hyaw ?? 0).toFixed(3)}</td></tr>
        </table>
        <h3>逐帧（当前帧）</h3>
        <table class="kvtable" id="frame-table"></table>
        <h3>入水</h3>
        <table class="kvtable">
          <tr><td>身体与竖直夹角</td><td>${result.entryAngleDeg.toFixed(1)}°</td></tr>
          <tr><td>手掌先破水</td><td>${result.state.entry?.handsFirst ? '是' : '否'}</td></tr>
          <tr><td>翻转零头</td><td>${Math.abs(result.residualSomDeg).toFixed(1)}°</td></tr>
          <tr><td>水花指数</td><td>${(result.score?.splashSize ?? 0).toFixed(2)}</td></tr>
        </table>
        <button class="btn primary" style="margin-top:14px" id="to-judge">查看裁判打分 →</button>
      </aside>
    </div>`;

  // 守恒偏差
  const h0 = flightFrames[0]?.hMag ?? 1;
  const h1 = flightFrames[flightFrames.length - 1]?.hMag ?? h0;
  const dev = Math.abs(h1 - h0) / Math.max(1e-9, h0);
  host.querySelector('#h-dev')!.textContent = `${(dev * 100).toExponential(1)} %`;

  drawSeries(host.querySelector<HTMLCanvasElement>('#c-h')!, flightFrames.map((f) => f.hMag));
  drawSeries(host.querySelector<HTMLCanvasElement>('#c-izz')!, flightFrames.map((f) => f.izz));
  drawSeries(
    host.querySelector<HTMLCanvasElement>('#c-omega')!,
    flightFrames.map((f) => f.hMag / Math.max(1e-6, f.izz)),
  );

  // 回放
  const canvas = host.querySelector<HTMLCanvasElement>('#gl-canvas')!;
  const renderer = new DiveRenderer(canvas);
  renderer.cameraTarget = [0, 5.5, 0];
  let idx = 0;
  let playing = false;
  let speed = 0.3;
  let acc = 0;
  let last = performance.now();
  const scrub = host.querySelector<HTMLInputElement>('#scrub')!;
  const table = host.querySelector<HTMLTableElement>('#frame-table')!;

  function showFrame(i: number): void {
    idx = Math.max(0, Math.min(frames.length - 1, i));
    const f = frames[idx];
    renderer.resetSplash();
    renderer.render(app.athlete, f, splash0);
    scrub.value = String(idx);
    host.querySelector<HTMLElement>('#frame-no')!.textContent = `${idx + 1}/${frames.length}`;
    host.querySelector<HTMLElement>('#r-t')!.textContent = f.t.toFixed(2);
    host.querySelector<HTMLElement>('#r-som')!.textContent = f.somTurns.toFixed(2);
    host.querySelector<HTMLElement>('#r-tw')!.textContent = f.twTurns.toFixed(2);
    table.innerHTML = `
      <tr><td>质心高度</td><td>${f.com[1].toFixed(3)} m</td></tr>
      <tr><td>质心速度</td><td>${f.comVel.map((x) => x.toFixed(2)).join(', ')}</td></tr>
      <tr><td>Izz / Iyy</td><td>${f.izz.toFixed(3)} / ${f.iyy.toFixed(3)}</td></tr>
      <tr><td>|H| / Hyaw</td><td>${f.hMag.toFixed(3)} / ${f.hyaw.toFixed(3)}</td></tr>
      <tr><td>姿态 hip/knee</td><td>${rad2deg(f.pose.hip).toFixed(0)}° / ${rad2deg(f.pose.knee).toFixed(0)}°</td></tr>`;
  }

  function loop(now: number): void {
    const dt = (now - last) / 1000;
    last = now;
    if (playing) {
      acc += dt * speed;
      const stepFrames = Math.floor(acc / (1 / 120));
      if (stepFrames > 0) {
        acc -= stepFrames / 120;
        idx += stepFrames;
        if (idx >= frames.length - 1) { idx = frames.length - 1; playing = false; host.querySelector<HTMLButtonElement>('#play')!.textContent = '▶ 播放'; }
        showFrame(idx);
      }
    }
    requestAnimationFrame(loop);
  }
  host.querySelector<HTMLButtonElement>('#play')!.onclick = (e) => {
    playing = !playing;
    (e.target as HTMLButtonElement).textContent = playing ? '⏸ 暂停' : '▶ 播放';
    if (idx >= frames.length - 1) { idx = 0; showFrame(0); }
  };
  scrub.addEventListener('input', () => { playing = false; showFrame(parseInt(scrub.value, 10)); });
  host.querySelectorAll<HTMLButtonElement>('[data-speed]').forEach((b) => {
    b.onclick = () => {
      speed = parseFloat(b.dataset.speed!);
      host.querySelectorAll('[data-speed]').forEach((x) => x.classList.remove('on'));
      b.classList.add('on');
    };
  });
  host.querySelector<HTMLButtonElement>('#to-judge')!.onclick = () => navigate('judge');  showFrame(0);
  requestAnimationFrame(loop);
}

function drawSeries(canvas: HTMLCanvasElement, data: number[]): void {
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const w = canvas.clientWidth * dpr;
  const h = canvas.clientHeight * dpr;
  canvas.width = w; canvas.height = h;
  const ctx = canvas.getContext('2d')!;
  ctx.scale(dpr, dpr);
  const W = canvas.clientWidth; const H = canvas.clientHeight;
  ctx.clearRect(0, 0, W, H);
  if (data.length < 2) return;
  const min = Math.min(...data);
  const max = Math.max(...data);
  const span = max - min || 1;
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 1.6;
  ctx.beginPath();
  data.forEach((v, i) => {
    const x = (i / (data.length - 1)) * W;
    const y = H - 6 - ((v - min) / span) * (H - 14);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
  ctx.fillStyle = '#8fa3c0';
  ctx.font = '10px monospace';
  ctx.fillText(max.toFixed(2), 4, 12);
  ctx.fillText(min.toFixed(2), 4, H - 4);
}

void ATHLETES;
