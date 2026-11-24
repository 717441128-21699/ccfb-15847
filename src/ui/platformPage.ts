// 页面二：跳台——起跳输入、空中姿态控制、实时物理 HUD
import { DiveRenderer } from '../game/renderer';
import { DEFAULT_TAKEOFF, TakeoffInputs } from '../game/session';
import { app, finishSession, navigate, newSession } from '../main';
import { POSE_PIKE, POSE_STRAIGHT, POSE_TUCK } from '../physics/bodyModel';

export function renderPlatformPage(host: HTMLElement): void {
  let session = app.session ?? newSession(app.autoPilot);
  const inputs: TakeoffInputs = { ...DEFAULT_TAKEOFF };
  let raf = 0;
  let last = performance.now();
  let launched = false;

  host.innerHTML = `
    <div class="grid two" style="flex:1; min-height:0">
      <section class="card" style="display:flex; flex-direction:column; gap:6px">
        <h2>${app.def.code} · ${app.def.name}</h2>
        <div class="muted">目标 ${app.def.halfSomersaults / 2} 周翻转
          ${app.def.halfTwists ? `＋ ${app.def.halfTwists / 2} 周转体` : ''}，DD ${app.def.dd}</div>

        <h3>起跳（蹬台瞬间决定一切）</h3>
        ${slider('push', '蹬台力度', 0.5, 1.5, 0.01, inputs.push, '× 基准冲量 → 腾空高度')}
        ${slider('lean', '前倾/翻转意图', 0, 1.6, 0.01, inputs.lean, '→ 初始翻转角动量')}
        ${slider('twist', '转体意图', 0, 1.6, 0.01, inputs.twist, '→ 纵轴角动量（5 字头动作）')}
        <button class="btn primary" id="launch">起跳！</button>

        <h3>空中姿态（腾空中无外力，只改惯量分配）</h3>
        <div class="pose-buttons">
          <button data-pose="tuck">团身<br><span class="muted">最快</span></button>
          <button data-pose="pike">屈体<br><span class="muted">次之</span></button>
          <button data-pose="straight">打开<br><span class="muted">最慢</span></button>
        </div>
        ${slider('arms', '双臂不对称（转体调节）', -1, 1, 0.05, 0, '偏轴手臂微调轴向惯量')}

        <h3>模式</h3>
        <div class="row">
          <button class="btn ${app.autoPilot ? 'on' : ''}" id="mode-auto">自动示范</button>
          <button class="btn ${!app.autoPilot ? 'on' : ''}" id="mode-manual">手动操控</button>
        </div>
        <p class="muted" id="mode-hint"></p>
        <button class="btn ghost" id="restart">重置本跳</button>
      </section>

      <section class="card" style="padding:10px; display:flex; flex-direction:column">
        <div class="canvas-wrap" style="flex:1">
          <canvas class="gl" id="gl-canvas"></canvas>
          <div class="hud">
            <span class="chip">阶段 <b id="h-phase">待起跳</b></span>
            <span class="chip">翻转 <b id="h-som">0.00</b> 周</span>
            <span class="chip">转体 <b id="h-tw">0.00</b> 周</span>
            <span class="chip">高度 <b id="h-height">0.0</b> m</span>
            <span class="chip">Izz <b id="h-izz">–</b></span>
            <span class="chip">H <b id="h-hmag">–</b></span>
          </div>
          <div class="flash" id="flash"></div>
        </div>
      </section>
    </div>`;

  const canvas = host.querySelector<HTMLCanvasElement>('#gl-canvas')!;
  const renderer = new DiveRenderer(canvas);
  renderer.cameraTarget = [0, 6, 0];
  attachCamera(canvas, renderer);

  const $ = <T extends HTMLElement>(id: string) => host.querySelector<T>('#' + id)!;
  const launchBtn = $('launch') as HTMLButtonElement;
  const syncModeHint = () => {
    $('mode-hint').textContent = app.autoPilot
      ? '自动示范：规划器按动作表闭环控制姿态，入水角与周数由物理保证。'
      : '手动操控：起跳后用团身/屈体/打开按钮决定抱紧时机，自己找打开点。';
  };
  syncModeHint();

  host.querySelectorAll<HTMLButtonElement>('[data-pose]').forEach((b) => {
    b.onclick = () => {
      if (app.autoPilot || !launched) return;
      const map = { tuck: POSE_TUCK, pike: POSE_PIKE, straight: POSE_STRAIGHT } as const;
      session.setManualPose(b.dataset.pose as 'tuck' | 'pike' | 'straight', session.state.pose.twistArms);
      host.querySelectorAll('[data-pose]').forEach((x) => x.classList.remove('on'));
      b.classList.add('on');
    };
  });
  const armsSlider = host.querySelector<HTMLInputElement>('#arms')!;
  armsSlider.addEventListener('input', () => {
    if (!app.autoPilot && launched) session.setTwistArms(parseFloat(armsSlider.value));
  });
  host.querySelectorAll<HTMLInputElement>('input[type="range"]').forEach((el) => {
    if (el.id === 'arms') return;
    el.addEventListener('input', () => {
      inputs[el.id as keyof TakeoffInputs] = parseFloat(el.value);
      el.parentElement!.querySelector('b')!.textContent = parseFloat(el.value).toFixed(2);
    });
  });

  $('mode-auto').onclick = () => { app.autoPilot = true; reset(); };
  $('mode-manual').onclick = () => { app.autoPilot = false; reset(); };
  $('restart').onclick = () => reset();
  launchBtn.onclick = () => {
    if (launched) return;
    launched = true;
    session.launch(inputs);
    launchBtn.disabled = true;
    flash('已离台 — 空中无外力矩，角动量守恒');
  };

  function reset(): void {
    cancelAnimationFrame(raf);
    session = newSession(app.autoPilot);
    launched = false;
    renderer.resetSplash();
    launchBtn.disabled = false;
    host.querySelectorAll('[data-pose]').forEach((x) => x.classList.remove('on'));
    syncModeHint();
    last = performance.now();
    loop();
  }

  function flash(text: string): void {
    const f = $('flash') as HTMLDivElement;
    f.textContent = text;
    f.classList.add('show');
    setTimeout(() => f.classList.remove('show'), 1600);
  }

  function loop(): void {
    const now = performance.now();
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    session.update(dt);
    const s = session.state;
    const snap = session.currentFrame();
    renderer.render(app.athlete, snap, session.score?.splashSize ?? 0.5);

    $('h-phase').textContent = s.phase === 'contact' ? '蹬台' : s.phase === 'flight' ? '腾空' : '入水';
    $('h-som').textContent = (Math.abs(s.somersault) / (Math.PI * 2)).toFixed(2);
    $('h-tw').textContent = (Math.abs(s.twist) / (Math.PI * 2)).toFixed(2);
    $('h-height').textContent = s.comPos[1].toFixed(1);
    if (snap) {
      $('h-izz').textContent = snap.izz.toFixed(2);
      $('h-hmag').textContent = snap.hMag.toFixed(1);
      $('h-hmag').title = `Hyaw=${snap.hyaw.toFixed(1)}`;
    }
    if (session.finished) {
      finishSession(session);
      const ang = session.entryAngleDeg;
      flash(`入水角 ${ang.toFixed(1)}° · ${ang <= 10 ? '压水合格' : '偏差过大'}`);
      setTimeout(() => navigate('analysis'), 1400);
      return;
    }
    raf = requestAnimationFrame(loop);
  }
  loop();
}

function slider(id: string, label: string, min: number, max: number, step: number, val: number, hint: string): string {
  return `
    <label class="field">
      <span class="lbl">${label}<b>${val.toFixed(2)}</b></span>
      <input type="range" id="${id}" min="${min}" max="${max}" step="${step}" value="${val}">
      <span class="muted" style="font-size:11px">${hint}</span>
    </label>`;
}

// 简单轨道相机：拖拽旋转、滚轮缩放
function attachCamera(canvas: HTMLCanvasElement, r: DiveRenderer): void {
  let dragging = false;
  let lx = 0; let ly = 0;
  canvas.addEventListener('pointerdown', (e) => { dragging = true; lx = e.clientX; ly = e.clientY; });
  window.addEventListener('pointerup', () => { dragging = false; });
  window.addEventListener('pointermove', (e) => {
    if (!dragging) return;
    r.cameraAzimuth -= (e.clientX - lx) * 0.008;
    r.cameraElevation = Math.max(-0.3, Math.min(1.1, r.cameraElevation + (e.clientY - ly) * 0.006));
    lx = e.clientX; ly = e.clientY;
  });
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    r.cameraDistance = Math.max(3.5, Math.min(16, r.cameraDistance + e.deltaY * 0.01));
  }, { passive: false });
}
