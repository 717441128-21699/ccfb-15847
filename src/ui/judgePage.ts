// 页面四：裁判打分——7 裁判分、三维度子分、难度系数换算、水花/入水复盘
import { app, navigate } from '../main';
import { DiveRenderer } from '../game/renderer';

export function renderJudgePage(host: HTMLElement): void {
  const result = app.lastResult;
  if (!result || !result.score) {
    host.innerHTML = `<section class="card"><h2>尚无成绩</h2>
      <button class="btn primary" style="width:auto; padding:10px 22px" id="go">去跳台</button></section>`;
    host.querySelector<HTMLButtonElement>('#go')!.onclick = () => navigate('platform');
    return;
  }
  const sc = result.score;
  const entry = result.state.entry!;
  const sorted = [...sc.judges].sort((a, b) => a - b);
  const dropped = new Set([sorted[0], sorted[sorted.length - 1]]);

  host.innerHTML = `
    <div class="grid two" style="flex:1; min-height:0">
      <section class="card" style="padding:10px; display:flex">
        <div class="canvas-wrap" style="flex:1"><canvas class="gl" id="gl-canvas"></canvas></div>
      </section>
      <section class="card">
        <h2>${app.def.code} · ${app.def.name}</h2>
        <div class="score-hero">
          <div class="total">${sc.total.toFixed(1)}</div>
          <div class="sub">最终得分 = 中间 5 裁判均值 ${sc.execution} × 难度 ${sc.dd} × 3</div>
        </div>
        <div class="judges">
          ${sc.judges.map((j) => `<span class="${dropped.has(j) ? 'dropped' : ''}" title="${dropped.has(j) ? '去掉的最高/最低分' : '计入'}">${j.toFixed(1)}</span>`).join('')}
        </div>

        <h3>三维度执行分</h3>
        ${metricBar('入水（角度/手位/水花）', sc.entry)}
        ${metricBar('空中姿态到位', sc.form)}
        ${metricBar('打开时机', sc.timing)}

        <h3>入水物理复盘</h3>
        <table class="kvtable">
          <tr><td>身体与竖直夹角</td><td>${entry.angleDeg.toFixed(1)}° ${entry.angleDeg > 10 ? '（超 10°，重罚）' : ''}</td></tr>
          <tr><td>手掌先破水</td><td>${entry.handsFirst ? '是' : '否（平拍/脚先）'}</td></tr>
          <tr><td>水花指数</td><td>${sc.splashSize.toFixed(2)} / 1.00</td></tr>
          <tr><td>入水瞬间速率</td><td>${Math.hypot(...entry.comVel).toFixed(1)} m/s</td></tr>
        </table>

        <h3>裁判评语</h3>
        ${sc.notes.length ? `<ul class="notes">${sc.notes.map((n) => `<li>${n}</li>`).join('')}</ul>`
          : '<ul class="notes ok"><li>动作完成质量高，无明显扣分点。</li></ul>'}

        <div class="row" style="margin-top:16px">
          <button class="btn ghost" id="replay">回看慢放</button>
          <button class="btn primary" id="again">再跳一次</button>
        </div>
      </section>
    </div>`;

  const canvas = host.querySelector<HTMLCanvasElement>('#gl-canvas')!;
  const renderer = new DiveRenderer(canvas);
  renderer.cameraTarget = [0, 4, 0];
  // 入水瞬间帧 + 持续水花
  const lastFrame = result.snapshots[result.snapshots.length - 1];
  renderer.resetSplash();
  let raf = 0;
  const loop = () => {
    renderer.render(app.athlete, lastFrame, sc.splashSize);
    raf = requestAnimationFrame(loop);
  };
  loop();
  host.addEventListener('remove', () => cancelAnimationFrame(raf), { once: true });

  host.querySelector<HTMLButtonElement>('#replay')!.onclick = () => navigate('analysis');
  host.querySelector<HTMLButtonElement>('#again')!.onclick = () => navigate('platform');
}

function metricBar(label: string, value: number): string {
  const pct = (value / 10) * 100;
  return `
    <div class="metric">
      <span>${label}</span><span class="v">${value.toFixed(1)}</span>
    </div>
    <div class="bar"><i style="width:${pct}%"></i></div>`;
}
