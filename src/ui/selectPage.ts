// 页面一：选动作（运动员身材 + 难度表动作）
import { ATHLETES } from '../physics/bodyModel';
import { DIVE_TABLE, GROUP_LABEL, POSITION_LABEL } from '../physics/diveTable';
import { app, navigate, newSession, setAthlete, setDive } from '../main';

export function renderSelectPage(host: HTMLElement): void {
  host.innerHTML = `
    <div class="grid two" style="flex:1; min-height:0">
      <section class="card">
        <h2>运动员身材（虚构参数）</h2>
        <div class="athlete-pick">
          ${ATHLETES.map((a) => `
            <button class="btn ${a.id === app.athlete.id ? 'sel' : ''}" data-athlete="${a.id}">
              ${a.label}<br>
              <span class="muted">${a.segments.length} 节刚体 · 总质量 ${a.mass}kg</span>
            </button>`).join('')}
        </div>
        <h3>身材对力学的影响</h3>
        <p class="muted">身高越高、体重越大，绕质心转动惯量越大，同样角动量下翻转越慢；
        但蹬台可获得的绝对冲量也更大。规划器会按各自惯量重新标定起跳角动量。</p>
      </section>
      <section class="card">
        <h2>选择动作（FINA 风格代码）</h2>
        <p class="muted" style="margin-top:0">
          首位=组别（${Object.entries(GROUP_LABEL).map(([k, v]) => `${k}${v}`).join('/')}），
          数字=翻转半周数，末位=姿态（${Object.entries(POSITION_LABEL).map(([k, v]) => `${k}${v}`).join('/')}）；
          5 字头为带转体动作。
        </p>
        <div class="dive-list">
          ${DIVE_TABLE.map((d) => `
            <button class="dive-item ${d.code === app.def.code ? 'sel' : ''}" data-dive="${d.code}">
              <div class="code">${d.code}</div>
              <div class="name">${d.name}</div>
              <div class="dd">难度系数 <b>${d.dd.toFixed(1)}</b> ·
                ${d.halfSomersaults / 2} 周翻转${d.halfTwists ? ` · ${d.halfTwists / 2} 周转体` : ''}</div>
            </button>`).join('')}
        </div>
        <div style="margin-top:16px">
          <button class="btn primary" id="go-platform">进入跳台 →</button>
        </div>
      </section>
    </div>`;

  host.querySelectorAll<HTMLButtonElement>('[data-athlete]').forEach((b) => {
    b.onclick = () => { setAthlete(b.dataset.athlete!); renderSelectPage(host); };
  });
  host.querySelectorAll<HTMLButtonElement>('[data-dive]').forEach((b) => {
    b.onclick = () => { setDive(b.dataset.dive!); renderSelectPage(host); };
  });
  host.querySelector<HTMLButtonElement>('#go-platform')!.onclick = () => {
    newSession(app.autoPilot);
    navigate('platform');
  };
}
