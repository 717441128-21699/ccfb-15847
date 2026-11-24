// 应用主控：页面路由 + 全局状态（运动员/动作/会话/回放）
import { ATHLETES, Athlete } from './physics/bodyModel';
import { DIVE_TABLE, DiveDef, findDive } from './physics/diveTable';
import { DiveSession } from './game/session';
import { renderSelectPage } from './ui/selectPage';
import { renderPlatformPage } from './ui/platformPage';
import { renderAnalysisPage } from './ui/analysisPage';
import { renderJudgePage } from './ui/judgePage';

export type PageName = 'select' | 'platform' | 'analysis' | 'judge';

export interface AppState {
  page: PageName;
  athlete: Athlete;
  def: DiveDef;
  session: DiveSession | null;
  lastResult: DiveSession | null;
  autoPilot: boolean;
}

export const app: AppState = {
  page: 'select',
  athlete: ATHLETES[0],
  def: DIVE_TABLE[0],
  session: null,
  lastResult: null,
  autoPilot: true,
};

const TITLES: Record<PageName, string> = {
  select: '① 选动作',
  platform: '② 跳台',
  analysis: '③ 慢放分析',
  judge: '④ 裁判打分',
};

export function navigate(page: PageName): void {
  app.page = page;
  render();
}

export function newSession(autoPilot = app.autoPilot): DiveSession {
  const s = new DiveSession(app.athlete, app.def, autoPilot);
  app.session = s;
  return s;
}

export function finishSession(s: DiveSession): void {
  app.lastResult = s;
  app.session = null;
}

const root = document.getElementById('app')!;

export function render(): void {
  root.innerHTML = `
    <header class="topbar">
      <h1>十米台 · 刚体跳水</h1>
      <nav class="nav">
        ${(Object.keys(TITLES) as PageName[]).map((p) => `
          <button data-nav="${p}" class="${app.page === p ? 'active' : ''}">${TITLES[p]}</button>`).join('')}
      </nav>
      <div class="spacer"></div>
      <div class="meta">
        <span class="tag">${app.athlete.label}</span>
        <span class="tag" style="margin-left:6px">${app.def.code} · DD ${app.def.dd}</span>
      </div>
    </header>
    <main class="page" id="page-root"></main>`;
  root.querySelectorAll<HTMLButtonElement>('[data-nav]').forEach((b) => {
    b.onclick = () => navigate(b.dataset.nav as PageName);
  });
  const host = document.getElementById('page-root')!;
  switch (app.page) {
    case 'select': renderSelectPage(host); break;
    case 'platform': renderPlatformPage(host); break;
    case 'analysis': renderAnalysisPage(host); break;
    case 'judge': renderJudgePage(host); break;
  }
}

export function setAthlete(id: string): void {
  app.athlete = ATHLETES.find((a) => a.id === id) ?? ATHLETES[0];
}
export function setDive(code: string): void {
  app.def = findDive(code);
}

// 应用入口：渲染首个页面
render();
