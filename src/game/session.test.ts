import { describe, expect, it } from 'vitest';
import { ATHLETES, POSE_TUCK } from '../physics/bodyModel';
import { findDive } from '../physics/diveTable';
import { DEFAULT_TAKEOFF, DiveSession } from './session';

function runToEnd(auto: boolean, code = '207C'): DiveSession {
  const s = new DiveSession(ATHLETES[0], findDive(code), auto);
  s.launch({ ...DEFAULT_TAKEOFF });
  let guard = 0;
  while (!s.finished && guard < 10000) {
    s.update(1 / 60);
    if (!auto && s.state.phase === 'flight') {
      // 手动模式：先团身，后段打开
      const remaining = findDive(code).halfSomersaults / 2 - s.state.somersault / (Math.PI * 2);
      if (remaining > 0.8) s.setManualPose('tuck');
      else s.setManualPose('straight');
    }
    guard++;
  }
  return s;
}

describe('会话层端到端', () => {
  it('自动示范 207C 能跑完并产出评分与快照', () => {
    const s = runToEnd(true);
    expect(s.finished).toBe(true);
    expect(s.score).not.toBeNull();
    expect(s.snapshots.length).toBeGreaterThan(50);
    expect(s.snapshots.at(-1)?.phase).toBe('entered');
    // 快照频率 120Hz、总时长约 2.3s
    expect(s.snapshots.length).toBeLessThan(400);
    expect(s.score!.total).toBeGreaterThan(0);
    expect(s.score!.judges).toHaveLength(7);
  });

  it('手动操控同样能完成一跳（姿态由玩家驱动）', () => {
    const s = runToEnd(false, '103B');
    expect(s.finished).toBe(true);
    expect(s.score).not.toBeNull();
  });

  it('蹬台力度缩放改变腾空高度', () => {
    const weak = new DiveSession(ATHLETES[0], findDive('101B'), true);
    weak.launch({ push: 0.6, lean: 1, twist: 1 });
    const strong = new DiveSession(ATHLETES[0], findDive('101B'), true);
    strong.launch({ push: 1.4, lean: 1, twist: 1 });
    let peakW = 0; let peakS = 0;
    while (!weak.finished) { weak.update(1 / 60); peakW = Math.max(peakW, weak.state.comPos[1]); }
    while (!strong.finished) { strong.update(1 / 60); peakS = Math.max(peakS, strong.state.comPos[1]); }
    expect(peakS).toBeGreaterThan(peakW + 1.5);
  });

  it('前倾为 0 时几乎不翻转', () => {
    const s = new DiveSession(ATHLETES[0], findDive('101B'), true);
    s.launch({ push: 1, lean: 0, twist: 0 });
    while (!s.finished) s.update(1 / 60);
    expect(Math.abs(s.state.somersault) / (Math.PI * 2)).toBeLessThan(0.15);
  });

  it('起跳前会话冻结在台面，不自行推进', () => {
    const s = new DiveSession(ATHLETES[0], findDive('101B'), true);
    for (let i = 0; i < 120; i++) s.update(1 / 60);
    expect(s.isLaunched).toBe(false);
    expect(s.state.phase).toBe('contact');
    expect(s.state.t).toBe(0);
    expect(s.snapshots.length).toBe(0);
    s.launch({ ...DEFAULT_TAKEOFF });
    expect(s.isLaunched).toBe(true);
  });

  it('快照包含分析页所需全部字段', () => {
    const s = runToEnd(true, '5253B');
    const f = s.snapshots[Math.floor(s.snapshots.length / 2)];
    for (const k of ['t', 'rootPos', 'rootQ', 'pose', 'com', 'comVel', 'somTurns',
      'twTurns', 'hMag', 'hyaw', 'izz', 'iyy', 'phase'] as const) {
      expect(f[k], `字段 ${k}`).toBeDefined();
    }
    expect(f.twTurns).toBeGreaterThan(0);
  });

  it('团身姿态 Izz 小于直体（抱紧转得快的根源）', () => {
    const s = runToEnd(true);
    void POSE_TUCK;
    const mid = s.snapshots[Math.floor(s.snapshots.length / 2)];
    expect(mid.izz).toBeGreaterThan(0);
  });

  it('入水角恶化时水花分与总分显著下降（>10° 罚分门槛）', () => {
    const good = runToEnd(true, '103B');
    // 失误跳：翻转不足，打开过早导致平拍入水
    const bad = new DiveSession(ATHLETES[0], findDive('103B'), false);
    bad.launch({ push: 1, lean: 0.45, twist: 0 });
    let guard = 0;
    while (!bad.finished && guard < 10000) {
      bad.update(1 / 60);
      if (bad.state.phase === 'flight') bad.setManualPose('straight'); // 全程直体 → 翻转慢
      guard++;
    }
    expect(bad.finished).toBe(true);
    expect(bad.entryAngleDeg).toBeGreaterThan(good.entryAngleDeg);
    expect(bad.score!.splashSize).toBeGreaterThan(good.score!.splashSize);
    expect(bad.score!.entry).toBeLessThan(good.score!.entry - 2);
    expect(bad.score!.total).toBeLessThan(good.score!.total * 0.7);
  });
});
