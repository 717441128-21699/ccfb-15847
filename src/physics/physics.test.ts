import { describe, expect, it } from 'vitest';
import { ATHLETES, POSE_PIKE, POSE_STRAIGHT, POSE_TUCK, Pose } from './bodyModel';
import { DIVE_TABLE, findDive } from './diveTable';
import { initialState, solveOmega, step, GRAVITY } from './dynamics';
import {
  angularMomentum, buildMassMatrix, forwardKinematics, internalVelocities,
} from './kinematics';
import { runPlannedDive } from './divePlanner';
import { splashModel } from './scoring';
import { quatToMat, vlen } from './math';

const DT = 1 / 960;
const ZERO_RATE: Pose = { hip: 0, knee: 0, elbow: 0, shoulder: 0, headTuck: 0, twistArms: 0 };
const POSE_KEYS = ['hip', 'knee', 'elbow', 'shoulder', 'headTuck', 'twistArms'] as const;

// 固定姿态、固定 H_z 腾空 1 秒，返回累计翻转角与平均翻转惯量
function fixedPoseSpin(athleteIdx: number, pose: Pose, hz = 8) {
  const athlete = ATHLETES[athleteIdx];
  const s = initialState(athlete);
  s.phase = 'flight';
  s.pose = { ...pose };
  s.poseTarget = { ...pose };
  s.Hworld = [0, 0, hz];
  let angle = 0;
  let invSum = 0;
  const steps = 960;
  for (let i = 0; i < steps; i++) {
    const mm = buildMassMatrix(athlete, s.rootQ, s.pose);
    const w = solveOmega(mm, s.rootQ, s.Hworld, ZERO_RATE);
    angle += w[2] * DT;
    invSum += mm.Iroot[8]; // Izz（体坐标）
    step(s, DT);
  }
  return { angle, avgIzz: invSum / steps, state: s };
}

describe('需求1：三姿态翻转周数比与转动惯量比一致（≤5%）', () => {
  for (let a = 0; a < ATHLETES.length; a++) {
    it(`运动员「${ATHLETES[a].label}」团身/屈体/直体比值`, () => {
      const tuck = fixedPoseSpin(a, POSE_TUCK);
      const pike = fixedPoseSpin(a, POSE_PIKE);
      const straight = fixedPoseSpin(a, POSE_STRAIGHT);

      // H 相同：ω ∝ 1/I，惯量给出的理论比值
      const invI = [1 / tuck.avgIzz, 1 / pike.avgIzz, 1 / straight.avgIzz];
      const normI = invI.map((x) => x / invI[0]);
      // 仿真翻转角比值
      const sim = [tuck.angle, pike.angle, straight.angle];
      const normSim = sim.map((x) => x / sim[0]);

      for (let k = 0; k < 3; k++) {
        const err = Math.abs(normSim[k] - normI[k]) / normI[k];
        expect(
          err,
          `姿态${k} 仿真比 ${normSim[k].toFixed(4)} vs 惯量比 ${normI[k].toFixed(4)}`,
        ).toBeLessThan(0.05);
      }
      // 物理合理性：团身最快、直体最慢
      expect(tuck.angle).toBeGreaterThan(pike.angle);
      expect(pike.angle).toBeGreaterThan(straight.angle);
    });
  }
});

describe('需求1：空中总角动量守恒（独立重算交叉验证）', () => {
  it('团身→打开全过程，由节段速度合成的 H 与守恒量偏差 < 2e-4', () => {
    const athlete = ATHLETES[1];
    const s = initialState(athlete);
    s.phase = 'flight';
    s.Hworld = [0, 0, 10];
    s.pose = { ...POSE_TUCK };
    s.poseTarget = { ...POSE_TUCK };
    const samples: number[] = [];
    for (let i = 0; i < 1440; i++) {
      if (i === 384) s.poseTarget = { ...POSE_STRAIGHT }; // 0.4s 后打开
      step(s, DT);
      if (i % 96 === 0) {
        const kin = forwardKinematics(athlete, s.rootPos, s.rootQ, s.pose);
        const qdot: Pose = { ...ZERO_RATE };
        for (const k of POSE_KEYS) {
          qdot[k] = Math.max(
            -s.poseRateLimit[k],
            Math.min(s.poseRateLimit[k], (s.poseTarget[k] - s.pose[k]) / DT),
          );
        }
        const mm = buildMassMatrix(athlete, s.rootQ, s.pose);
        const wb = solveOmega(mm, s.rootQ, s.Hworld, qdot);
        const R = quatToMat(s.rootQ);
        const ww: [number, number, number] = [0, 0, 0];
        for (let r = 0; r < 3; r++)
          for (let c = 0; c < 3; c++) ww[r] += R[r * 3 + c] * wb[c];
        const vel = internalVelocities(athlete, s.rootPos, s.rootQ, s.pose, qdot, 1e-3);
        for (let j = 0; j < 10; j++) {
          const d: [number, number, number] = [
            kin.frames[j].r[0] - kin.com[0],
            kin.frames[j].r[1] - kin.com[1],
            kin.frames[j].r[2] - kin.com[2],
          ];
          vel.v[j][0] += ww[1] * d[2] - ww[2] * d[1];
          vel.v[j][1] += ww[2] * d[0] - ww[0] * d[2];
          vel.v[j][2] += ww[0] * d[1] - ww[1] * d[0];
          vel.omega[j][0] += ww[0];
          vel.omega[j][1] += ww[1];
          vel.omega[j][2] += ww[2];
        }
        const H = angularMomentum(athlete, kin, vel);
        samples.push(vlen(H));
      }
    }
    const h0 = samples[0];
    expect(h0).toBeGreaterThan(0);
    for (const h of samples) expect(Math.abs(h - h0) / h0).toBeLessThan(2e-4);
  });

  it('线动量守恒：腾空中水平速度恒定、竖直速度严格按重力变化', () => {
    const athlete = ATHLETES[2];
    const s = initialState(athlete);
    s.phase = 'flight';
    s.Hworld = [0, 0, 6];
    s.comVel = [0.4, 4.0, 0];
    s.pose = { ...POSE_PIKE };
    s.poseTarget = { ...POSE_PIKE };
    let maxVelErr = 0;
    let maxPosErr = 0;
    const t0 = s.t;
    const p0 = [...s.comPos] as [number, number, number];
    const v0 = [...s.comVel] as [number, number, number];
    for (let i = 0; i < 700; i++) {
      step(s, DT);
      const t = s.t - t0;
      // 半隐式欧拉：v_n = v0 + n·g·dt（机器精度）
      maxVelErr = Math.max(
        maxVelErr,
        Math.abs(s.comVel[0] - v0[0]),
        Math.abs(s.comVel[1] - (v0[1] - GRAVITY * t)),
      );
      // 对应解析位置：x_n = x0 + n·v0·dt + 0.5·g·dt²·n(n+1)
      const n = i + 1;
      const expX = p0[0] + n * v0[0] * DT;
      const expY = p0[1] + n * v0[1] * DT + 0.5 * -GRAVITY * DT * DT * n * (n + 1);
      maxPosErr = Math.max(maxPosErr, Math.abs(s.comPos[0] - expX), Math.abs(s.comPos[1] - expY));
    }
    expect(s.phase).toBe('flight');
    expect(maxVelErr).toBeLessThan(1e-12);
    expect(maxPosErr).toBeLessThan(1e-12);
  });
});

describe('转体偏航角动量（专用自由度）守恒', () => {
  it('腾空中 Hyaw 恒定；纵轴惯量团身>屈体>直体（与翻转惯量趋势相反，符合真实跳水）', () => {
    const athlete = ATHLETES[1];
    const s = initialState(athlete);
    s.phase = 'flight';
    s.Hworld = [0, 0, 8];
    s.Hyaw = 12;
    s.pose = { ...POSE_TUCK };
    s.poseTarget = { ...POSE_TUCK };
    for (let i = 0; i < 480; i++) {
      if (i === 120) s.poseTarget = { ...POSE_STRAIGHT };
      step(s, DT);
    }
    // 专用转体角动量在无外力矩下严格守恒
    expect(s.Hyaw).toBeCloseTo(12, 10);
    // 纵轴惯量：折叠姿态四肢偏离纵轴，两种折叠姿态均大于直体；
    // 屈体双腿伸直横折、脚离轴最远，故 Iyy 屈体 > 团身 > 直体
    const iTuck = buildMassMatrix(athlete, [0, 0, 0, 1], POSE_TUCK).Iroot[4];
    const iPike = buildMassMatrix(athlete, [0, 0, 0, 1], POSE_PIKE).Iroot[4];
    const iStraight = buildMassMatrix(athlete, [0, 0, 0, 1], POSE_STRAIGHT).Iroot[4];
    expect(iPike).toBeGreaterThan(iTuck);
    expect(iTuck).toBeGreaterThan(iStraight);
    // 因此同 Hyaw 下直体转体速率最高
    expect(12 / iStraight).toBeGreaterThan(12 / iTuck);
  });
});

describe('需求2：动作表可完成性（规划器跑完且周数正确）', () => {
  for (const def of DIVE_TABLE) {
    it(`${def.code} ${def.name}（DD ${def.dd}）`, () => {
      const { state } = runPlannedDive(ATHLETES[0], def);
      expect(state.phase).toBe('entered');
      const somTurns = Math.abs(state.somersault) / (Math.PI * 2);
      const twTurns = Math.abs(state.twist) / (Math.PI * 2);
      expect(somTurns, `翻转 ${somTurns.toFixed(2)} 周`).toBeGreaterThan(
        def.halfSomersaults / 2 - 0.35,
      );
      if (def.halfTwists > 0) {
        expect(twTurns, `转体 ${twTurns.toFixed(2)} 周`).toBeGreaterThan(
          def.halfTwists / 2 - 0.4,
        );
      }
      expect(state.flightTime).toBeGreaterThan(1.0);
      expect(state.flightTime).toBeLessThan(2.8);
    });
  }

  it('207C 与 109B 都能入水且周数达标', () => {
    for (const code of ['207C', '109B']) {
      const def = findDive(code);
      const { state } = runPlannedDive(ATHLETES[0], def);
      const turns = Math.abs(state.somersault) / (Math.PI * 2);
      expect(turns).toBeGreaterThan(def.halfSomersaults / 2 - 0.35);
      expect(state.entry).not.toBeNull();
    }
  });

  it('规划动作垂直入水：入水角（与竖直最小夹角）应小于 20°', () => {
    for (const code of ['207C', '109B', '5152B']) {
      const { state } = runPlannedDive(ATHLETES[0], findDive(code));
      expect(state.entry!.angleDeg, code).toBeLessThan(20);
    }
  });
});

describe('需求2：入水角 >10° 时水花评分明显下降', () => {
  const mk = (angleDeg: number, handsFirst = true) =>
    splashModel(
      {
        t: 1.5, angleDeg, handsFirst, somersaultResidualDeg: 0, twistResidualDeg: 0,
        comVel: [0, -14, 0],
      },
      0.5,
    );
  it('垂直 vs 15° vs 30° 水花单调显著增大', () => {
    const s0 = mk(2);
    const s15 = mk(15);
    const s30 = mk(30);
    expect(s15).toBeGreaterThan(s0 * 1.8);
    expect(s30).toBeGreaterThan(s15 * 1.25);
  });
  it('非手先入水额外惩罚', () => {
    expect(mk(5, false)).toBeGreaterThan(mk(5, true) + 0.2);
  });
});

describe('身材参数与转体耦合', () => {
  it('身高越高直体翻转惯量越大', () => {
    const iSmall = buildMassMatrix(ATHLETES[2], [0, 0, 0, 1], POSE_STRAIGHT).Iroot[8];
    const iTall = buildMassMatrix(ATHLETES[0], [0, 0, 0, 1], POSE_STRAIGHT).Iroot[8];
    expect(iTall).toBeGreaterThan(iSmall);
  });
  it('偏轴手臂产生 I_xz 惯量积（转体耦合）', () => {
    const sym = buildMassMatrix(ATHLETES[0], [0, 0, 0, 1], { ...POSE_PIKE, twistArms: 0 }).Iroot;
    const asym = buildMassMatrix(ATHLETES[0], [0, 0, 0, 1], { ...POSE_PIKE, twistArms: 0.9 }).Iroot;
    expect(Math.abs(asym[2] - sym[2])).toBeGreaterThan(1e-4);
  });
});
