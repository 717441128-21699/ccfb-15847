// 动作规划器：代数估值 + 仿真内回路标定
// 翻转：网格搜索 H_z 与打开阈值；转体：按累计转体闭环（转够整数周即回正手臂）
import { Athlete, POSE_PIKE, POSE_STRAIGHT, POSE_TUCK, Pose, totalMass } from './bodyModel';
import { ContactPlan, DiveState, GRAVITY, initialState, setContact, step } from './dynamics';
import { DiveDef, somersaultSign } from './diveTable';
import { Vec3, clamp, mat3InvSym, mat3Vec, v } from './math';
import { buildMassMatrix } from './kinematics';

export interface PlannedDive {
  contact: ContactPlan;
  targetSomersaults: number;
  targetTwists: number;
  openThreshold: number; // 剩余翻转多少周时打开
  flightTimeEst: number;
  twistArmLevel: number;
}

const DT_FINE = 1 / 480;
const DT_COARSE = 1 / 240;
const TWIST_ARM = 0.9;

export function mainPose(def: DiveDef): Pose {
  if (def.position === 'A') return { ...POSE_STRAIGHT };
  if (def.position === 'B') return { ...POSE_PIKE };
  return { ...POSE_TUCK };
}

// 姿态冻结时 ω_body = Iroot⁻¹·H_body，单位 H 的翻转角速度（代数标定）
function spinRatePerHz(athlete: Athlete, pose: Pose): number {
  const s = initialState(athlete);
  const mm = buildMassMatrix(athlete, s.rootQ, pose);
  const hUnit: Vec3 = [0, 0, 1];
  return mat3Vec(mat3InvSym(mm.Iroot), hUnit)[2];
}

interface TrialParams {
  Hz: number;
  Hyaw: number;
  openThreshold: number;
  v0: number;
}

// 转体速率闭环：根据当前累计转体与剩余翻转时间，连续调节偏轴手臂量（0..0.9）
function twistArmLevelFor(s: DiveState, def: DiveDef, openThreshold: number): number {
  const targetTw = def.halfTwists / 2;
  if (targetTw <= 0) return 0;
  const twDone = Math.abs(s.twist) / (Math.PI * 2);
  if (twDone >= targetTw - 0.02) return 0; // 转够即回正
  const targetSom = def.halfSomersaults / 2;
  const remainingSom = targetSom - Math.abs(s.somersault) / (Math.PI * 2);
  if (remainingSom <= openThreshold + 0.05) return 0; // 进入打开阶段前回正
  // 期望的转体速率（周/s）：剩余转体 / 可用时间（用剩余翻转周数粗略估时）
  const spinRateTurns = Math.max(0.4, Math.abs(s.somersault) / (Math.PI * 2) / Math.max(0.05, s.flightTime));
  const timeLeft = Math.max(0.15, remainingSom / spinRateTurns);
  const desiredRate = (targetTw - twDone) / timeLeft * (Math.PI * 2); // rad/s
  // 当前姿态下偏轴手臂能给出的速率范围（Hyaw / Iyy）
  const mm = buildMassMatrix(s.athlete, s.rootQ, s.pose);
  const Iyy = mm.Iroot[4];
  const currentRate = s.Hyaw / Iyy;
  // 比例调节：当前慢于期望 → 加大偏轴（Iyy 变小、速率升），反之回正
  const err = desiredRate - currentRate;
  const cmd = s.pose.twistArms + clamp(err * 0.25, -0.12, 0.12);
  return clamp(cmd, 0, TWIST_ARM);
}

interface TrialResult {
  state: DiveState;
  somTurns: number;
  twTurns: number;
  entryAngle: number;
  handsFirst: boolean;
  flightTime: number;
  cost: number;
}

// 闭环姿态控制（试跳与正式回放共用同一逻辑）
function controlPose(s: DiveState, def: DiveDef, openThreshold: number): void {
  if (s.phase === 'contact') {
    s.poseTarget = { ...mainPose(def), twistArms: 0 };
    return;
  }
  if (s.phase !== 'flight') return;
  const targetSom = def.halfSomersaults / 2;
  const remainingSom = targetSom - Math.abs(s.somersault) / (Math.PI * 2);
  const base = mainPose(def);
  if (remainingSom <= openThreshold) {
    s.poseTarget = { ...POSE_STRAIGHT }; // 打开拉直，准备垂直入水
    return;
  }
  // 转体速率闭环：偏轴手臂连续调节（Iyy 小幅变化 → 速率微调）
  s.poseTarget = { ...base, twistArms: twistArmLevelFor(s, def, openThreshold) };
}

function trial(
  athlete: Athlete,
  def: DiveDef,
  p: TrialParams,
  calibDt = DT_COARSE,
): TrialResult {
  const M = totalMass(athlete);
  const verticalImpulse = M * (p.v0 + GRAVITY * 0.32);
  const horizontalSpeed = def.group === 1 || def.group === 3 ? 0.55 : 0.35;
  const contact: ContactPlan = {
    duration: 0.32,
    impulse: [
      (def.group === 2 || def.group === 4 ? -1 : 1) * M * horizontalSpeed,
      verticalImpulse,
      0,
    ],
    angularImpulseBody: v(0, 0, p.Hz),
    yawAngularImpulse: p.Hyaw,
  };
  const s = initialState(athlete);
  setContact(s, contact);
  while (s.phase !== 'entered' && s.t < 6) {
    controlPose(s, def, p.openThreshold);
    step(s, calibDt);
  }
  const somTurns = Math.abs(s.somersault) / (Math.PI * 2);
  const twTurns = Math.abs(s.twist) / (Math.PI * 2);
  const entryAngle = s.entry?.angleDeg ?? 90;
  const handsFirst = s.entry?.handsFirst ?? false;
  const targetSom = def.halfSomersaults / 2;
  const targetTw = def.halfTwists / 2;
  const residualSom = Math.abs(somTurns - targetSom);
  const residualTw = targetTw > 0 ? Math.abs(twTurns - targetTw) : 0;
  const cost =
    residualSom * 200 +
    entryAngle * 1.2 +
    (handsFirst ? 0 : 30) +
    residualTw * 180;
  return {
    state: s, somTurns, twTurns, entryAngle, handsFirst,
    flightTime: s.flightTime, cost,
  };
}

export function planDive(athlete: Athlete, def: DiveDef): PlannedDive {
  const sign = somersaultSign(def.group);
  const targetSom = def.halfSomersaults / 2;
  const targetTw = def.halfTwists / 2;
  const pose = mainPose(def);

  const v0 = 3.6 + Math.min(1.1, targetSom * 0.16);
  const fallH = 10 + athlete.height * 0.62;
  const flightTimeEst = (v0 + Math.sqrt(v0 * v0 + 2 * GRAVITY * fallH)) / GRAVITY;

  // H_z 代数初值
  const spinPerHz = Math.abs(spinRatePerHz(athlete, pose));
  const spinTime = Math.max(0.5, flightTimeEst - 0.38);
  const Hz0 = (sign * (targetSom * Math.PI * 2) / spinTime) / spinPerHz;

  // H_yaw 代数初值：转体角动量 = Iyy(偏轴姿态) × 所需轴向角速度
  let Hyaw0 = 0;
  if (targetTw > 0) {
    const twistPose = { ...pose, twistArms: TWIST_ARM };
    const mmTw = buildMassMatrix(athlete, [0, 0, 0, 1], twistPose);
    const IyyTwist = mmTw.Iroot[4];
    const needYawRate = (targetTw * Math.PI * 2) / (spinTime * 0.85);
    Hyaw0 = IyyTwist * needYawRate;
  }

  // 翻转网格：H_z 因子 × 打开阈值
  const openGrid = [0.35, 0.55, 0.75, 0.95, 1.15, 1.35];
  const hzFactors = [0.6, 0.72, 0.84, 0.92, 1.0, 1.08, 1.2, 1.35];
  let best: TrialResult | null = null;
  let bestParams: TrialParams = { Hz: Hz0, Hyaw: Hyaw0, openThreshold: 0.75, v0 };
  for (const f of hzFactors) {
    for (const ot of openGrid) {
      const r = trial(athlete, def, { Hz: Hz0 * f, Hyaw: Hyaw0, openThreshold: ot, v0 });
      if (!best || r.cost < best.cost) {
        best = r;
        bestParams = { Hz: Hz0 * f, Hyaw: Hyaw0, openThreshold: ot, v0 };
      }
    }
  }

  // 转体精修：速率闭环对 Hyaw 不敏感，只需粗调量级，再联合精修翻转参数
  if (targetTw > 0 && best) {
    for (const fy of [0.5, 0.75, 1.0, 1.3]) {
      const r = trial(athlete, def, { ...bestParams, Hyaw: Hyaw0 * fy });
      if (r.cost < best.cost) {
        best = r;
        bestParams = { ...bestParams, Hyaw: Hyaw0 * fy };
      }
    }
    for (const f of hzFactors) {
      for (const ot of openGrid) {
        const r = trial(athlete, def, { ...bestParams, Hz: Hz0 * f, openThreshold: ot });
        if (r.cost < best.cost) {
          best = r;
          bestParams = { ...bestParams, Hz: Hz0 * f, openThreshold: ot };
        }
      }
    }
  }

  const M = totalMass(athlete);
  const horizontalSpeed = def.group === 1 || def.group === 3 ? 0.55 : 0.35;
  const contact: ContactPlan = {
    duration: 0.32,
    impulse: [
      (def.group === 2 || def.group === 4 ? -1 : 1) * M * horizontalSpeed,
      M * (v0 + GRAVITY * 0.32),
      0,
    ],
    angularImpulseBody: v(0, 0, bestParams.Hz),
    yawAngularImpulse: bestParams.Hyaw,
  };

  return {
    contact,
    targetSomersaults: targetSom,
    targetTwists: targetTw,
    openThreshold: bestParams.openThreshold,
    flightTimeEst,
    twistArmLevel: targetTw > 0 ? TWIST_ARM : 0,
  };
}

// 实时/回放控制
export function applyDiveControls(s: DiveState, plan: PlannedDive, def: DiveDef): void {
  controlPose(s, def, plan.openThreshold);
}

export function runPlannedDive(
  athlete: Athlete,
  def: DiveDef,
): { state: DiveState; plan: PlannedDive } {
  const plan = getPlan(athlete, def);
  const s = initialState(athlete);
  setContact(s, plan.contact);
  while (s.phase !== 'entered' && s.t < 6) {
    applyDiveControls(s, plan, def);
    step(s, DT_FINE);
  }
  return { state: s, plan };
}

const planCache = new Map<string, PlannedDive>();
export function getPlan(athlete: Athlete, def: DiveDef): PlannedDive {
  const key = `${athlete.id}:${def.code}`;
  let plan = planCache.get(key);
  if (!plan) {
    plan = planDive(athlete, def);
    planCache.set(key, plan);
  }
  return plan;
}

export function clearPlanCache(): void {
  planCache.clear();
}
