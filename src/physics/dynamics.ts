// 跳水动力学求解器
// 接触阶段：台面对脚的反力作为外力冲量（含重力），同时积分线动量与绕质心角动量；
// 腾空阶段：无外力矩，H_world 恒定，每步由 H_b = Iroot·ω_b + A·qdot 反解根角速度，
//           因此总角动量在机器精度内守恒（姿态变化只在 Iroot 与内部项之间重新分配）。
import { Athlete, Pose, totalMass } from './bodyModel';
import {
  Mat3, Quat, Vec3, approach, clamp, lerp, mat3InvSym, mat3Mul, mat3T, mat3Vec,
  quatAxisAngle, quatIntegrate, quatMul, quatToMat, v, vadd, vcross, vscale, vsub, wrapAngle,
} from './math';
import {
  Kinematics, MassMatrix, POSE_KEYS, applyA, buildMassMatrix, forwardKinematics,
} from './kinematics';

export type DivePhase = 'contact' | 'flight' | 'entered';

export interface ContactPlan {
  duration: number; // 起跳蹬台时长 s
  impulse: Vec3; // 台面对人体的总冲量 N·s（不含重力）
  angularImpulseBody?: Vec3; // 蹬台期间施加的总角冲量（躯干体坐标，z=翻转）
  yawAngularImpulse?: number; // 绕身体纵轴的转体角冲量 kg·m²/s
}

export interface EntryData {
  t: number;
  angleDeg: number; // 身体纵轴与竖直向下的夹角，0 = 垂直入水
  handsFirst: boolean; // 手是否先于脚触水
  somersaultResidualDeg: number; // 剩余翻转（未转完的零头）
  twistResidualDeg: number; // 剩余转体零头
  comVel: Vec3;
}

export interface DiveState {
  t: number;
  phase: DivePhase;
  athlete: Athlete;
  rootPos: Vec3;
  rootQ: Quat;
  comPos: Vec3;
  comVel: Vec3;
  pose: Pose;
  poseTarget: Pose;
  poseRateLimit: Pose; // 每秒最大变化量
  Hworld: Vec3; // 绕质心角动量（世界系），腾空中恒定
  Hyaw: number; // 绕身体纵轴的专用转体角动量（轴向，独立守恒），kg·m²/s
  yaw: number; // 转体偏航角 rad（绕躯干局部 y，独立于 rootQ 的翻滚旋转）
  contact: ContactPlan | null;
  somersault: number; // 累计翻转角 rad（绕世界 z，解卷绕）
  twist: number; // 累计转体角 rad（= yaw，绕身体纵轴）
  entry: EntryData | null;
  flightTime: number;
}

export const GRAVITY = 9.81;
const PLATFORM_Y = 10.0; // 台高 10 m，水面 y=0

export function defaultRateLimit(): Pose {
  return { hip: 9, knee: 12, elbow: 14, shoulder: 12, headTuck: 8, twistArms: 6 };
}

// 初始状态：运动员直立站于台端，根（躯干中心）高度按质心几何摆放
export function initialState(athlete: Athlete): DiveState {
  const straight: Pose = {
    hip: 0, knee: 0, elbow: 0, shoulder: 0, headTuck: 0, twistArms: 0,
  };
  const rootPos: Vec3 = [0, PLATFORM_Y + athlete.height * 0.62, 0];
  const rootQ: Quat = [0, 0, 0, 1];
  const kin = forwardKinematics(athlete, rootPos, rootQ, straight);
  const M = totalMass(athlete);
  void M;
  return {
    t: 0,
    phase: 'contact',
    athlete,
    rootPos,
    rootQ,
    comPos: kin.com,
    comVel: [0, 0, 0],
    pose: { ...straight },
    poseTarget: { ...straight },
    poseRateLimit: defaultRateLimit(),
    Hworld: [0, 0, 0],
    Hyaw: 0,
    yaw: 0,
    contact: null,
    somersault: 0,
    twist: 0,
    entry: null,
    flightTime: 0,
  };
}

export function setContact(s: DiveState, plan: ContactPlan): void {
  s.contact = plan;
}

// 外力冲量剖面：sin² 形，∫₀¹ profile(u) du = 1，保证总冲量精确等于计划值
function forceProfile(u: number): number {
  if (u < 0 || u > 1) return 0;
  return 2 * Math.sin(Math.PI * u) ** 2;
}

function stepPose(pose: Pose, target: Pose, limit: Pose, dt: number): Pose {
  const out = { ...pose };
  for (const k of POSE_KEYS) {
    out[k] = approach(pose[k], target[k], limit[k] * dt);
  }
  return out;
}

// 由当前 H_world 与姿态速率反解根角速度（体坐标）
export function solveOmega(
  mm: MassMatrix,
  rootQ: Quat,
  Hworld: Vec3,
  qdot: Pose,
): Vec3 {
  const R = quatToMat(rootQ);
  const Hb = mat3Vec(mat3T(R), Hworld);
  const internal = applyA(mm.A, qdot);
  const rhs: Vec3 = [Hb[0] - internal[0], Hb[1] - internal[1], Hb[2] - internal[2]];
  return mat3Vec(mat3InvSym(mm.Iroot), rhs);
}

// 姿态的瞬时速率（朝目标逼近的实际速率），用于内部角动量项
function currentPoseRate(s: DiveState, dt: number): Pose {
  const r = { hip: 0, knee: 0, elbow: 0, shoulder: 0, headTuck: 0, twistArms: 0 } as Pose;
  for (const k of POSE_KEYS) {
    const diff = s.poseTarget[k] - s.pose[k];
    const max = s.poseRateLimit[k] * dt;
    r[k] = clamp(diff, -max, max) / dt;
  }
  return r;
}

// 单步积分（dt 建议 ≤ 1/480）
export function step(s: DiveState, dt: number): void {
  if (s.phase === 'entered') return;
  s.t += dt;

  // ---- 1. 姿态速率（限幅） ----
  const qdot = currentPoseRate(s, dt);
  const poseNew = stepPose(s.pose, s.poseTarget, s.poseRateLimit, dt);

  // ---- 2. 线动量 / 角动量外力项 ----
  const M = totalMass(s.athlete);
  const F: Vec3 = [0, -GRAVITY * M, 0];
  let tau: Vec3 = [0, 0, 0];
  if (s.phase === 'contact' && s.contact) {
    const c = s.contact;
    const u = s.t / c.duration;
    const prof = forceProfile(clamp(u, 0, 1)) / c.duration;
    const Fpush = vscale(c.impulse, prof);
    // 线性冲量等效作用于质心（力度 → 腾空速度/高度）；
    // 全部转速由前倾角/摆臂的角冲量 angularImpulseBody 单独决定，映射清晰。
    F[0] += Fpush[0]; F[1] += Fpush[1]; F[2] += Fpush[2];
    if (c.angularImpulseBody) {
      const tauBody = vscale(c.angularImpulseBody, prof);
      tau = mat3Vec(quatToMat(s.rootQ), tauBody);
    }
    if (c.yawAngularImpulse) {
      s.Hyaw += c.yawAngularImpulse * prof * dt;
    }
    if (u >= 1) {
      s.phase = 'flight';
      s.flightTime = 0;
      s.contact = null;
    }
  }
  s.comVel = vadd(s.comVel, vscale(F, dt / M));
  s.Hworld = vadd(s.Hworld, vscale(tau, dt));

  // ---- 3. 由 H 反解 ω，积分根朝向与姿态 ----
  const mm = buildMassMatrix(s.athlete, s.rootQ, poseNew);
  const omegaBody = solveOmega(mm, s.rootQ, s.Hworld, qdot);
  s.rootQ = quatIntegrate(s.rootQ, omegaBody, dt);
  s.pose = poseNew;

  // ---- 3b. 专用转体偏航：Hyaw = Iyy(yaw轴) · yawRate，腾空中 Hyaw 守恒 ----
  // 偏轴手臂使 Iyy 变化 → 转体速率随之变化（抱得越紧转得越快，回正即变慢）。
  const Iyy = mm.Iroot[4];
  const yawRate = s.Hyaw / Iyy;
  s.yaw += yawRate * dt;
  s.twist = s.yaw;

  // ---- 4. 质心平动 & 根位置反解 ----
  s.comPos = vadd(s.comPos, vscale(s.comVel, dt));
  if (s.phase === 'flight') s.flightTime += dt;
  const kin = forwardKinematics(s.athlete, [0, 0, 0], s.rootQ, s.pose);
  s.rootPos = vsub(s.comPos, mat3Vec(quatToMat(s.rootQ), kin.comOffsetBody));

  // ---- 5. 翻转/转体累计 ----
  trackRotation(s, omegaBody, dt);

  // ---- 6. 入水检测（用含偏航的有效朝向）----
  const kin2 = forwardKinematics(s.athlete, s.rootPos, effectiveRootQ(s), s.pose);
  if (lowestPoint(kin2) <= 0) {
    enterWater(s, kin2, omegaBody);
  }
}

// 含转体偏航的有效根朝向：rootQ（翻滚）⊗ Ry(yaw)
export function effectiveRootQ(s: DiveState): Quat {
  if (Math.abs(s.yaw) < 1e-12) return s.rootQ;
  return quatMul(s.rootQ, quatAxisAngle([0, 1, 0], s.yaw));
}

function trackRotation(s: DiveState, omegaBody: Vec3, dt: number): void {
  void dt;
  void omegaBody;
  // 翻转：躯干纵轴（体坐标 y）在世界 x-y 平面内的方位角
  const R = quatToMat(s.rootQ);
  const axisY: Vec3 = [R[1], R[4], R[7]];
  const ang = Math.atan2(axisY[0], axisY[1]);
  const last = (s as unknown as { _lastSom?: number })._lastSom ?? ang;
  s.somersault += wrapAngle(ang - last);
  (s as unknown as { _lastSom?: number })._lastSom = ang;
  // 转体由专用偏航状态 yaw 驱动（见 step 3b），此处不重复累计
}

function footPoint(s: DiveState): Vec3 {
  const kin = forwardKinematics(s.athlete, s.rootPos, s.rootQ, s.pose);
  // 小腿（8/9）质心再沿局部 -y 推半个节段长
  const pts = [8, 9].map((i) => {
    const seg = s.athlete.segments[i];
    const down = mat3Vec(kin.frames[i].R, [0, -seg.length / 2, 0]);
    return vadd(kin.frames[i].r, down);
  });
  return pts[0][1] < pts[1][1] ? pts[0] : pts[1];
}

function lowestPoint(kin: Kinematics): number {
  let lo = Infinity;
  for (const f of kin.frames) lo = Math.min(lo, f.r[1]);
  return lo;
}

function handPoint(s: DiveState, kin: Kinematics): Vec3 {
  const pts = [4, 5].map((i) => {
    const seg = s.athlete.segments[i];
    const down = mat3Vec(kin.frames[i].R, [0, -seg.length / 2, 0]);
    return vadd(kin.frames[i].r, down);
  });
  return pts[0][1] < pts[1][1] ? pts[0] : pts[1];
}

function enterWater(s: DiveState, kin: Kinematics, omegaBody: Vec3): void {
  const R = kin.frames[1].R;
  // 入水角 = 身体纵轴与竖直方向的最小夹角（0 = 垂直，无论头朝上还是朝下的轴表示）。
  // 纵轴朝头，垂直入水时头朝下 → 与向下方向平行，用绝对值把 180° 折回 0°。
  const bodyAxis: Vec3 = [R[1], R[4], R[7]];
  const down: Vec3 = [0, -1, 0];
  const cosA = clamp(Math.abs(bodyAxis[0] * down[0] + bodyAxis[1] * down[1] + bodyAxis[2] * down[2]), -1, 1);
  const angleDeg = (Math.acos(cosA) * 180) / Math.PI;
  const hand = handPoint(s, kin);
  const foot = footPoint(s);
  s.entry = {
    t: s.t,
    angleDeg,
    handsFirst: hand[1] <= foot[1] + 0.05,
    somersaultResidualDeg: (wrapAngle(s.somersault) * 180) / Math.PI,
    twistResidualDeg: (wrapAngle(s.twist) * 180) / Math.PI,
    comVel: [...s.comVel] as Vec3,
  };
  s.phase = 'entered';
  void omegaBody;
}

// 固定步长跑到入水或超时
export function simulate(
  s: DiveState,
  controls: (t: number, s: DiveState) => void,
  dt = 1 / 480,
  maxT = 6,
): DiveState {
  while (s.phase !== 'entered' && s.t < maxT) {
    controls(s.t, s);
    step(s, dt);
  }
  return s;
}

// 姿态目标便捷设置
export function setPoseTarget(s: DiveState, p: Partial<Pose>): void {
  Object.assign(s.poseTarget, p);
}

export { lerp, mat3Mul };
