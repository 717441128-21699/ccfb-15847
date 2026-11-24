// 正向运动学与质量矩阵组装
// 根节点 = 躯干；姿态角驱动各关节在矢状面内旋转（绕体坐标 z），
// twistArms 驱动单臂侧展（绕 x），用惯量积把翻惯量耦合为转体。
import {
  Athlete, Pose, SEG_INDEX, jointLayout, headJoint, totalMass,
} from './bodyModel';
import {
  Mat3, Quat, Vec3, mat3Mul, mat3Scale, mat3T, mat3Vec, quatAxisAngle, quatMul,
  quatRotate, quatToMat, skew, v, vadd, vscale, vsub, vcross,
} from './math';

export interface SegmentFrame {
  r: Vec3; // 节段质心（世界）
  q: Quat; // 节段朝向（世界）
  R: Mat3; // 对应旋转矩阵
}

export interface Kinematics {
  frames: SegmentFrame[]; // 长度 10
  com: Vec3; // 全身质心（世界）
  comOffsetBody: Vec3; // 质心相对根（躯干）的体坐标偏移
}

const Z_AXIS: Vec3 = [0, 0, 1];
const X_AXIS: Vec3 = [1, 0, 0];
const rz = (a: number): Quat => quatAxisAngle(Z_AXIS, a);
const rx = (a: number): Quat => quatAxisAngle(X_AXIS, a);

// 各节段相对父节段的关节四元数（由姿态角决定）
export function jointQuats(pose: Pose): Quat[] {
  const out: Quat[] = new Array(10);
  out[1] = [0, 0, 0, 1]; // 躯干 = 根
  out[0] = rz(-pose.headTuck); // 头：低头
  // 双臂：肩屈（过头 → 向前收拢）；左臂额外侧展产生转体耦合
  const twistRad = pose.twistArms * 0.85;
  out[2] = quatMul(rx(twistRad), rz(-pose.shoulder));
  out[3] = rz(-pose.shoulder);
  out[4] = rz(pose.elbow); // 左前臂
  out[5] = rz(pose.elbow); // 右前臂
  out[6] = rz(pose.hip); // 左大腿
  out[7] = rz(pose.hip); // 右大腿
  out[8] = rz(-pose.knee); // 左小腿
  out[9] = rz(-pose.knee); // 右小腿
  return out;
}

export const PARENT = [1, -1, 1, 1, 2, 3, 1, 1, 6, 7];

export function forwardKinematics(
  athlete: Athlete,
  rootPos: Vec3,
  rootQ: Quat,
  pose: Pose,
): Kinematics {
  const layout = jointLayout(athlete);
  layout[0] = headJoint(athlete);
  const rel = jointQuats(pose);
  const frames: SegmentFrame[] = new Array(10);

  frames[1] = { r: [rootPos[0], rootPos[1], rootPos[2]], q: rootQ, R: quatToMat(rootQ) };
  // 动态拓扑排序，保证父先于子
  const order: number[] = [];
  const visited = new Array(10).fill(false);
  visited[1] = true;
  const visit = (i: number): void => {
    if (visited[i]) return;
    if (PARENT[i] !== -1) visit(PARENT[i]);
    visited[i] = true;
    order.push(i);
  };
  for (let i = 0; i < 10; i++) visit(i);
  for (const i of order) {
    const p = PARENT[i];
    const q = quatMul(frames[p].q, rel[i]);
    const R = quatToMat(q);
    const jd = layout[i];
    const anchorP = mat3Vec(frames[p].R, jd.parentAnchor);
    const anchorC = mat3Vec(R, jd.childAnchor);
    const r = vsub(vadd(frames[p].r, anchorP), anchorC);
    frames[i] = { r, q, R };
  }

  // 全身质心
  let m = 0;
  const acc = v();
  athlete.segments.forEach((s, i) => {
    m += s.mass;
    acc[0] += frames[i].r[0] * s.mass;
    acc[1] += frames[i].r[1] * s.mass;
    acc[2] += frames[i].r[2] * s.mass;
  });
  const com = vscale(acc, 1 / m);
  const RrootT = mat3T(frames[1].R);
  const comOffsetBody = mat3Vec(RrootT, vsub(com, rootPos));
  return { frames, com, comOffsetBody };
}

// 节段在体坐标下的对角惯量张量（长轴沿局部 y）
export function segmentInertiaBody(athlete: Athlete, i: number): Mat3 {
  const s = athlete.segments[i];
  return [
    s.iTrans, 0, 0,
    0, s.iLong, 0,
    0, 0, s.iTrans,
  ];
}

export interface MassMatrix {
  Iroot: Mat3; // 根角速度 → 绕全身质心角动量（根体坐标下的 3x3）
  A: Mat3; // 6 个姿态自由度 → 内部角动量（3x6，列主展开为长度 18）
  M: number; // 总质量
}

export const POSE_KEYS: (keyof Pose)[] = [
  'hip', 'knee', 'elbow', 'shoulder', 'headTuck', 'twistArms',
];

// 仅由姿态速率产生的各节段速度（根静止），用中心差分 + 解析角速度
export function internalVelocities(
  athlete: Athlete,
  rootPos: Vec3,
  rootQ: Quat,
  pose: Pose,
  poseRate: Pose,
  dt: number,
): { v: Vec3[]; omega: Vec3[] } {
  const fP = POSE_KEYS.map((k) => {
    const p = { ...pose, [k]: pose[k] + poseRate[k] * dt };
    return forwardKinematics(athlete, rootPos, rootQ, p);
  });
  const fM = POSE_KEYS.map((k) => {
    const p = { ...pose, [k]: pose[k] - poseRate[k] * dt };
    return forwardKinematics(athlete, rootPos, rootQ, p);
  });
  const kin = forwardKinematics(athlete, rootPos, rootQ, pose);
  const v: Vec3[] = new Array(10);
  const omega: Vec3[] = new Array(10);

  for (let i = 0; i < 10; i++) {
    let vi: Vec3 = [0, 0, 0];
    for (let j = 0; j < 6; j++) {
      const dv = vscale(vsub(fP[j].frames[i].r, fM[j].frames[i].r), 1 / (2 * dt));
      vi = vadd(vi, dv);
    }
    v[i] = vi;

    // 解析相对角速度，沿父链把各关节轴转到世界系累加
    let ow: Vec3 = [0, 0, 0];
    const chain: number[] = [];
    let cur = i;
    while (cur !== 1 && cur !== -1) {
      chain.push(cur);
      cur = PARENT[cur];
    }
    for (let c = chain.length - 1; c >= 0; c--) {
      const seg = chain[c];
      const pq = PARENT[seg] === 1 ? rootQ : kin.frames[PARENT[seg]].q;
      if (seg === 2) {
        // 左上臂 = Rx(twist*0.85) ⊗ Rz(-shoulder)
        const xAxis = quatRotate(pq, [1, 0, 0]);
        ow = vadd(ow, vscale(xAxis, poseRate.twistArms * 0.85));
        const zAxis = quatRotate(quatMul(pq, rx(pose.twistArms * 0.85)), [0, 0, 1]);
        ow = vadd(ow, vscale(zAxis, -poseRate.shoulder));
      } else {
        const zAxis = quatRotate(pq, [0, 0, 1]);
        ow = vadd(ow, vscale(zAxis, relAngleRate(seg, poseRate)));
      }
    }
    omega[i] = ow;
  }
  return { v, omega };
}

function relAngleRate(i: number, pr: Pose): number {
  switch (i) {
    case 0: return -pr.headTuck;
    case 2:
    case 3: return -pr.shoulder;
    case 4:
    case 5: return pr.elbow;
    case 6:
    case 7: return pr.hip;
    case 8:
    case 9: return -pr.knee;
    default: return 0;
  }
}

// 绕全身质心的角动量（世界系），质心平动速度由线动量守恒自行扣除
export function angularMomentum(
  athlete: Athlete,
  kin: Kinematics,
  vel: { v: Vec3[]; omega: Vec3[] },
): Vec3 {
  let M = 0;
  const Vc: Vec3 = [0, 0, 0];
  for (let i = 0; i < 10; i++) {
    M += athlete.segments[i].mass;
    Vc[0] += vel.v[i][0] * athlete.segments[i].mass;
    Vc[1] += vel.v[i][1] * athlete.segments[i].mass;
    Vc[2] += vel.v[i][2] * athlete.segments[i].mass;
  }
  Vc[0] /= M; Vc[1] /= M; Vc[2] /= M;
  const H: Vec3 = [0, 0, 0];
  for (let i = 0; i < 10; i++) {
    const s = athlete.segments[i];
    const Ri = kin.frames[i].R;
    const Ib = segmentInertiaBody(athlete, i);
    const Iw = mat3Mul(mat3Mul(Ri, Ib), mat3T(Ri));
    const spin = mat3Vec(Iw, vel.omega[i]);
    const d = vsub(kin.frames[i].r, kin.com);
    const orb = vscale(vcross(d, vsub(vel.v[i], Vc)), s.mass);
    H[0] += spin[0] + orb[0];
    H[1] += spin[1] + orb[1];
    H[2] += spin[2] + orb[2];
  }
  return H;
}

// 动力学量组装：
//  Iroot = 冻结姿态时根单位角速度对应的绕质心角动量（根体坐标）
//  A 的第 j 列 = 第 j 个姿态自由度单位速率（根静止）对应的内部角动量（根体坐标）
// 结果只取决于姿态（与根朝向无关），按姿态量化缓存，规划器网格搜索可复用。
const mmCache = new Map<string, MassMatrix>();

export function buildMassMatrix(
  athlete: Athlete,
  rootQ: Quat,
  pose: Pose,
  fdDt = 1e-3,
): MassMatrix {
  void rootQ;
  const key =
    athlete.id +
    '|' +
    POSE_KEYS.map((k) => Math.round(pose[k] * 1000)).join(',');
  const cached = mmCache.get(key);
  if (cached) return cached;

  const rootPos: Vec3 = [0, 0, 0];
  const kin = forwardKinematics(athlete, rootPos, [0, 0, 0, 1], pose);
  const M = totalMass(athlete);
  const Rroot = kin.frames[1].R;
  const RrootT = mat3T(Rroot);

  const Iworld: Mat3 = new Array(9).fill(0);
  for (let i = 0; i < 10; i++) {
    const s = athlete.segments[i];
    const Ri = kin.frames[i].R;
    const Ib = segmentInertiaBody(athlete, i);
    const Iw = mat3Mul(mat3Mul(Ri, Ib), mat3T(Ri));
    const d = vsub(kin.frames[i].r, kin.com);
    const sk = skew(d);
    const parallel = mat3Scale(mat3Mul(sk, sk), -s.mass); // m(|d|²1 − ddᵀ)
    for (let k = 0; k < 9; k++) Iworld[k] += Iw[k] + parallel[k];
  }
  const IrootBody = mat3Mul(mat3Mul(RrootT, Iworld), Rroot);

  const A = new Array<number>(18).fill(0);
  const identityQ: Quat = [0, 0, 0, 1];
  for (let j = 0; j < 6; j++) {
    const rate: Pose = { hip: 0, knee: 0, elbow: 0, shoulder: 0, headTuck: 0, twistArms: 0 };
    rate[POSE_KEYS[j]] = 1;
    const vel = internalVelocities(athlete, rootPos, identityQ, pose, rate, fdDt);
    const Hw = angularMomentum(athlete, kin, vel);
    const Hb = mat3Vec(RrootT, Hw);
    for (let r = 0; r < 3; r++) A[r * 6 + j] = Hb[r];
  }
  const result = { Iroot: IrootBody, A, M };
  mmCache.set(key, result);
  return result;
}

// 矩阵 × 6 维姿态速率向量（A 按列存储）
export function applyA(A: Mat3, qdot: Pose): Vec3 {
  const x = [qdot.hip, qdot.knee, qdot.elbow, qdot.shoulder, qdot.headTuck, qdot.twistArms];
  const out: Vec3 = [0, 0, 0];
  for (let r = 0; r < 3; r++)
    for (let j = 0; j < 6; j++) out[r] += A[r * 6 + j] * x[j];
  return out;
}

export { SEG_INDEX };
