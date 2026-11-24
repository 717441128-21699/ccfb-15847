// 三维向量 / 3x3 矩阵 / 四元数数学库（行优先存储，无外部依赖，便于单测）
export type Vec3 = [number, number, number];
export type Mat3 = number[]; // 长度 9，行优先：m[row*3+col]
export type Quat = [number, number, number, number]; // x,y,z,w

export const v = (x = 0, y = 0, z = 0): Vec3 => [x, y, z];
export const vadd = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const vsub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const vscale = (a: Vec3, s: number): Vec3 => [a[0] * s, a[1] * s, a[2] * s];
export const vdot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const vcross = (a: Vec3, b: Vec3): Vec3 => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const vlen = (a: Vec3): number => Math.hypot(a[0], a[1], a[2]);
export const vdist = (a: Vec3, b: Vec3): number => vlen(vsub(a, b));
export const vnorm = (a: Vec3): Vec3 => {
  const l = vlen(a);
  return l > 1e-12 ? vscale(a, 1 / l) : [0, 0, 0];
};
export const vlerp = (a: Vec3, b: Vec3, t: number): Vec3 => [
  a[0] + (b[0] - a[0]) * t,
  a[1] + (b[1] - a[1]) * t,
  a[2] + (b[2] - a[2]) * t,
];

export function mat3Identity(): Mat3 {
  return [1, 0, 0, 0, 1, 0, 0, 0, 1];
}
export function mat3Mul(a: Mat3, b: Mat3): Mat3 {
  const r = new Array(9).fill(0);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      for (let k = 0; k < 3; k++) r[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
  return r;
}
export function mat3T(a: Mat3): Mat3 {
  return [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]];
}
export function mat3Vec(m: Mat3, x: Vec3): Vec3 {
  return [
    m[0] * x[0] + m[1] * x[1] + m[2] * x[2],
    m[3] * x[0] + m[4] * x[1] + m[5] * x[2],
    m[6] * x[0] + m[7] * x[1] + m[8] * x[2],
  ];
}
// 反对称矩阵 [a]×：[a]× b = a × b
export function skew(a: Vec3): Mat3 {
  return [0, -a[2], a[1], a[2], 0, -a[0], -a[1], a[0], 0];
}
// 对称 3x3 求逆（惯量张量恒对称正定）
export function mat3InvSym(m: Mat3): Mat3 {
  const det =
    m[0] * (m[4] * m[8] - m[5] * m[7]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6]);
  const inv = [
    m[4] * m[8] - m[5] * m[7],
    m[2] * m[7] - m[1] * m[8],
    m[1] * m[5] - m[2] * m[4],
    m[5] * m[6] - m[3] * m[8],
    m[0] * m[8] - m[2] * m[6],
    m[2] * m[3] - m[0] * m[5],
    m[3] * m[7] - m[4] * m[6],
    m[1] * m[6] - m[0] * m[7],
    m[0] * m[4] - m[1] * m[3],
  ];
  return inv.map((x) => x / det);
}
export function trace3(m: Mat3): number {
  return m[0] + m[4] + m[8];
}
export function mat3Scale(m: Mat3, s: number): Mat3 {
  return m.map((x) => x * s);
}

export function quatIdentity(): Quat {
  return [0, 0, 0, 1];
}
export function quatNormalize(q: Quat): Quat {
  const l = Math.hypot(q[0], q[1], q[2], q[3]) || 1;
  return [q[0] / l, q[1] / l, q[2] / l, q[3] / l];
}
export function quatMul(a: Quat, b: Quat): Quat {
  return [
    a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
    a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
    a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
    a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
  ];
}
// 绕单位轴 axis 转 angle 的四元数
export function quatAxisAngle(axis: Vec3, angle: number): Quat {
  const h = angle * 0.5;
  const s = Math.sin(h);
  return [axis[0] * s, axis[1] * s, axis[2] * s, Math.cos(h)];
}
export function quatRotate(q: Quat, x: Vec3): Vec3 {
  // q x q*
  const u: Vec3 = [q[0], q[1], q[2]];
  const w = q[3];
  const t = vscale(vcross(u, x), 2);
  return vadd(vadd(x, vscale(t, w)), vcross(u, t));
}
// 体坐标系角速度 ω_body 积分：q' = q ⊗ quat(0.5 ω dt)
export function quatIntegrate(q: Quat, omegaBody: Vec3, dt: number): Quat {
  const dq: Quat = [omegaBody[0] * dt * 0.5, omegaBody[1] * dt * 0.5, omegaBody[2] * dt * 0.5, 1];
  return quatNormalize(quatMul(q, dq));
}
export function quatSlerp(a: Quat, b: Quat, t: number): Quat {
  let cos = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  let b2 = b;
  if (cos < 0) {
    b2 = [-b[0], -b[1], -b[2], -b[3]];
    cos = -cos;
  }
  if (cos > 0.9995) {
    return quatNormalize([
      a[0] + (b2[0] - a[0]) * t,
      a[1] + (b2[1] - a[1]) * t,
      a[2] + (b2[2] - a[2]) * t,
      a[3] + (b2[3] - a[3]) * t,
    ]);
  }
  const ang = Math.acos(Math.min(1, cos));
  const s0 = Math.sin(ang * (1 - t)) / Math.sin(ang);
  const s1 = Math.sin(ang * t) / Math.sin(ang);
  return [
    a[0] * s0 + b2[0] * s1,
    a[1] * s0 + b2[1] * s1,
    a[2] * s0 + b2[2] * s1,
    a[3] * s0 + b2[3] * s1,
  ];
}
// 四元数 -> 旋转矩阵
export function quatToMat(q: Quat): Mat3 {
  const [x, y, z, w] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}
// 角度归一到 (-π, π]
export function wrapAngle(a: number): number {
  let r = a % (Math.PI * 2);
  if (r <= -Math.PI) r += Math.PI * 2;
  if (r > Math.PI) r -= Math.PI * 2;
  return r;
}
export const deg2rad = (d: number): number => (d * Math.PI) / 180;
export const rad2deg = (r: number): number => (r * 180) / Math.PI;
export const clamp = (x: number, lo: number, hi: number): number => Math.max(lo, Math.min(hi, x));
export const lerp = (a: number, b: number, t: number): number => a + (b - a) * t;
// 带速率限制的趋近
export function approach(current: number, target: number, maxDelta: number): number {
  if (current < target) return Math.min(target, current + maxDelta);
  return Math.max(target, current - maxDelta);
}
