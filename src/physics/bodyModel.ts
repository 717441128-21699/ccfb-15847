// 人体多刚体模型：头、躯干、左右上臂、左右前臂、左右大腿、左右小腿（共 10 节）
// 所有几何长度单位 m，质量 kg，惯量 kg·m²。坐标系：x 向前（跳台外侧），y 向上，z 侧向。
import { Vec3, v, vadd, vscale } from './math';

export interface RigidSegment {
  name: string;
  mass: number;
  // 主轴惯量（体坐标，细长体：纵向 Ix，横向 Iy=Iz）
  iLong: number;
  iTrans: number;
  length: number;
  radius: number; // 渲染用
}

export interface Athlete {
  id: string;
  label: string;
  height: number;
  mass: number;
  segments: RigidSegment[];
}

// 用 Hanavan 风格的比例模型：节段长度按身高比例、质量按总体重比例，
// 每节近似为带半径的圆杆（I_long = 1/2 m r²，I_trans = 1/12 m(3r²+L²)）。
function rodInertias(m: number, len: number, r: number) {
  return {
    iLong: 0.5 * m * r * r,
    iTrans: (1 / 12) * m * (3 * r * r + len * len),
  };
}

// 节段占身高比例 / 占体重比例 / 等效半径（m 或占身高比例）
const SEG_RATIOS: { name: string; len: number; mass: number; radius: number }[] = [
  { name: 'head', len: 0.13, mass: 0.081, radius: 0.053 },
  { name: 'torso', len: 0.30, mass: 0.497, radius: 0.095 },
  { name: 'upperArmL', len: 0.173, mass: 0.027, radius: 0.032 },
  { name: 'upperArmR', len: 0.173, mass: 0.027, radius: 0.032 },
  { name: 'forearmL', len: 0.157, mass: 0.016, radius: 0.026 },
  { name: 'forearmR', len: 0.157, mass: 0.016, radius: 0.026 },
  { name: 'thighL', len: 0.232, mass: 0.105, radius: 0.058 },
  { name: 'thighR', len: 0.232, mass: 0.105, radius: 0.058 },
  { name: 'shinL', len: 0.246, mass: 0.047, radius: 0.038 },
  { name: 'shinR', len: 0.246, mass: 0.047, radius: 0.038 },
];
// 质量比例合计 0.968，剩余 ~3.2% 均摊（手脚等），这里归一化处理。

export function buildAthlete(id: string, label: string, height: number, mass: number): Athlete {
  const total = SEG_RATIOS.reduce((s, r) => s + r.mass, 0);
  const segments: RigidSegment[] = SEG_RATIOS.map((r) => {
    const len = r.len * height;
    const m = (r.mass / total) * mass;
    const rad = r.radius * height * 1.9; // 半径比例基于半身高标定
    const { iLong, iTrans } = rodInertias(m, len, rad);
    return { name: r.name, mass: m, length: len, radius: rad, iLong, iTrans };
  });
  return { id, label, height, mass, segments };
}

// 三种身材：男子力量型、女子灵巧型、少年轻盈型（参数为虚构的游戏角色）
export const ATHLETES: Athlete[] = [
  buildAthlete('power', '力量型 · 陈涛 1.78m/72kg', 1.78, 72),
  buildAthlete('agile', '灵巧型 · 林溪 1.64m/52kg', 1.64, 52),
  buildAthlete('youth', '少年型 · 周跃 1.52m/42kg', 1.52, 42),
];

export const SEG_INDEX: Record<string, number> = Object.fromEntries(
  SEG_RATIOS.map((s, i) => [s.name, i]),
);

// ---- 姿态自由度 ----------------------------------------------------------
// 关节角（rad），全部在人体根坐标系（躯干朝向）下描述：
//  hip:        大腿相对躯干的屈角（0 = 直体贴躯干，正值向前折）
//  knee:       小腿相对大腿的屈角（0 = 伸直，正值后勾）
//  elbow:      前臂相对上臂的屈角（0 = 伸直，正值屈臂收拢）
//  shoulder:   上臂相对躯干（0 = 过头伸直贴耳，正值向身体方向收拢）
//  headTuck:   头部下颔角（团身时低头）
//  twistArms:  双臂不对称量（-1..1，单侧多收 → 绕纵轴转体）
export interface Pose {
  hip: number;
  knee: number;
  elbow: number;
  shoulder: number;
  headTuck: number;
  twistArms: number;
}

export const POSE_STRAIGHT: Pose = {
  hip: 0, knee: 0, elbow: 0, shoulder: 0, headTuck: 0, twistArms: 0,
};
export const POSE_PIKE: Pose = {
  hip: 1.9, knee: 0.05, elbow: 0.25, shoulder: 0.35, headTuck: 0.25, twistArms: 0,
};
export const POSE_TUCK: Pose = {
  hip: 2.35, knee: 2.3, elbow: 1.5, shoulder: 0.9, headTuck: 0.5, twistArms: 0,
};

// 人体拓扑（根 = 躯干）。parent 为 -1 表示根。
// 每个关节给出：父节段、父节段上的锚点（体坐标，沿局部 -y 方向为下）、
// 子节段锚点（局部 +y 顶端）、该关节受哪些姿态角驱动。
export interface JointDef {
  parent: number;
  parentAnchor: Vec3;
  childAnchor: Vec3;
}

export function jointLayout(athlete: Athlete): JointDef[] {
  const s = athlete.segments;
  const L = (name: string) => s[SEG_INDEX[name]].length;
  const half = (name: string) => L(name) / 2;
  const tor = half('torso');
  // 躯干局部：+y 朝头、-y 朝腿；肩在躯干上部两侧 ±z，髋在底端两侧 ±z
  return [
    { parent: -1, parentAnchor: v(0, 0, 0), childAnchor: v(0, 0, 0) }, // 0 head 占位（下面覆盖）
    { parent: -1, parentAnchor: v(0, 0, 0), childAnchor: v(0, 0, 0) }, // 1 torso 根
    { parent: 1, parentAnchor: v(0, tor * 0.72, 0.11 * athlete.height), childAnchor: v(0, half('upperArmL'), 0) },
    { parent: 1, parentAnchor: v(0, tor * 0.72, -0.11 * athlete.height), childAnchor: v(0, half('upperArmR'), 0) },
    { parent: 2, parentAnchor: v(0, -half('upperArmL'), 0), childAnchor: v(0, half('forearmL'), 0) },
    { parent: 3, parentAnchor: v(0, -half('upperArmR'), 0), childAnchor: v(0, half('forearmR'), 0) },
    { parent: 1, parentAnchor: v(0, -tor, 0.08 * athlete.height), childAnchor: v(0, half('thighL'), 0) },
    { parent: 1, parentAnchor: v(0, -tor, -0.08 * athlete.height), childAnchor: v(0, half('thighR'), 0) },
    { parent: 6, parentAnchor: v(0, -half('thighL'), 0), childAnchor: v(0, half('shinL'), 0) },
    { parent: 7, parentAnchor: v(0, -half('thighR'), 0), childAnchor: v(0, half('shinR'), 0) },
  ];
}

// 头挂在躯干顶端
export function headJoint(athlete: Athlete): JointDef {
  return {
    parent: 1,
    parentAnchor: v(0, athlete.segments[1].length / 2, 0),
    childAnchor: v(0, athlete.segments[0].length / 2, 0),
  };
}

// 每节段的质心在其局部坐标中的位置（杆中心即原点，偏移为 0，保留接口）
export const segmentCom = (_seg: RigidSegment): Vec3 => v(0, 0, 0);

export function totalMass(athlete: Athlete): number {
  return athlete.segments.reduce((s, x) => s + x.mass, 0);
}

// 工具：按质量加权求质心
export function weightedCom(points: { p: Vec3; m: number }[]): Vec3 {
  let m = 0;
  const acc = v();
  for (const e of points) {
    m += e.m;
    acc[0] += e.p[0] * e.m;
    acc[1] += e.p[1] * e.m;
    acc[2] += e.p[2] * e.m;
  }
  return vscale(acc, 1 / m);
}

export function appendLocal(a: Vec3, b: Vec3): Vec3 {
  return vadd(a, b);
}
