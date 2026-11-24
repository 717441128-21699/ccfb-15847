// 裁判评分：7 名裁判取中间 5 个（去掉最高最低），三维度子分 → 综合执行分，
// 再按 FINA 风格公式：总分 = 执行分均值 × 难度系数 × 3。
import { DiveDef } from './diveTable';
import { EntryData } from './dynamics';
import { clamp, rad2deg, wrapAngle } from './math';

export interface ScoreBreakdown {
  entry: number; // 入水（水花/角度/手位）0-10
  form: number; // 空中姿态到位 0-10
  timing: number; // 打开时机 0-10
  judges: number[]; // 7 名裁判综合分（含少量人为偏差）
  execution: number; // 中间 5 个的均值
  dd: number;
  total: number; // 最终得分（0-100 量级）
  splashSize: number; // 0..1，越大水花越大
  notes: string[];
}

// 水花物理近似：入水角越偏、残留角速度越大、非手先入水 → 水花越大
export function splashModel(entry: EntryData, omegaAtEntry: number): number {
  const anglePen = (entry.angleDeg / 10) ** 2 * 0.16; // 10° 处明显下降，30° 仍未封顶
  const spinPen = clamp(Math.abs(omegaAtEntry) / 6, 0, 1) * 0.2;
  const handPen = entry.handsFirst ? 0 : 0.24;
  return clamp(0.05 + anglePen + spinPen + handPen, 0, 1);
}

// 确定性伪随机（裁判风格偏差，不依赖 Math.random 保证回放一致）
function judgeBias(seed: number): number {
  const x = Math.sin(seed * 127.1 + 311.7) * 43758.5453;
  return (x - Math.floor(x) - 0.5) * 0.7;
}

export interface ScoreInput {
  def: DiveDef;
  entry: EntryData;
  omegaAtEntry: number;
  // 空中姿态贴合度 0..1（主姿态保持时间占比，由回放统计）
  formQuality: number;
  // 打开时机误差 s（实际打开时刻 vs 理想时刻）
  openTimingError: number;
  // 实际完成的翻转/转体周数（由会话统计，用于计算相对目标的残差）
  actualSomersaults: number;
  actualTwists: number;
}

export function scoreDive(input: ScoreInput): ScoreBreakdown {
  const { def, entry } = input;
  const notes: string[] = [];

  const splash = splashModel(entry, input.omegaAtEntry);

  // --- 入水维度 ---
  let entryScore = 10;
  if (entry.angleDeg > 10) {
    entryScore -= clamp((entry.angleDeg - 10) * 0.45, 0, 5.5);
    notes.push(`入水角偏离垂直 ${entry.angleDeg.toFixed(1)}°，超过 10° 罚分`);
  } else if (entry.angleDeg > 4) {
    entryScore -= (entry.angleDeg - 4) * 0.25;
  }
  entryScore -= splash * 2.6;
  if (!entry.handsFirst) {
    entryScore -= 1.2;
    notes.push('非手掌先入水，水花增大');
  }
  // 翻转残差：实际周数与目标周数之差换算到 ±180° 内（整数周对齐）
  const targetSom = input.def.halfSomersaults / 2;
  const residualSom = Math.abs(rad2deg(wrapAngle((input.actualSomersaults - targetSom) * Math.PI * 2)));
  if (residualSom > 12) {
    entryScore -= clamp((residualSom - 12) * 0.08, 0, 2);
    notes.push(`翻转零头 ${residualSom.toFixed(0)}°，入水方位不理想`);
  }
  entryScore = clamp(entryScore, 0, 10);

  // --- 姿态维度 ---
  const form = clamp(4 + input.formQuality * 6, 0, 10);
  if (input.formQuality < 0.75) notes.push('空中姿态保持不够稳定');

  // --- 打开时机维度 ---
  const timing = clamp(10 - Math.abs(input.openTimingError) * 9, 0, 10);
  if (Math.abs(input.openTimingError) > 0.12) notes.push('打开时机偏晚/偏早');

  // 裁判团：三维度加权 + 个人风格偏差
  const base = entryScore * 0.5 + form * 0.28 + timing * 0.22;
  const judges = Array.from({ length: 7 }, (_, i) =>
    clamp(Math.round((base + judgeBias(i + def.halfSomersaults)) * 2) / 2, 0, 10),
  );
  const sorted = [...judges].sort((a, b) => a - b);
  const middle = sorted.slice(1, 6);
  const execution = middle.reduce((a, b) => a + b, 0) / 5;
  const total = Math.round(execution * def.dd * 3 * 10) / 10;

  return {
    entry: Math.round(entryScore * 10) / 10,
    form: Math.round(form * 10) / 10,
    timing: Math.round(timing * 10) / 10,
    judges,
    execution: Math.round(execution * 100) / 100,
    dd: def.dd,
    total,
    splashSize: splash,
    notes,
  };
}
