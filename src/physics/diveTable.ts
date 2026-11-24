// 动作难度表（10 米台常见动作，难度系数参照 FINA 规则量级，用于游戏内评分）
// 代码规则：首位=组别(1前/2后/3反/4内/5转体)，随后数字=半周数，末位=姿态(A直/B屈/C团/D自由)
export type Position = 'A' | 'B' | 'C' | 'D';

export interface DiveDef {
  code: string;
  group: 1 | 2 | 3 | 4 | 5;
  halfSomersaults: number; // 翻转半周数（3 = 1.5 周）
  halfTwists: number; // 转体半周数
  position: Position;
  dd: number; // 难度系数
  name: string;
}

export const POSITION_LABEL: Record<Position, string> = {
  A: '直体',
  B: '屈体',
  C: '团身',
  D: '自由',
};

export const GROUP_LABEL: Record<number, string> = {
  1: '向前',
  2: '向后',
  3: '反身',
  4: '向内',
  5: '转体',
};

export const DIVE_TABLE: DiveDef[] = [
  { code: '101B', group: 1, halfSomersaults: 1, halfTwists: 0, position: 'B', dd: 1.5, name: '向前跳水 屈体' },
  { code: '201B', group: 2, halfSomersaults: 1, halfTwists: 0, position: 'B', dd: 1.8, name: '向后跳水 屈体' },
  { code: '301B', group: 3, halfSomersaults: 1, halfTwists: 0, position: 'B', dd: 1.9, name: '反身跳水 屈体' },
  { code: '401B', group: 4, halfSomersaults: 1, halfTwists: 0, position: 'B', dd: 1.5, name: '向内跳水 屈体' },
  { code: '103B', group: 1, halfSomersaults: 3, halfTwists: 0, position: 'B', dd: 1.6, name: '向前一周半 屈体' },
  { code: '203C', group: 2, halfSomersaults: 3, halfTwists: 0, position: 'C', dd: 1.9, name: '向后一周半 团身' },
  { code: '303C', group: 3, halfSomersaults: 3, halfTwists: 0, position: 'C', dd: 2.0, name: '反身一周半 团身' },
  { code: '405C', group: 4, halfSomersaults: 5, halfTwists: 0, position: 'C', dd: 2.7, name: '向内两周半 团身' },
  { code: '205B', group: 2, halfSomersaults: 5, halfTwists: 0, position: 'B', dd: 2.9, name: '向后两周半 屈体' },
  { code: '207C', group: 2, halfSomersaults: 7, halfTwists: 0, position: 'C', dd: 3.3, name: '向后三周半 团身' },
  { code: '109C', group: 1, halfSomersaults: 9, halfTwists: 0, position: 'C', dd: 3.7, name: '向前四周半 团身' },
  { code: '109B', group: 1, halfSomersaults: 9, halfTwists: 0, position: 'B', dd: 3.8, name: '向前四周半 屈体' },
  { code: '5253B', group: 5, halfSomersaults: 5, halfTwists: 3, position: 'B', dd: 3.2, name: '向后一周半转体一周半 屈体' },
  { code: '5152B', group: 5, halfSomersaults: 5, halfTwists: 2, position: 'B', dd: 2.9, name: '向前一周半转体一周 屈体' },
];

export function findDive(code: string): DiveDef {
  const d = DIVE_TABLE.find((x) => x.code === code);
  if (!d) throw new Error(`未知动作 ${code}`);
  return d;
}

// 组别的翻转方向：向前/反身向前翻（负 z 角速度），向后/向内后翻（正 z）
export function somersaultSign(group: number): 1 | -1 {
  return group === 1 || group === 3 ? -1 : 1;
}
