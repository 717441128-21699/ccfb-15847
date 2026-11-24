// 一跳的完整会话：起跳参数 → 固定步长物理 → 快照录制 → 评分
import { Athlete, Pose, POSE_PIKE, POSE_STRAIGHT, POSE_TUCK } from '../physics/bodyModel';
import {
  ContactPlan, DiveState, effectiveRootQ, initialState, setContact, solveOmega, step,
} from '../physics/dynamics';
import { DiveDef, POSITION_LABEL } from '../physics/diveTable';
import { PlannedDive, applyDiveControls, getPlan, mainPose } from '../physics/divePlanner';
import { ScoreBreakdown, scoreDive } from '../physics/scoring';
import { Quat, Vec3, clamp, rad2deg } from '../physics/math';
import { buildMassMatrix } from '../physics/kinematics';

export const PHYS_DT = 1 / 480;
const RECORD_EVERY = 4; // 120 Hz 快照

export interface FrameSnapshot {
  t: number;
  flightTime: number;
  phase: DiveState['phase'];
  rootPos: Vec3;
  rootQ: Quat; // 含偏航的有效朝向
  pose: Pose;
  com: Vec3;
  comVel: Vec3;
  omegaMag: number;
  somTurns: number;
  twTurns: number;
  hMag: number;
  hyaw: number;
  izz: number;
  iyy: number;
}

export interface TakeoffInputs {
  push: number; // 蹬台力度 0.5..1.5（相对规划基准的冲量缩放）
  lean: number; // 前倾角 -1..1（翻转角动量缩放，负号方向由组别决定）
  twist: number; // 转体意图 -1..1（偏航角动量缩放）
}

export const DEFAULT_TAKEOFF: TakeoffInputs = { push: 1, lean: 1, twist: 1 };

export class DiveSession {
  readonly athlete: Athlete;
  readonly def: DiveDef;
  readonly plan: PlannedDive;
  state: DiveState;
  autoPilot: boolean;
  snapshots: FrameSnapshot[] = [];
  score: ScoreBreakdown | null = null;
  private acc = 0;
  private stepCount = 0;
  private launched = false;
  private openAtActual: number | null = null;
  private formMatchTime = 0;
  private omegaAtEntry = 0;
  private userPose: Partial<Pose> | null = null;

  constructor(athlete: Athlete, def: DiveDef, autoPilot = true) {
    this.athlete = athlete;
    this.def = def;
    this.plan = getPlan(athlete, def);
    this.autoPilot = autoPilot;
    this.state = initialState(athlete);
  }

  // 按玩家起跳输入组装接触计划（在规划基准上缩放）
  launch(inputs: TakeoffInputs): void {
    const base = this.plan.contact;
    const contact: ContactPlan = {
      duration: base.duration,
      impulse: [
        base.impulse[0] * inputs.push,
        base.impulse[1] * inputs.push,
        0,
      ],
      angularImpulseBody: base.angularImpulseBody
        ? [0, 0, base.angularImpulseBody[2] * inputs.lean]
        : undefined,
      yawAngularImpulse: (base.yawAngularImpulse ?? 0) * inputs.twist,
    };
    setContact(this.state, contact);
    this.launched = true;
  }

  // 玩家手动指定姿态（团身/屈体/直体/偏轴手臂）
  setManualPose(kind: 'tuck' | 'pike' | 'straight', twistArms?: number): void {
    const base = kind === 'tuck' ? POSE_TUCK : kind === 'pike' ? POSE_PIKE : POSE_STRAIGHT;
    this.userPose = { ...base, twistArms: twistArms ?? this.state.poseTarget.twistArms };
  }
  setTwistArms(level: number): void {
    this.userPose = { ...this.state.poseTarget, twistArms: clamp(level, -1, 1) };
  }

  update(realDt: number): void {
    if (this.state.phase === 'entered') return;
    if (!this.launched) return; // 起跳前冻结在台面，等玩家确认输入
    this.acc += Math.min(realDt, 0.05);
    while (this.acc >= PHYS_DT) {
      this.acc -= PHYS_DT;
      this.fixedStep();
      if ((this.state.phase as string) === 'entered') break;
    }
  }

  get isLaunched(): boolean {
    return this.launched;
  }

  private fixedStep(): void {
    const s = this.state;
    if (s.phase === 'flight') {
      if (this.autoPilot) {
        applyDiveControls(s, this.plan, this.def);
      } else if (this.userPose) {
        Object.assign(s.poseTarget, this.userPose);
      }
      this.trackFormAndOpen();
    } else if (s.phase === 'contact') {
      s.poseTarget = { ...mainPose(this.def), twistArms: 0 };
    }
    step(s, PHYS_DT);
    this.stepCount++;
    if (s.phase === 'flight' || s.phase === 'entered') {
      this.omegaAtEntry = this.currentOmegaMag();
    }
    if (this.stepCount % RECORD_EVERY === 0 || s.phase === 'entered') this.record();
    if (s.phase === 'entered') this.finalize();
  }

  // 物理步频率的角速度：守恒求解器给出的体坐标 ω + 偏航速率
  private currentOmegaMag(): number {
    const s = this.state;
    const mm = buildMassMatrix(this.athlete, s.rootQ, s.pose);
    // 姿态目标与当前的限幅速率
    const qdot: Pose = { hip: 0, knee: 0, elbow: 0, shoulder: 0, headTuck: 0, twistArms: 0 };
    for (const k of ['hip', 'knee', 'elbow', 'shoulder', 'headTuck', 'twistArms'] as const) {
      qdot[k] = clamp(s.poseTarget[k] - s.pose[k], -s.poseRateLimit[k] * PHYS_DT, s.poseRateLimit[k] * PHYS_DT) / PHYS_DT;
    }
    const wb = solveOmega(mm, s.rootQ, s.Hworld, qdot);
    const yawRate = s.Hyaw / mm.Iroot[4];
    return Math.hypot(wb[0], wb[1], wb[2], yawRate);
  }

  private trackFormAndOpen(): void {
    const s = this.state;
    const required = mainPose(this.def);
    const close =
      Math.abs(s.pose.hip - required.hip) < 0.35 &&
      Math.abs(s.pose.knee - required.knee) < 0.45;
    const isStraight = s.pose.hip < 0.25 && s.pose.knee < 0.25;
    if (close || isStraight) this.formMatchTime += PHYS_DT;
    if (isStraight && this.openAtActual === null && s.flightTime > 0.15) {
      this.openAtActual = s.flightTime;
    }
  }

  private record(): void {
    const s = this.state;
    const q = effectiveRootQ(s);
    const mm = this.currentInertia();
    this.snapshots.push({
      t: s.t,
      flightTime: s.flightTime,
      phase: s.phase,
      rootPos: [...s.rootPos] as Vec3,
      rootQ: [...q] as Quat,
      pose: { ...s.pose },
      com: [...s.comPos] as Vec3,
      comVel: [...s.comVel] as Vec3,
      omegaMag: this.omegaAtEntry,
      somTurns: Math.abs(s.somersault) / (Math.PI * 2),
      twTurns: Math.abs(s.twist) / (Math.PI * 2),
      hMag: Math.hypot(s.Hworld[0], s.Hworld[1], s.Hworld[2]),
      hyaw: s.Hyaw,
      izz: mm.izz,
      iyy: mm.iyy,
    });
  }

  private currentInertia(): { izz: number; iyy: number } {
    const mm = buildMassMatrix(this.athlete, this.state.rootQ, this.state.pose);
    return { izz: mm.Iroot[8], iyy: mm.Iroot[4] };
  }

  private finalize(): void {
    const s = this.state;
    if (!s.entry) return;
    const flight = Math.max(0.3, s.flightTime);
    const formQuality = clamp(this.formMatchTime / flight, 0, 1);
    const idealOpen = this.plan.flightTimeEst - 0.38;
    const openError = this.openAtActual !== null
      ? this.openAtActual - idealOpen
      : 0.5;
    this.score = scoreDive({
      def: this.def,
      entry: s.entry,
      omegaAtEntry: this.omegaAtEntry,
      formQuality,
      openTimingError: clamp(openError, -0.5, 0.5),
      actualSomersaults: Math.abs(s.somersault) / (Math.PI * 2),
      actualTwists: Math.abs(s.twist) / (Math.PI * 2),
    });
  }

  get finished(): boolean {
    return this.state.phase === 'entered';
  }

  // 供渲染层取当前帧：未起跳时给出静止首帧，起跳后取最新快照
  currentFrame(): FrameSnapshot {
    if (this.snapshots.length === 0) this.record();
    return this.snapshots[this.snapshots.length - 1];
  }
  get positionLabel(): string {
    return POSITION_LABEL[this.def.position];
  }
  get entryAngleDeg(): number {
    return this.state.entry?.angleDeg ?? 0;
  }
  get residualSomDeg(): number {
    return this.state.entry ? rad2deg(this.state.entry.somersaultResidualDeg) : 0;
  }
}
