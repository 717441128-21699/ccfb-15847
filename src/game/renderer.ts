// WebGL2 渲染器：胶囊体 + 球拼装 10 节人体，跳台/水面/粒子水花，无第三方依赖
import { Athlete } from '../physics/bodyModel';
import { forwardKinematics } from '../physics/kinematics';
import { FrameSnapshot } from './session';
import { Mat3, Vec3, vadd } from '../physics/math';

const VERT = `#version 300 es
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
uniform mat4 uViewProj;
uniform mat4 uModel;
uniform vec3 uColor;
uniform float uShade;
out vec3 vColor;
out vec3 vNormal;
void main() {
  vec4 world = uModel * vec4(aPos, 1.0);
  gl_Position = uViewProj * world;
  vColor = uColor * uShade;
  vNormal = mat3(uModel) * aNormal;
}`;

const FRAG = `#version 300 es
precision highp float;
in vec3 vColor;
in vec3 vNormal;
uniform vec3 uLightDir;
out vec4 frag;
void main() {
  float diff = max(dot(normalize(vNormal), normalize(uLightDir)), 0.0);
  float amb = 0.45;
  frag = vec4(vColor * (amb + 0.75 * diff), 1.0);
}`;

// ---------- 小型列主 4x4 矩阵 ----------
type Mat4 = Float32Array;
function m4Identity(): Mat4 {
  return new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
}
function m4Mul(a: Mat4, b: Mat4): Mat4 {
  const o = new Float32Array(16);
  for (let c = 0; c < 4; c++)
    for (let r = 0; r < 4; r++)
      o[c * 4 + r] =
        a[0 * 4 + r] * b[c * 4 + 0] +
        a[1 * 4 + r] * b[c * 4 + 1] +
        a[2 * 4 + r] * b[c * 4 + 2] +
        a[3 * 4 + r] * b[c * 4 + 3];
  return o;
}
function m4Translation(x: number, y: number, z: number): Mat4 {
  const m = m4Identity();
  m[12] = x; m[13] = y; m[14] = z;
  return m;
}
function m4FromMat3(R: Mat3): Mat4 {
  const m = m4Identity();
  m[0] = R[0]; m[1] = R[3]; m[2] = R[6];
  m[4] = R[1]; m[5] = R[4]; m[6] = R[7];
  m[8] = R[2]; m[9] = R[5]; m[10] = R[8];
  return m;
}
function m4Perspective(fovy: number, aspect: number, near: number, far: number): Mat4 {
  const f = 1 / Math.tan(fovy / 2);
  const nf = 1 / (near - far);
  const m = new Float32Array(16);
  m[0] = f / aspect;
  m[5] = f;
  m[10] = (far + near) * nf;
  m[11] = -1;
  m[14] = 2 * far * near * nf;
  return m;
}
function m4LookAt(eye: Vec3, center: Vec3, up: Vec3): Mat4 {
  const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const norm = (a: Vec3): Vec3 => {
    const l = Math.hypot(a[0], a[1], a[2]) || 1;
    return [a[0] / l, a[1] / l, a[2] / l];
  };
  const cross = (a: Vec3, b: Vec3): Vec3 => [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
  const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const z = norm(sub(eye, center));
  const x = norm(cross(up, z));
  const y = cross(z, x);
  const m = m4Identity();
  m[0] = x[0]; m[4] = x[1]; m[8] = x[2]; m[12] = -dot(x, eye);
  m[1] = y[0]; m[5] = y[1]; m[9] = y[2]; m[13] = -dot(y, eye);
  m[2] = z[0]; m[6] = z[1]; m[10] = z[2]; m[14] = -dot(z, eye);
  return m;
}

// ---------- 几何：胶囊（圆柱 + 两半球）与立方体 ----------
// 胶囊局部坐标：轴沿 y，半径 1，圆柱半高 1（总高 4，两端球心在 y=±1）
interface Mesh {
  vao: WebGLVertexArrayObject;
  count: number;
}
function makeCapsuleMesh(gl: WebGL2RenderingContext, seg = 16, hemiRings = 8): Mesh {
  const pos: number[] = [];
  const nor: number[] = [];
  const idx: number[] = [];
  const add = (p: Vec3, n: Vec3) => { pos.push(...p); nor.push(...n); };
  const addRing = (cy: number, rr: number, ny: number) => {
    for (let i = 0; i <= seg; i++) {
      const a = (i / seg) * Math.PI * 2;
      add([rr * Math.cos(a), cy, rr * Math.sin(a)], [Math.cos(a), ny, Math.sin(a)]);
    }
  };
  // 圆柱两端环（法线纯径向）
  addRing(-1, 1, 0);
  addRing(1, 1, 0);
  for (let i = 0; i < seg; i++) {
    const a = i, b = i + 1, c = (seg + 1) + i, d = c + 1;
    idx.push(a, c, b, b, c, d);
  }
  // 半球：j=0 为与圆柱衔接的环，j=hemiRings 为极点
  const hemisphere = (top: boolean) => {
    const base = pos.length / 3;
    for (let j = 0; j <= hemiRings; j++) {
      const phi = (j / hemiRings) * (Math.PI / 2);
      const rr = Math.cos(phi);
      const along = Math.sin(phi); // 沿轴分量
      const y = top ? 1 + along : -1 - along;
      const ny = top ? along : -along;
      for (let i = 0; i <= seg; i++) {
        const a = (i / seg) * Math.PI * 2;
        add([rr * Math.cos(a), y, rr * Math.sin(a)], [rr * Math.cos(a), ny, rr * Math.sin(a)]);
      }
    }
    for (let j = 0; j < hemiRings; j++) {
      for (let i = 0; i < seg; i++) {
        const a = base + j * (seg + 1) + i;
        const b = a + 1, c = a + seg + 1, d = c + 1;
        // 保证朝外的绕序
        if (top) idx.push(a, c, b, b, c, d);
        else idx.push(a, b, c, b, d, c);
      }
    }
  };
  hemisphere(true);
  hemisphere(false);
  return uploadMesh(gl, pos, nor, idx);
}

function makeCubeMesh(gl: WebGL2RenderingContext): Mesh {
  const p = [
    -1, -1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1,
    -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1,
  ];
  const faces = [
    [0, 3, 2, 0, 2, 1], [4, 5, 6, 4, 6, 7], [0, 1, 5, 0, 5, 4],
    [3, 7, 6, 3, 6, 2], [1, 2, 6, 1, 6, 5], [0, 4, 7, 0, 7, 3],
  ];
  const normals = [
    [0, 0, -1], [0, 0, 1], [0, -1, 0], [0, 1, 0], [1, 0, 0], [-1, 0, 0],
  ];
  const pos: number[] = [];
  const nor: number[] = [];
  const idx: number[] = [];
  faces.forEach((f, fi) => f.forEach((vi) => {
    pos.push(p[vi * 3], p[vi * 3 + 1], p[vi * 3 + 2]);
    nor.push(...normals[fi]);
    idx.push(idx.length);
  }));
  return uploadMesh(gl, pos, nor, idx);
}

function uploadMesh(gl: WebGL2RenderingContext, pos: number[], nor: number[], idx: number[]): Mesh {
  const vao = gl.createVertexArray()!;
  gl.bindVertexArray(vao);
  const pb = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, pb);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(pos), gl.STATIC_DRAW);
  gl.enableVertexAttribArray(0);
  gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
  const nb = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, nb);
  gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(nor), gl.STATIC_DRAW);
  gl.enableVertexAttribArray(1);
  gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 0, 0);
  const ib = gl.createBuffer();
  gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, ib);
  gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(idx), gl.STATIC_DRAW);
  return { vao, count: idx.length };
}

const SEGMENT_COLORS: Record<string, Vec3> = {
  head: [0.96, 0.78, 0.62],
  torso: [0.16, 0.42, 0.78],
  upperArmL: [0.92, 0.34, 0.3], upperArmR: [0.92, 0.34, 0.3],
  forearmL: [0.96, 0.78, 0.62], forearmR: [0.96, 0.78, 0.62],
  thighL: [0.2, 0.26, 0.4], thighR: [0.2, 0.26, 0.4],
  shinL: [0.2, 0.26, 0.4], shinR: [0.2, 0.26, 0.4],
};

export interface SplashParticle {
  p: Vec3; v: Vec3; life: number; maxLife: number;
}

export class DiveRenderer {
  private gl: WebGL2RenderingContext;
  private prog: WebGLProgram;
  private capsule: Mesh;
  private cube: Mesh;
  private uLoc: Record<string, WebGLUniformLocation | null> = {};
  cameraAzimuth = 0.5;
  cameraElevation = 0.18;
  cameraDistance = 7.5;
  cameraTarget: Vec3 = [0, 6, 0];
  private particles: SplashParticle[] = [];
  private splashEmitted = false;

  constructor(private canvas: HTMLCanvasElement) {
    const gl = canvas.getContext('webgl2', { antialias: true });
    if (!gl) throw new Error('WebGL2 不可用');
    this.gl = gl;
    this.prog = this.buildProgram(VERT, FRAG);
    this.capsule = makeCapsuleMesh(gl);
    this.cube = makeCubeMesh(gl);
    for (const n of ['uViewProj', 'uModel', 'uColor', 'uShade', 'uLightDir']) {
      this.uLoc[n] = gl.getUniformLocation(this.prog, n);
    }
    gl.enable(gl.DEPTH_TEST);
    gl.clearColor(0.82, 0.9, 0.96, 1);
  }

  private buildProgram(vs: string, fs: string): WebGLProgram {
    const gl = this.gl;
    const compile = (type: number, src: string) => {
      const sh = gl.createShader(type)!;
      gl.shaderSource(sh, src);
      gl.compileShader(sh);
      if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
        throw new Error(gl.getShaderInfoLog(sh) ?? 'shader error');
      }
      return sh;
    };
    const p = gl.createProgram()!;
    gl.attachShader(p, compile(gl.VERTEX_SHADER, vs));
    gl.attachShader(p, compile(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) {
      throw new Error(gl.getProgramInfoLog(p) ?? 'link error');
    }
    return p;
  }

  resetSplash(): void {
    this.particles = [];
    this.splashEmitted = false;
  }

  private emitSplash(center: Vec3, intensity: number): void {
    const n = Math.round(40 + intensity * 90);
    for (let i = 0; i < n; i++) {
      const a = Math.random() * Math.PI * 2;
      const r = Math.random() * 0.5 * intensity;
      const speed = (1.5 + Math.random() * 3.5) * (0.4 + intensity);
      this.particles.push({
        p: [center[0] + Math.cos(a) * r, 0.1, center[2] + Math.sin(a) * r],
        v: [Math.cos(a) * speed * 0.6, speed, Math.sin(a) * speed * 0.6],
        life: 0, maxLife: 0.7 + Math.random() * 0.7,
      });
    }
  }

  private drawMesh(mesh: Mesh, model: Mat4, color: Vec3, shade = 1): void {
    const gl = this.gl;
    gl.bindVertexArray(mesh.vao);
    gl.uniformMatrix4fv(this.uLoc.uModel, false, model);
    gl.uniform3fv(this.uLoc.uColor, color);
    gl.uniform1f(this.uLoc.uShade, shade);
    gl.drawElements(gl.TRIANGLES, mesh.count, gl.UNSIGNED_SHORT, 0);
  }

  private segmentModel(r: Vec3, R: Mat3, radius: number, length: number): Mat4 {
    // 胶囊网格：半径 1、圆柱半高 1（总高 4）。缩放到目标 r、半高 L/2-r
    const halfCyl = Math.max(radius, length / 2 - radius);
    const s = m4Identity();
    s[0] = radius; s[5] = halfCyl; s[10] = radius;
    return m4Mul(m4Translation(r[0], r[1], r[2]), m4Mul(m4FromMat3(R), s));
  }

  render(athlete: Athlete, snap: FrameSnapshot | null, splashIntensity: number): void {
    const gl = this.gl;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    const w = Math.floor(this.canvas.clientWidth * dpr);
    const h = Math.floor(this.canvas.clientHeight * dpr);
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    gl.useProgram(this.prog);

    const aspect = this.canvas.width / this.canvas.height;
    const proj = m4Perspective((50 * Math.PI) / 180, aspect, 0.1, 100);
    const az = this.cameraAzimuth;
    const el = this.cameraElevation;
    const eye: Vec3 = [
      this.cameraTarget[0] + Math.cos(el) * Math.sin(az) * this.cameraDistance,
      this.cameraTarget[1] + Math.sin(el) * this.cameraDistance,
      this.cameraTarget[2] + Math.cos(el) * Math.cos(az) * this.cameraDistance,
    ];
    const view = m4LookAt(eye, this.cameraTarget, [0, 1, 0]);
    const vp = m4Mul(proj, view);
    gl.uniformMatrix4fv(this.uLoc.uViewProj, false, vp);
    gl.uniform3f(this.uLoc.uLightDir, 0.4, 0.9, 0.3);

    // 水面
    this.drawMesh(this.cube, m4Mul(m4Translation(0, -0.15, 0), (() => {
      const s = m4Identity(); s[0] = 30; s[5] = 0.05; s[10] = 30; return s;
    })()), [0.18, 0.5, 0.72], 0.9);
    // 跳台（10m 高，台面向外 +x）
    const platTop = 10;
    this.drawMesh(this.cube, m4Mul(m4Translation(-1.2, platTop - 0.15, 0), (() => {
      const s = m4Identity(); s[0] = 1.6; s[5] = 0.15; s[10] = 0.8; return s;
    })()), [0.72, 0.74, 0.78]);
    this.drawMesh(this.cube, m4Mul(m4Translation(-2.6, 5, 0), (() => {
      const s = m4Identity(); s[0] = 0.12; s[5] = 5; s[10] = 0.12; return s;
    })()), [0.6, 0.62, 0.66]);

    // 人体
    if (snap) {
      const kin = forwardKinematics(athlete, snap.rootPos, snap.rootQ, snap.pose);
      athlete.segments.forEach((seg, i) => {
        const f = kin.frames[i];
        const model = this.segmentModel(f.r, f.R, seg.radius, seg.length);
        this.drawMesh(this.capsule, model, SEGMENT_COLORS[seg.name] ?? [0.8, 0.8, 0.8]);
      });
      // 入水后发射水花
      if (snap.phase === 'entered' && !this.splashEmitted) {
        const entry: Vec3 = [snap.com[0], 0, snap.com[2]];
        this.emitSplash(entry, splashIntensity);
        this.splashEmitted = true;
      }
    }

    this.updateParticles();
  }

  private updateParticles(): void {
    const gl = this.gl;
    const dt = 1 / 60;
    const alive: SplashParticle[] = [];
    for (const p of this.particles) {
      p.life += dt;
      if (p.life >= p.maxLife) continue;
      p.v[1] -= 9.81 * dt;
      p.p = vadd(p.p, [p.v[0] * dt, p.v[1] * dt, p.v[2] * dt]);
      if (p.p[1] < 0.02) { p.p[1] = 0.02; p.v[1] *= -0.3; p.v[0] *= 0.6; p.v[2] *= 0.6; }
      alive.push(p);
      const fade = 1 - p.life / p.maxLife;
      const s = m4Identity(); const k = 0.03 + 0.05 * fade;
      s[0] = k; s[5] = k; s[10] = k;
      this.drawMesh(this.cube, m4Mul(m4Translation(p.p[0], p.p[1], p.p[2]), s),
        [0.75, 0.9, 1.0], 0.6 + 0.4 * fade);
    }
    this.particles = alive;
    void gl;
  }
}
