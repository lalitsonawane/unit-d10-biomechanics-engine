import { GAIT_SPECS, LEG_IDS, UNIT_D10, type GaitMode, type LegId } from '../specs/unitD10Specs';
import type { TerrainField, Vec3 } from '../terrain/TerrainField';
import type { LegGaitState } from '../locomotion/GaitController';

export interface Quat {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface EulerAngles {
  roll: number; // around +X (+ = right bank)
  pitch: number; // around +Z (+ = nose up)
  yaw: number; // around +Y (+ = turn left)
}

export interface RigidBodyState {
  pos: Vec3;
  vel: Vec3;
  acc: Vec3;
  localAcc: Vec3;
  quat: Quat;
  euler: EulerAngles;
  /** Angular velocity in body frame (rad/s) */
  omegaBody: Vec3;
  /** Angular acceleration in body frame (rad/s^2) */
  alphaBody: Vec3;
  /** Ground Reaction Forces per leg in world frame (N) */
  grfWorld: Record<LegId, Vec3>;
  /** Ground Reaction Forces per leg in body frame (N) */
  grfBody: Record<LegId, Vec3>;
  /** Normal force magnitude per leg (N) */
  normalForce: Record<LegId, number>;
  /** Friction utilization ratio |Ft| / (mu * Fn) in [0..1] */
  frictionRatio: Record<LegId, number>;
  /** Total vertical ground reaction force (N) */
  totalVerticalGRF: number;
  /** Number of legs currently bearing ground load */
  activeContactCount: number;
  /** True when all 4 legs are airborne (ballistic flight phase) */
  isAirborne: boolean;
  /** Center of Pressure / ZMP ground projection in world XZ */
  copWorld: Vec3;
  /** CoM ground projection in world XZ */
  comGroundWorld: Vec3;
}

function clamp(v: number, min: number, max: number): number {
  return v < min ? min : v > max ? max : v;
}

export function quatRotateVec(q: Quat, v: Vec3): Vec3 {
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  return {
    x: v.x + q.w * tx + (q.y * tz - q.z * ty),
    y: v.y + q.w * ty + (q.z * tx - q.x * tz),
    z: v.z + q.w * tz + (q.x * ty - q.y * tx),
  };
}

export function quatInvRotateVec(q: Quat, v: Vec3): Vec3 {
  return quatRotateVec({ x: -q.x, y: -q.y, z: -q.z, w: q.w }, v);
}

export function quatFromEulerYXZ(yaw: number, pitch: number, roll: number): Quat {
  // Composition: R = R_y(yaw) * R_z(pitch) * R_x(roll)
  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);

  // q = qy * qz * qx
  // qy = (0, sy, 0, cy), qz = (0, 0, sp, cp), qx = (sr, 0, 0, cr)
  // First qyz = qy * qz:
  const yzx = sy * sp;
  const yzy = sy * cp;
  const yzz = cy * sp;
  const yzw = cy * cp;

  return {
    x: yzw * sr + yzx * cr,
    y: yzy * cr - yzz * sr,
    z: yzz * cr + yzy * sr,
    w: yzw * cr - yzx * sr,
  };
}

export function quatToEulerYZX(q: Quat): EulerAngles {
  // Extract yaw (Y), pitch (Z), roll (X) from rotation matrix R = R_y(yaw) * R_z(pitch) * R_x(roll)
  // R_00 = cy cp, R_10 = sp, R_20 = -sy cp
  // R_11 = cp cr, R_12 = -cp sr
  const r00 = 1 - 2 * (q.y * q.y + q.z * q.z);
  const r10 = 2 * (q.x * q.y + q.w * q.z);
  const r20 = 2 * (q.x * q.z - q.w * q.y);
  const r11 = 1 - 2 * (q.x * q.x + q.z * q.z);
  const r12 = 2 * (q.y * q.z - q.w * q.x);

  const pitch = Math.asin(clamp(r10, -1, 1));
  const yaw = Math.atan2(-r20, r00);
  const roll = Math.atan2(-r12, r11);
  return { roll, pitch, yaw };
}

/**
 * Solve 6x6 symmetric positive-definite linear system M * x = b via Gaussian elimination with partial pivoting.
 */
function solveLinear6(M: Float64Array, b: Float64Array): Float64Array {
  const n = 6;
  const A = new Float64Array(M);
  const x = new Float64Array(b);

  for (let i = 0; i < n; i++) {
    // Partial pivot
    let maxRow = i;
    let maxVal = Math.abs(A[i * n + i]);
    for (let r = i + 1; r < n; r++) {
      const val = Math.abs(A[r * n + i]);
      if (val > maxVal) {
        maxVal = val;
        maxRow = r;
      }
    }
    if (maxRow !== i) {
      for (let c = i; c < n; c++) {
        const tmp = A[i * n + c];
        A[i * n + c] = A[maxRow * n + c];
        A[maxRow * n + c] = tmp;
      }
      const tb = x[i];
      x[i] = x[maxRow];
      x[maxRow] = tb;
    }

    const diag = A[i * n + i];
    if (Math.abs(diag) < 1e-10) continue;

    for (let r = i + 1; r < n; r++) {
      const factor = A[r * n + i] / diag;
      for (let c = i + 1; c < n; c++) {
        A[r * n + c] -= factor * A[i * n + c];
      }
      x[r] -= factor * x[i];
    }
  }

  for (let i = n - 1; i >= 0; i--) {
    let sum = x[i];
    for (let c = i + 1; c < n; c++) {
      sum -= A[i * n + c] * x[c];
    }
    const diag = A[i * n + i];
    x[i] = Math.abs(diag) > 1e-10 ? sum / diag : 0;
  }

  return x;
}

export interface PhysicsStepInput {
  dt: number;
  gait: GaitMode;
  gaitPhase: number;
  desiredSpeed: number;
  desiredYawRate: number;
  legs: Record<LegId, LegGaitState>;
  terrain: TerrainField;
  /** Optional external impulse applied in body frame on this 240Hz tick (N·s) */
  externalBodyImpulse?: Vec3;
  /** Optional lever arm in body frame where external impulse acts (m) */
  externalImpulseLeverBody?: Vec3;
}

/**
 * 240Hz 6-DOF Rigid-Body Physics & 4-Leg Ground Reaction Force (GRF) Engine.
 * Uses exact Unit D-10 mass (55.0 kg) and inertia tensor (Ixx = 2.4, Iyy_pitch = 6.2, Izz_yaw = 7.1 kg·m²).
 */
export class RigidBodyEngine {
  public state: RigidBodyState;
  private prevContact: Record<LegId, boolean> = { FL: true, FR: true, RL: true, RR: true };
  private impactPulse: Record<LegId, number> = { FL: 0, FR: 0, RL: 0, RR: 0 };

  // Inertia in our body frame (+X forward/roll, +Y up/yaw, +Z right/pitch)
  private readonly Ix = UNIT_D10.inertia.Ixx; // 2.4 kg·m² (roll)
  private readonly Iy = UNIT_D10.inertia.Izz; // 7.1 kg·m² (yaw)
  private readonly Iz = UNIT_D10.inertia.Iyy; // 6.2 kg·m² (pitch)
  private readonly m = UNIT_D10.mass; // 55.0 kg
  private readonly g = UNIT_D10.gravity; // 9.81 m/s²

  constructor() {
    this.state = this.createInitialState(0);
  }

  private createInitialState(groundY: number): RigidBodyState {
    const y0 = groundY + UNIT_D10.envelope.nominalComHeight;
    const zero: Vec3 = { x: 0, y: 0, z: 0 };
    const mgQuarter = (this.m * this.g) / 4;
    return {
      pos: { x: 0, y: y0, z: 0 },
      vel: { ...zero },
      acc: { ...zero },
      localAcc: { ...zero },
      quat: { x: 0, y: 0, z: 0, w: 1 },
      euler: { roll: 0, pitch: 0, yaw: 0 },
      omegaBody: { ...zero },
      alphaBody: { ...zero },
      grfWorld: {
        FL: { x: 0, y: mgQuarter, z: 0 },
        FR: { x: 0, y: mgQuarter, z: 0 },
        RL: { x: 0, y: mgQuarter, z: 0 },
        RR: { x: 0, y: mgQuarter, z: 0 },
      },
      grfBody: {
        FL: { x: 0, y: mgQuarter, z: 0 },
        FR: { x: 0, y: mgQuarter, z: 0 },
        RL: { x: 0, y: mgQuarter, z: 0 },
        RR: { x: 0, y: mgQuarter, z: 0 },
      },
      normalForce: { FL: mgQuarter, FR: mgQuarter, RL: mgQuarter, RR: mgQuarter },
      frictionRatio: { FL: 0, FR: 0, RL: 0, RR: 0 },
      totalVerticalGRF: this.m * this.g,
      activeContactCount: 4,
      isAirborne: false,
      copWorld: { x: 0, y: groundY, z: 0 },
      comGroundWorld: { x: 0, y: groundY, z: 0 },
    };
  }

  public reset(terrain: TerrainField, dropHeight = 0): void {
    const groundY = terrain.getHeight(0, 0);
    this.state = this.createInitialState(groundY);
    this.state.pos.y += dropHeight;
    this.prevContact = { FL: dropHeight <= 0.01, FR: dropHeight <= 0.01, RL: dropHeight <= 0.01, RR: dropHeight <= 0.01 };
    this.impactPulse = { FL: 0, FR: 0, RL: 0, RR: 0 };
  }

  public applyWorldImpulse(impulseWorld: Vec3, leverBody: Vec3 = { x: 0, y: 0, z: 0 }): void {
    this.state.vel.x += impulseWorld.x / this.m;
    this.state.vel.y += impulseWorld.y / this.m;
    this.state.vel.z += impulseWorld.z / this.m;

    const impBody = quatInvRotateVec(this.state.quat, impulseWorld);
    // Angular impulse τ_imp = r × J in body frame
    const tx = leverBody.y * impBody.z - leverBody.z * impBody.y;
    const ty = leverBody.z * impBody.x - leverBody.x * impBody.z;
    const tz = leverBody.x * impBody.y - leverBody.y * impBody.x;
    this.state.omegaBody.x += tx / this.Ix;
    this.state.omegaBody.y += ty / this.Iy;
    this.state.omegaBody.z += tz / this.Iz;
  }

  public step(input: PhysicsStepInput): RigidBodyState {
    const s = this.state;
    const dt = input.dt;
    const spec = GAIT_SPECS[input.gait];

    // 1. Apply any instantaneous external body-frame impulse (e.g., railgun discharge!)
    if (input.externalBodyImpulse) {
      const imp = input.externalBodyImpulse;
      if (imp.x !== 0 || imp.y !== 0 || imp.z !== 0) {
        const impW = quatRotateVec(s.quat, imp);
        this.applyWorldImpulse(impW, input.externalImpulseLeverBody ?? { x: 0, y: 0, z: 0 });
      }
    }

    // 2. Collect active stance legs & compute terrain reference plane
    const activeLegs: LegId[] = [];
    let terrainSum = 0;
    for (const id of LEG_IDS) {
      const leg = input.legs[id];
      terrainSum += leg.terrainHeight;
      if (leg.inContact) {
        activeLegs.push(id);
        // Detect fresh touchdown -> inject realistic Hunt-Crossley foot-strike impact pulse!
        if (!this.prevContact[id]) {
          const downSpeed = Math.max(0.4, -s.vel.y + 0.8);
          this.impactPulse[id] = clamp(downSpeed * 65, 25, 130);
        }
      }
      this.prevContact[id] = leg.inContact;
      // Exponential decay of touchdown impact spike (~25ms)
      this.impactPulse[id] *= Math.exp(-dt * 42);
    }

    const avgTerrainY = terrainSum * 0.25;

    // Fit terrain pitch & roll from front/rear and left/right paw terrain heights
    const frontH = 0.5 * (input.legs.FL.terrainHeight + input.legs.FR.terrainHeight);
    const rearH = 0.5 * (input.legs.RL.terrainHeight + input.legs.RR.terrainHeight);
    const leftH = 0.5 * (input.legs.FL.terrainHeight + input.legs.RL.terrainHeight);
    const rightH = 0.5 * (input.legs.FR.terrainHeight + input.legs.RR.terrainHeight);

    const terrainPitch = Math.atan2(frontH - rearH, UNIT_D10.envelope.wheelbase);
    const terrainRoll = Math.atan2(leftH - rightH, UNIT_D10.envelope.trackWidth);

    // 3. Whole-Body Virtual Model Controller (VMC) Desired Wrench
    const yaw = s.euler.yaw;
    const fwdX = Math.cos(yaw);
    const fwdZ = -Math.sin(yaw);
    const desVx = fwdX * input.desiredSpeed;
    const desVz = fwdZ * input.desiredSpeed;

    // Dynamic height target + natural vertical pump during gallop so push-off launches into flight!
    let targetHeightOffset = 0;
    let gallopPitchWave = 0;
    let extraPushVy = 0;
    if (input.gait === 'gallop') {
      const phase2pi = input.gaitPhase * Math.PI * 2;
      // Rhythmic vertical gallop bound & forward-aft pitch rocking
      targetHeightOffset = 0.032 * Math.sin(phase2pi - 0.35);
      gallopPitchWave = 0.075 * Math.cos(phase2pi - 0.35);
      // Upward push-off impulse during late stance to produce crisp ballistic flight arc
      extraPushVy = 0.34 * Math.max(0, Math.sin(phase2pi * 2 - 0.4));
    } else if (input.gait === 'trot') {
      targetHeightOffset = 0.012 * Math.sin(input.gaitPhase * Math.PI * 4);
    } else if (input.gait === 'idle') {
      targetHeightOffset = 0.004 * Math.sin(input.gaitPhase * Math.PI * 2);
    }

    const desY = avgTerrainY + spec.bodyHeight + targetHeightOffset;
    const desPitch = clamp(terrainPitch + gallopPitchWave, -0.34, 0.34);
    // Subtle inward banking into fast turns (authentic quadruped biomechanics!)
    const turnBank = clamp(-input.desiredYawRate * input.desiredSpeed * 0.032, -0.14, 0.14);
    const desRoll = clamp(terrainRoll + turnBank, -0.28, 0.28);

    // Height & linear velocity PD gains (tuned for 55kg chassis at 240Hz)
    const kpY = input.gait === 'gallop' ? 460 : 340;
    const kdY = input.gait === 'gallop' ? 42 : 38;
    const kpVel = input.gait === 'idle' ? 9.5 : 15.0;

    const FdesX = this.m * kpVel * (desVx - s.vel.x);
    const FdesY = this.m * (this.g + kpY * (desY - s.pos.y) + kdY * (extraPushVy - s.vel.y));
    const FdesZ = this.m * kpVel * (desVz - s.vel.z);

    // Attitude PD in body frame -> transform to world frame
    const kpRoll = 260;
    const kdRoll = 24;
    const kpPitch = 360;
    const kdPitch = 36;
    const kdYaw = 68;

    const tauRollBody = kpRoll * (desRoll - s.euler.roll) - kdRoll * s.omegaBody.x;
    const tauPitchBody = kpPitch * (desPitch - s.euler.pitch) - kdPitch * s.omegaBody.z;
    const tauYawBody = kdYaw * (input.desiredYawRate - s.omegaBody.y);

    const tauDesWorld = quatRotateVec(s.quat, {
      x: tauRollBody,
      y: tauYawBody,
      z: tauPitchBody,
    });

    // 4. 4-Leg Ground Reaction Force Distribution via Weighted Regularized Least-Squares (6x6 Dual)
    const zeroVec: Vec3 = { x: 0, y: 0, z: 0 };
    for (const id of LEG_IDS) {
      s.grfWorld[id] = { ...zeroVec };
      s.grfBody[id] = { ...zeroVec };
      s.normalForce[id] = 0;
      s.frictionRatio[id] = 0;
    }

    const k = activeLegs.length;
    if (k > 0) {
      // Build 6x6 dual matrix M = A * A^T + diag(reg)
      // Each active foot i contributes block A_i = [ I_3 ; [r_i]_x ]
      // Let r_i = p_foot_i - p_CoM
      const M = new Float64Array(36);
      const rList: Vec3[] = [];

      for (let idx = 0; idx < k; idx++) {
        const leg = input.legs[activeLegs[idx]];
        const rx = leg.footWorld.x - s.pos.x;
        const ry = leg.footWorld.y - s.pos.y;
        const rz = leg.footWorld.z - s.pos.z;
        rList.push({ x: rx, y: ry, z: rz });

        // Top-left 3x3: += I_3
        M[0 * 6 + 0] += 1;
        M[1 * 6 + 1] += 1;
        M[2 * 6 + 2] += 1;

        // Top-right 3x3: += [r]_x^T = -[r]_x, Bottom-left 3x3: += [r]_x
        // [r]_x * f = (ry*fz - rz*fy, rz*fx - rx*fz, rx*fy - ry*fx)
        // [r]_x = [  0, -rz,  ry ]
        //         [ rz,   0, -rx ]
        //         [-ry,  rx,   0 ]
        M[3 * 6 + 1] += -rz;
        M[3 * 6 + 2] += ry;
        M[4 * 6 + 0] += rz;
        M[4 * 6 + 2] += -rx;
        M[5 * 6 + 0] += -ry;
        M[5 * 6 + 1] += rx;

        M[0 * 6 + 4] += rz;
        M[0 * 6 + 5] += -ry;
        M[1 * 6 + 3] += -rz;
        M[1 * 6 + 5] += rx;
        M[2 * 6 + 3] += ry;
        M[2 * 6 + 4] += -rx;

        // Bottom-right 3x3: += [r]_x * [r]_x^T = |r|^2 I - r r^T
        const r2 = rx * rx + ry * ry + rz * rz;
        M[3 * 6 + 3] += r2 - rx * rx;
        M[3 * 6 + 4] += -rx * ry;
        M[3 * 6 + 5] += -rx * rz;
        M[4 * 6 + 3] += -ry * rx;
        M[4 * 6 + 4] += r2 - ry * ry;
        M[4 * 6 + 5] += -ry * rz;
        M[5 * 6 + 3] += -rz * rx;
        M[5 * 6 + 4] += -rz * ry;
        M[5 * 6 + 5] += r2 - rz * rz;
      }

      // Diagonal regularization λ * W^(-1) (lower = higher priority; prioritize Fy & Pitch/Roll!)
      const reg = [0.06, 0.015, 0.06, 0.02, 0.05, 0.02];
      for (let d = 0; d < 6; d++) {
        M[d * 6 + d] += reg[d];
      }

      const b = new Float64Array([FdesX, FdesY, FdesZ, tauDesWorld.x, tauDesWorld.y, tauDesWorld.z]);
      const yDual = solveLinear6(M, b);

      const mu = UNIT_D10.contact.frictionCoeff;
      const maxFn = UNIT_D10.contact.maxNormalForce;

      for (let idx = 0; idx < k; idx++) {
        const id = activeLegs[idx];
        const r = rList[idx];
        const leg = input.legs[id];

        // f_i = A_i^T * yDual = y_0..2 + [r_i]_x^T * y_3..5 = y_F + y_T × r_i
        let fx = yDual[0] + (yDual[4] * r.z - yDual[5] * r.y);
        let fy = yDual[1] + (yDual[5] * r.x - yDual[3] * r.z);
        let fz = yDual[2] + (yDual[3] * r.y - yDual[4] * r.x);

        // Add subtle Hunt-Crossley touchdown impact compliance force
        fy += this.impactPulse[id];

        // Project onto local terrain surface normal for true unilateral & Coulomb friction cone constraints!
        const n = input.terrain.getNormal(leg.footWorld.x, leg.footWorld.z);
        const fnRaw = fx * n.x + fy * n.y + fz * n.z;
        const fn = clamp(fnRaw, 0, maxFn);

        let tx = fx - fnRaw * n.x;
        let ty = fy - fnRaw * n.y;
        let tz = fz - fnRaw * n.z;
        const ftMag = Math.hypot(tx, ty, tz);
        const maxFt = mu * fn;

        if (ftMag > maxFt && ftMag > 1e-6) {
          const scale = maxFt / ftMag;
          tx *= scale;
          ty *= scale;
          tz *= scale;
        }

        const clampedF: Vec3 = {
          x: fn * n.x + tx,
          y: fn * n.y + ty,
          z: fn * n.z + tz,
        };

        s.grfWorld[id] = clampedF;
        s.grfBody[id] = quatInvRotateVec(s.quat, clampedF);
        s.normalForce[id] = fn;
        s.frictionRatio[id] = maxFt > 1e-3 ? clamp(Math.hypot(tx, ty, tz) / maxFt, 0, 1) : 0;
      }
    }

    // 5. Sum Actual Clamped Ground Reaction Forces & Torques into 6-DOF Newton-Euler Rigid Body Equations!
    let sumFx = 0;
    let sumFy = 0;
    let sumFz = 0;
    let sumTx = 0;
    let sumTy = 0;
    let sumTz = 0;
    let copNumX = 0;
    let copNumZ = 0;
    let copDen = 0;

    for (const id of LEG_IDS) {
      const f = s.grfWorld[id];
      const leg = input.legs[id];
      sumFx += f.x;
      sumFy += f.y;
      sumFz += f.z;

      const rx = leg.footWorld.x - s.pos.x;
      const ry = leg.footWorld.y - s.pos.y;
      const rz = leg.footWorld.z - s.pos.z;

      sumTx += ry * f.z - rz * f.y;
      sumTy += rz * f.x - rx * f.z;
      sumTz += rx * f.y - ry * f.x;

      const w = Math.max(0, f.y);
      copNumX += leg.footWorld.x * w;
      copNumZ += leg.footWorld.z * w;
      copDen += w;
    }

    // Gentle aerodynamic drag + slight airborne rotational stability damping
    const drag = 0.6;
    sumFx -= s.vel.x * drag;
    sumFz -= s.vel.z * drag;

    // Linear acceleration & semi-implicit Euler integration
    s.acc.x = sumFx / this.m;
    s.acc.y = sumFy / this.m - this.g;
    s.acc.z = sumFz / this.m;
    s.localAcc = quatInvRotateVec(s.quat, s.acc);

    s.vel.x += s.acc.x * dt;
    s.vel.y += s.acc.y * dt;
    s.vel.z += s.acc.z * dt;

    s.pos.x += s.vel.x * dt;
    s.pos.y += s.vel.y * dt;
    s.pos.z += s.vel.z * dt;

    // Soft belly floor contact guard (only engages on extreme drops)
    const groundUnderCom = input.terrain.getHeight(s.pos.x, s.pos.z);
    const minComY = groundUnderCom + 0.30;
    if (s.pos.y < minComY) {
      s.pos.y = minComY;
      if (s.vel.y < 0) s.vel.y *= -0.1;
    }

    // Rotational dynamics in body frame: I * α = τ_body - ω × (I * ω)
    const tauBody = quatInvRotateVec(s.quat, { x: sumTx, y: sumTy, z: sumTz });
    const wx = s.omegaBody.x;
    const wy = s.omegaBody.y;
    const wz = s.omegaBody.z;

    // Gyroscopic cross product ω × (I * ω)
    const Iwx = this.Ix * wx;
    const Iwy = this.Iy * wy;
    const Iwz = this.Iz * wz;
    const gyroX = wy * Iwz - wz * Iwy;
    const gyroY = wz * Iwx - wx * Iwz;
    const gyroZ = wx * Iwy - wy * Iwx;

    // Tiny passive damping (air + internal cable harness)
    const angDamp = 0.8;
    s.alphaBody.x = (tauBody.x - gyroX - wx * angDamp) / this.Ix;
    s.alphaBody.y = (tauBody.y - gyroY - wy * angDamp) / this.Iy;
    s.alphaBody.z = (tauBody.z - gyroZ - wz * angDamp) / this.Iz;

    s.omegaBody.x = clamp(s.omegaBody.x + s.alphaBody.x * dt, -10, 10);
    s.omegaBody.y = clamp(s.omegaBody.y + s.alphaBody.y * dt, -10, 10);
    s.omegaBody.z = clamp(s.omegaBody.z + s.alphaBody.z * dt, -10, 10);

    // Quaternion integration: dq/dt = 0.5 * q ⊗ (ω_body, 0)
    const q = s.quat;
    const dqx = 0.5 * (q.w * s.omegaBody.x + q.y * s.omegaBody.z - q.z * s.omegaBody.y);
    const dqy = 0.5 * (q.w * s.omegaBody.y + q.z * s.omegaBody.x - q.x * s.omegaBody.z);
    const dqz = 0.5 * (q.w * s.omegaBody.z + q.x * s.omegaBody.y - q.y * s.omegaBody.x);
    const dqw = 0.5 * (-q.x * s.omegaBody.x - q.y * s.omegaBody.y - q.z * s.omegaBody.z);

    q.x += dqx * dt;
    q.y += dqy * dt;
    q.z += dqz * dt;
    q.w += dqw * dt;
    const qLen = Math.hypot(q.x, q.y, q.z, q.w) || 1;
    q.x /= qLen;
    q.y /= qLen;
    q.z /= qLen;
    q.w /= qLen;

    // Keep roll & pitch bounded against pathological flip-overs
    s.euler = quatToEulerYZX(q);
    if (Math.abs(s.euler.roll) > 0.72 || Math.abs(s.euler.pitch) > 0.72) {
      s.euler.roll = clamp(s.euler.roll, -0.72, 0.72);
      s.euler.pitch = clamp(s.euler.pitch, -0.72, 0.72);
      s.quat = quatFromEulerYXZ(s.euler.yaw, s.euler.pitch, s.euler.roll);
      s.omegaBody.x *= 0.5;
      s.omegaBody.z *= 0.5;
    }

    s.totalVerticalGRF = sumFy;
    s.activeContactCount = k;
    s.isAirborne = k === 0;
    s.comGroundWorld = { x: s.pos.x, y: groundUnderCom, z: s.pos.z };
    if (copDen > 1.0) {
      const copX = copNumX / copDen;
      const copZ = copNumZ / copDen;
      s.copWorld = { x: copX, y: input.terrain.getHeight(copX, copZ), z: copZ };
    } else {
      s.copWorld = { ...s.comGroundWorld };
    }

    return s;
  }
}
