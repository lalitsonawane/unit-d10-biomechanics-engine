import { DEG2RAD, UNIT_D10, type GaitMode } from '../specs/unitD10Specs';
import type { Vec3 } from '../terrain/TerrainField';

function clamp(v: number, min: number, max: number): number {
  return v < min ? min : v > max ? max : v;
}

export interface SecondaryInput {
  gait: GaitMode;
  gaitPhase: number;
  chassisPitch: number; // rad (+ = nose up)
  chassisRoll: number; // rad (+ = bank right)
  chassisYawRate: number; // rad/s (+ = turn left)
  localAccel: Vec3; // m/s^2 in body frame (+X fwd, +Y up, +Z right)
  targetTurretAzimuth: number; // rad (±148°)
  targetTurretElevation: number; // rad (-24°..+34°)
  forwardSpeed: number; // m/s
}

export interface RecoilFireResult {
  fired: boolean;
  /** Recoil impulse vector in body frame (N·s) */
  bodyImpulse: Vec3;
  /** Spine disturbance torque (N·m) */
  spinePitchTorque: number;
  spineYawTorque: number;
  /** Turret barrel firing direction in thorax local frame (unit vector) */
  barrelDirLocal: Vec3;
}

export interface SecondaryState {
  // Dorsal Railgun Turret
  turretAzimuth: number; // rad
  turretElevation: number; // rad
  recoilDisp: number; // m [0..0.125] (125mm linear stroke)
  recoilVel: number; // m/s
  capacitorCharge: number; // [0..1]
  muzzleFlash: number; // [0..1] decaying flash intensity
  cooldownRemaining: number; // s
  shotsFired: number;

  // Recoil-Responsive Head & Neck
  neckPitch: number; // rad (+ = head up)
  neckPitchVel: number;
  neckYaw: number; // rad (+ = look left)
  neckYawVel: number;
  headRoll: number; // rad (horizon roll stabilization)
  headRollVel: number;
  headRecoilOffset: number; // m slight axial flinch

  // Acoustic / Tactical Ears (pin back on railgun blast, track motion)
  earLeftPitch: number; // rad (0 = upright, negative = pinned back)
  earLeftVel: number;
  earRightPitch: number;
  earRightVel: number;
  earSwivel: number; // rad subtle alertness sweep

  // 5-Segment Inertial Tail (whip + counterbalance dynamics)
  tailPitch: number[]; // length 5
  tailPitchVel: number[];
  tailYaw: number[]; // length 5
  tailYawVel: number[];
}

export const TAIL_SEGMENTS = 5;

/**
 * 240Hz Secondary Biomechanics & Weapon Dynamics:
 * - IMU-Stabilized Dorsal Railgun Turret with 125mm hydro-pneumatic recoil stroke
 * - Vestibulo-ocular stabilized Head & Neck with blast-wave flinch & inertial recovery
 * - Blast-reactive Acoustic/Radar Ears
 * - 5-segment coupled inertial counterbalance Tail
 */
export class SecondaryDynamics {
  public state: SecondaryState = this.createInitialState();
  private fireQueued = false;
  private recoilImpulseActiveTimer = 0;
  private activeSpinePitchKick = 0;
  private activeSpineYawKick = 0;

  private createInitialState(): SecondaryState {
    return {
      turretAzimuth: 0,
      turretElevation: UNIT_D10.turret.restElevation,
      recoilDisp: 0,
      recoilVel: 0,
      capacitorCharge: 1.0,
      muzzleFlash: 0,
      cooldownRemaining: 0,
      shotsFired: 0,

      neckPitch: 0,
      neckPitchVel: 0,
      neckYaw: 0,
      neckYawVel: 0,
      headRoll: 0,
      headRollVel: 0,
      headRecoilOffset: 0,

      earLeftPitch: 0,
      earLeftVel: 0,
      earRightPitch: 0,
      earRightVel: 0,
      earSwivel: 0,

      tailPitch: new Array(TAIL_SEGMENTS).fill(0),
      tailPitchVel: new Array(TAIL_SEGMENTS).fill(0),
      tailYaw: new Array(TAIL_SEGMENTS).fill(0),
      tailYawVel: new Array(TAIL_SEGMENTS).fill(0),
    };
  }

  public reset(): void {
    this.state = this.createInitialState();
    this.fireQueued = false;
    this.recoilImpulseActiveTimer = 0;
    this.activeSpinePitchKick = 0;
    this.activeSpineYawKick = 0;
  }

  public requestFire(): boolean {
    if (this.state.cooldownRemaining <= 0.01 && this.state.capacitorCharge >= 0.85) {
      this.fireQueued = true;
      return true;
    }
    return false;
  }

  public applyExternalShock(lateralImpulse: number, verticalImpulse: number): void {
    // Shoving the body whips the head, ears, and tail inertially!
    this.state.neckYawVel += clamp(lateralImpulse * 0.08, -8, 8);
    this.state.headRollVel += clamp(-lateralImpulse * 0.06, -6, 6);
    this.state.earLeftVel -= Math.abs(lateralImpulse) * 0.06;
    this.state.earRightVel -= Math.abs(lateralImpulse) * 0.06;
    for (let i = 0; i < TAIL_SEGMENTS; i++) {
      const gain = 1 + i * 0.45;
      this.state.tailYawVel[i] += clamp(-lateralImpulse * 0.14 * gain, -16, 16);
      this.state.tailPitchVel[i] += clamp(-verticalImpulse * 0.06 * gain, -12, 12);
    }
  }

  public step(dt: number, input: SecondaryInput): RecoilFireResult {
    const s = this.state;

    // 1. Turret Azimuth & Elevation IMU-Stabilized Tracking
    const targetAz = clamp(
      input.targetTurretAzimuth,
      -UNIT_D10.turret.azimuthLimit,
      UNIT_D10.turret.azimuthLimit
    );
    // Compensate chassis pitch partially so turret stays IMU-stabilized on horizon
    const stabilizedEl = clamp(
      input.targetTurretElevation - input.chassisPitch * 0.75,
      UNIT_D10.turret.elevationMin,
      UNIT_D10.turret.elevationMax
    );

    const maxAzRate = 3.2; // rad/s
    const maxElRate = 2.4; // rad/s
    const azErr = targetAz - s.turretAzimuth;
    const elErr = stabilizedEl - s.turretElevation;
    s.turretAzimuth += clamp(azErr * 9.0, -maxAzRate, maxAzRate) * dt;
    s.turretElevation += clamp(elErr * 9.0, -maxElRate, maxElRate) * dt;

    // Barrel direction in thorax frame (+X forward, +Y up, +Z right)
    // Yaw +az turns from +X toward -Z; Pitch +el tilts from horizontal toward +Y
    const cosEl = Math.cos(s.turretElevation);
    const sinEl = Math.sin(s.turretElevation);
    const cosAz = Math.cos(s.turretAzimuth);
    const sinAz = Math.sin(s.turretAzimuth);
    const barrelDirLocal: Vec3 = {
      x: cosEl * cosAz,
      y: sinEl,
      z: -cosEl * sinAz,
    };

    // Cooldown & supercapacitor recharge
    if (s.cooldownRemaining > 0) {
      s.cooldownRemaining = Math.max(0, s.cooldownRemaining - dt);
    }
    s.capacitorCharge = clamp(s.capacitorCharge + dt * 1.85, 0, 1);
    s.muzzleFlash = Math.max(0, s.muzzleFlash - dt * 6.5);

    // 2. Railgun Firing & 125mm Linear Recoil Stroke
    let fired = false;
    const bodyImpulse: Vec3 = { x: 0, y: 0, z: 0 };

    if (this.fireQueued && s.cooldownRemaining <= 0) {
      this.fireQueued = false;
      fired = true;
      s.shotsFired += 1;
      s.cooldownRemaining = UNIT_D10.turret.cooldownSec;
      s.capacitorCharge = 0.08;
      s.muzzleFlash = 1.0;

      // Kick linear barrel recoil sled rapidly to ~114-122mm stroke (within 125mm blueprint stroke)
      s.recoilVel = 4.65;

      // Body recoil impulse opposite to firing vector
      const J = UNIT_D10.turret.recoilImpulseNs;
      bodyImpulse.x = -barrelDirLocal.x * J;
      bodyImpulse.y = -barrelDirLocal.y * J;
      bodyImpulse.z = -barrelDirLocal.z * J;

      // Spine reaction torque pulse (~55ms duration)
      this.recoilImpulseActiveTimer = 0.055;
      this.activeSpinePitchKick = -barrelDirLocal.x * 34.0;
      this.activeSpineYawKick = barrelDirLocal.z * 30.0;

      // Recoil-responsive Head & Neck flinch!
      // Blast wave directly over head ducks nose down and jolts sideways opposite muzzle azimuth
      const forwardBlastFactor = Math.max(0.25, barrelDirLocal.x);
      s.neckPitchVel -= 7.2 * forwardBlastFactor + 2.4;
      s.neckYawVel += barrelDirLocal.z * 6.8;
      s.headRollVel += barrelDirLocal.z * 3.8;
      s.headRecoilOffset = 0.032;

      // Ears violently pin back from electromagnetic muzzle blast!
      s.earLeftVel -= 16.5 + (barrelDirLocal.z < 0 ? 3.5 : 0);
      s.earRightVel -= 16.5 + (barrelDirLocal.z > 0 ? 3.5 : 0);

      // Tail whips sharply as dynamic counterbalance to railgun kick!
      for (let i = 0; i < TAIL_SEGMENTS; i++) {
        const whipGain = 1.0 + i * 0.52;
        s.tailPitchVel[i] += (5.8 * barrelDirLocal.x + 2.2) * whipGain;
        s.tailYawVel[i] += -barrelDirLocal.z * 10.0 * whipGain;
      }
    }

    // Hydro-pneumatic recoil sled dynamics (0..0.125 m)
    const kRec = 240;
    const cRec = 21;
    const recAcc = -kRec * s.recoilDisp - cRec * s.recoilVel;
    s.recoilVel += recAcc * dt;
    s.recoilDisp += s.recoilVel * dt;
    if (s.recoilDisp > UNIT_D10.turret.recoilStroke) {
      s.recoilDisp = UNIT_D10.turret.recoilStroke;
      if (s.recoilVel > 0) s.recoilVel *= -0.18;
    } else if (s.recoilDisp < 0) {
      s.recoilDisp = 0;
      if (s.recoilVel < 0) s.recoilVel = 0;
    }

    let spinePitchTorque = 0;
    let spineYawTorque = 0;
    if (this.recoilImpulseActiveTimer > 0) {
      this.recoilImpulseActiveTimer = Math.max(0, this.recoilImpulseActiveTimer - dt);
      const env = this.recoilImpulseActiveTimer / 0.055;
      spinePitchTorque = this.activeSpinePitchKick * env;
      spineYawTorque = this.activeSpineYawKick * env;
    }

    // 3. Head & Neck Vestibulo-Ocular Stabilization + Inertial Bob
    // Counter chassis pitch & roll so optical payload remains steady
    const targetNeckPitch = clamp(
      -input.chassisPitch * 0.82 - input.localAccel.y * 0.009 + (input.gait === 'gallop' ? 0.06 : 0.0),
      -34 * DEG2RAD,
      36 * DEG2RAD
    );
    const targetNeckYaw = clamp(
      input.chassisYawRate * 0.22 + s.turretAzimuth * 0.24,
      -42 * DEG2RAD,
      42 * DEG2RAD
    );
    const targetHeadRoll = clamp(-input.chassisRoll * 0.85, -24 * DEG2RAD, 24 * DEG2RAD);

    // Inertial coupling from sudden body acceleration
    const neckPitchAcc =
      145 * (targetNeckPitch - s.neckPitch) -
      14.5 * s.neckPitchVel -
      input.localAccel.y * 0.32 +
      input.localAccel.x * 0.18;
    const neckYawAcc =
      120 * (targetNeckYaw - s.neckYaw) -
      13.5 * s.neckYawVel -
      input.localAccel.z * 0.38;
    const headRollAcc =
      160 * (targetHeadRoll - s.headRoll) - 16.0 * s.headRollVel;

    s.neckPitchVel += neckPitchAcc * dt;
    s.neckPitch = clamp(s.neckPitch + s.neckPitchVel * dt, -42 * DEG2RAD, 42 * DEG2RAD);
    s.neckYawVel += neckYawAcc * dt;
    s.neckYaw = clamp(s.neckYaw + s.neckYawVel * dt, -50 * DEG2RAD, 50 * DEG2RAD);
    s.headRollVel += headRollAcc * dt;
    s.headRoll = clamp(s.headRoll + s.headRollVel * dt, -30 * DEG2RAD, 30 * DEG2RAD);
    s.headRecoilOffset += (0 - s.headRecoilOffset) * Math.min(1, dt * 16);

    // 4. Acoustic / Radar Ears Dynamics
    const aeroSweep = -clamp(input.forwardSpeed * 0.055, 0, 22 * DEG2RAD);
    const earTargetL = aeroSweep - clamp(input.chassisYawRate * 0.08, -0.1, 0.1);
    const earTargetR = aeroSweep + clamp(input.chassisYawRate * 0.08, -0.1, 0.1);

    const earAccL = 135 * (earTargetL - s.earLeftPitch) - 11.0 * s.earLeftVel - input.localAccel.x * 0.15;
    const earAccR = 135 * (earTargetR - s.earRightPitch) - 11.0 * s.earRightVel - input.localAccel.x * 0.15;

    s.earLeftVel += earAccL * dt;
    s.earRightVel += earAccR * dt;
    s.earLeftPitch = clamp(s.earLeftPitch + s.earLeftVel * dt, -68 * DEG2RAD, 22 * DEG2RAD);
    s.earRightPitch = clamp(s.earRightPitch + s.earRightVel * dt, -68 * DEG2RAD, 22 * DEG2RAD);
    s.earSwivel += (clamp(s.turretAzimuth * 0.18, -0.3, 0.3) - s.earSwivel) * Math.min(1, dt * 8);

    // 5. 5-Segment Coupled Inertial Tail Chain
    // Natural droop in idle matching Blueprint View A, lifts gracefully toward horizontal during fast trot/gallop
    const speedLift = clamp(input.forwardSpeed * 0.075, 0, 0.28);
    const baseTailPitch = -0.36 + speedLift;
    const gallopTailPump =
      input.gait === 'gallop'
        ? 0.14 * Math.sin(input.gaitPhase * Math.PI * 2 - 0.8)
        : input.gait === 'trot'
          ? 0.04 * Math.sin(input.gaitPhase * Math.PI * 2)
          : 0.015 * Math.sin(input.gaitPhase * Math.PI * 2);

    // Counterbalance yaw swings opposite to centrifugal turn force
    const turnCounterYaw = clamp(-input.chassisYawRate * 0.24 - input.localAccel.z * 0.035, -0.45, 0.45);

    for (let i = 0; i < TAIL_SEGMENTS; i++) {
      // Stiffness decreases toward tail tip for organic whip wave propagation
      const flexFactor = 1.0 - i * 0.14;
      const kp = 95 * flexFactor;
      const kd = 8.2 * flexFactor;

      const parentPitch = i === 0 ? baseTailPitch + gallopTailPump : s.tailPitch[i - 1] * 0.65;
      const parentYaw = i === 0 ? turnCounterYaw : s.tailYaw[i - 1] * 0.72;

      // Inertial forcing increases down the chain
      const inertiaDrivePitch = (-input.localAccel.y * 0.24 + input.localAccel.x * 0.14) * (1 + i * 0.22);
      const inertiaDriveYaw = (-input.localAccel.z * 0.34 - input.chassisYawRate * 0.8) * (1 + i * 0.28);

      const pAcc = kp * (parentPitch - s.tailPitch[i]) - kd * s.tailPitchVel[i] + inertiaDrivePitch;
      const yAcc = kp * (parentYaw - s.tailYaw[i]) - kd * s.tailYawVel[i] + inertiaDriveYaw;

      s.tailPitchVel[i] += pAcc * dt;
      s.tailYawVel[i] += yAcc * dt;
      s.tailPitch[i] = clamp(s.tailPitch[i] + s.tailPitchVel[i] * dt, -0.78, 0.72);
      s.tailYaw[i] = clamp(s.tailYaw[i] + s.tailYawVel[i] * dt, -0.75, 0.75);
    }

    return {
      fired,
      bodyImpulse,
      spinePitchTorque,
      spineYawTorque,
      barrelDirLocal,
    };
  }
}
