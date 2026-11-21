import { GAIT_SPECS, UNIT_D10, type GaitMode } from '../specs/unitD10Specs';

function clamp(v: number, min: number, max: number): number {
  return v < min ? min : v > max ? max : v;
}

export interface SpineInput {
  gait: GaitMode;
  gaitPhase: number;
  speedRatio: number; // 0..1 normalized speed
  yawRate: number; // rad/s
  verticalAccel: number; // m/s^2
  terrainSlopeDiff: number; // rad (front slope - rear slope)
  recoilPitchTorque: number; // N·m impulse torque from railgun / impacts
  recoilYawTorque: number; // N·m impulse torque from railgun / impacts
}

export interface SpineState {
  pitch: number; // rad (+ = arch / concave up extension, - = flex / hunch)
  pitchVel: number; // rad/s
  yaw: number; // rad (+ = bend left)
  yawVel: number; // rad/s
  pitchTorque: number; // N·m (Harmonic Drive, max 45 N·m)
  yawTorque: number; // N·m (Harmonic Drive, max 45 N·m)
}

/**
 * 240Hz 2-DOF Lumbar Spine Harmonic Drive Solver.
 * Connects Front Thorax and Rear Pelvis across the 182mm lumbar span
 * with ±16° pitch and ±15° yaw ROM and 45 N·m actuator torque limits.
 */
export class SpineSolver {
  public state: SpineState = {
    pitch: 0,
    pitchVel: 0,
    yaw: 0,
    yawVel: 0,
    pitchTorque: 0,
    yawTorque: 0,
  };

  // Effective rotational inertia of front/rear halves around lumbar joint
  private readonly I_pitch = 0.65;
  private readonly I_yaw = 0.72;
  private readonly kpPitch = 165;
  private readonly kdPitch = 18.5;
  private readonly kpYaw = 150;
  private readonly kdYaw = 17.0;

  public reset(): void {
    this.state = {
      pitch: 0,
      pitchVel: 0,
      yaw: 0,
      yawVel: 0,
      pitchTorque: 0,
      yawTorque: 0,
    };
  }

  public step(dt: number, input: SpineInput): SpineState {
    const spec = GAIT_SPECS[input.gait];
    const twoPi = Math.PI * 2;
    const speedScale = input.gait === 'idle' ? 0.35 : clamp(0.35 + 0.65 * input.speedRatio, 0.35, 1.15);

    // 1. Gait-driven rhythmic lumbar flexion/extension & counter-yaw
    const gaitPitchTarget =
      spec.spinePitchAmp *
      speedScale *
      Math.sin(twoPi * (input.gaitPhase + spec.spinePhaseOffset));

    // In walk/trot, spine yaws smoothly to increase stride reach; plus curves into turns
    const gaitYawWave =
      spec.spineYawAmp *
      speedScale *
      Math.sin(twoPi * input.gaitPhase);

    const turnYawTarget = clamp(input.yawRate * 0.13, -0.18, 0.18);

    // Inertial compression on vertical landing + terrain conformity
    const inertialPitch = clamp(-input.verticalAccel * 0.0085, -0.12, 0.12);
    const terrainPitch = clamp(input.terrainSlopeDiff * 0.45, -0.14, 0.14);

    const targetPitch = clamp(
      gaitPitchTarget + inertialPitch + terrainPitch,
      -UNIT_D10.spine.pitchLimit,
      UNIT_D10.spine.pitchLimit
    );
    const targetYaw = clamp(
      gaitYawWave + turnYawTarget,
      -UNIT_D10.spine.yawLimit,
      UNIT_D10.spine.yawLimit
    );

    // 2. PD Harmonic Drive actuator torque (clamped to 45 N·m blueprint limit)
    const maxTau = UNIT_D10.spine.peakTorque;
    const tauPitchCmd =
      this.kpPitch * (targetPitch - this.state.pitch) - this.kdPitch * this.state.pitchVel;
    const tauYawCmd =
      this.kpYaw * (targetYaw - this.state.yaw) - this.kdYaw * this.state.yawVel;

    const tauPitchAct = clamp(tauPitchCmd, -maxTau, maxTau);
    const tauYawAct = clamp(tauYawCmd, -maxTau, maxTau);

    // 3. Semi-implicit Euler integration including external recoil torques
    const pitchAcc = (tauPitchAct + input.recoilPitchTorque) / this.I_pitch;
    const yawAcc = (tauYawAct + input.recoilYawTorque) / this.I_yaw;

    this.state.pitchVel += pitchAcc * dt;
    this.state.yawVel += yawAcc * dt;
    this.state.pitch += this.state.pitchVel * dt;
    this.state.yaw += this.state.yawVel * dt;

    // Hard mechanical end-stops with damping
    if (this.state.pitch > UNIT_D10.spine.pitchLimit) {
      this.state.pitch = UNIT_D10.spine.pitchLimit;
      if (this.state.pitchVel > 0) this.state.pitchVel *= -0.15;
    } else if (this.state.pitch < -UNIT_D10.spine.pitchLimit) {
      this.state.pitch = -UNIT_D10.spine.pitchLimit;
      if (this.state.pitchVel < 0) this.state.pitchVel *= -0.15;
    }

    if (this.state.yaw > UNIT_D10.spine.yawLimit) {
      this.state.yaw = UNIT_D10.spine.yawLimit;
      if (this.state.yawVel > 0) this.state.yawVel *= -0.15;
    } else if (this.state.yaw < -UNIT_D10.spine.yawLimit) {
      this.state.yaw = -UNIT_D10.spine.yawLimit;
      if (this.state.yawVel < 0) this.state.yawVel *= -0.15;
    }

    this.state.pitchTorque = tauPitchAct;
    this.state.yawTorque = tauYawAct;

    return this.state;
  }
}
