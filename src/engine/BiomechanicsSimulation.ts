import {
  GAIT_SPECS,
  LEG_IDS,
  UNIT_D10,
  type GaitMode,
  type LegId,
  type TerrainMode,
} from '../specs/unitD10Specs';
import { TerrainField, type Vec3 } from '../terrain/TerrainField';
import { solveLegIK, type LegIKSolution } from '../kinematics/QuadrupedIK';
import { SpineSolver, type SpineState } from '../biomechanics/SpineSolver';
import { SecondaryDynamics, type SecondaryState } from '../biomechanics/SecondaryDynamics';
import { GaitController, type LegGaitState } from '../locomotion/GaitController';
import {
  RigidBodyEngine,
  quatFromEulerYXZ,
  quatInvRotateVec,
  quatRotateVec,
  type Quat,
  type RigidBodyState,
} from '../physics/RigidBodyEngine';

export interface SegmentPose {
  pos: Vec3;
  quat: Quat;
}

export interface ProjectileBolt {
  id: number;
  origin: Vec3;
  pos: Vec3;
  dir: Vec3;
  age: number;
  maxAge: number;
  speed: number;
}

export interface TelemetrySample {
  time: number;
  grfFL: number;
  grfFR: number;
  grfRL: number;
  grfRR: number;
  totalGRF: number;
  spinePitchDeg: number;
  spineYawDeg: number;
  bodyHeight: number;
  bodyPitchDeg: number;
  bodyRollDeg: number;
  recoilMm: number;
  headPitchDeg: number;
  tailTipYawDeg: number;
  isAirborne: boolean;
}

export interface SimulationCommand {
  gait: GaitMode;
  desiredSpeed: number; // m/s
  desiredYawRate: number; // rad/s
  turretAzimuth: number; // rad
  turretElevation: number; // rad
  timeScale: number; // 0.1..1.0
  paused: boolean;
}

export interface SimulationSnapshot {
  simTime: number;
  stepCount: number;
  physicsHz: number;
  command: SimulationCommand;
  body: RigidBodyState;
  frontSegment: SegmentPose;
  rearSegment: SegmentPose;
  hipWorld: Record<LegId, Vec3>;
  spine: SpineState;
  secondary: SecondaryState;
  legs: Record<LegId, LegGaitState>;
  ik: Record<LegId, LegIKSolution>;
  projectiles: ProjectileBolt[];
  peakTorquePct: number;
  gaitPhase: number;
}

function quatMultiply(a: Quat, b: Quat): Quat {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

/**
 * Master 240Hz Quadruped Biomechanics Engine for Unit D-10.
 * Coordinates:
 *   - 240Hz 6-DOF Rigid-Body Physics & 4-Leg Ground Reaction Forces
 *   - 3D Analytical Forelimb & 4-Bar Pantograph Hindlimb Inverse Kinematics
 *   - Dynamic 2-DOF Lumbar Spine Harmonic Drive
 *   - Walk / Trot / Gallop Gait Generator with Raibert Footsteps & Aerial Flight
 *   - Recoil-Responsive Dorsal Railgun, Head/Neck, Acoustic Ears & 5-Segment Tail
 */
export class BiomechanicsSimulation {
  public readonly terrain = new TerrainField();
  public readonly physics = new RigidBodyEngine();
  public readonly gaitController = new GaitController();
  public readonly spineSolver = new SpineSolver();
  public readonly secondaryDynamics = new SecondaryDynamics();

  public command: SimulationCommand = {
    gait: 'trot',
    desiredSpeed: GAIT_SPECS.trot.nominalSpeed,
    desiredYawRate: 0,
    turretAzimuth: 0,
    turretElevation: UNIT_D10.turret.restElevation,
    timeScale: 1.0,
    paused: false,
  };

  public simTime = 0;
  public stepCount = 0;
  public projectiles: ProjectileBolt[] = [];

  // Ring buffer of 240Hz telemetry (downsampled 2:1 -> 120Hz, 240 samples = 2.0 seconds window)
  public readonly historyCapacity = 240;
  public readonly history: TelemetrySample[] = [];

  private accumulator = 0;
  private nextProjectileId = 1;
  private latestSnapshot: SimulationSnapshot;

  constructor() {
    this.latestSnapshot = this.buildInitial();
  }

  private buildInitial(): SimulationSnapshot {
    this.physics.reset(this.terrain, 0);
    this.spineSolver.reset();
    this.secondaryDynamics.reset();
    this.gaitController.initializeFeet(
      this.physics.state.pos,
      this.physics.state.euler.yaw,
      this.terrain,
      this.command.gait
    );
    this.simTime = 0;
    this.stepCount = 0;
    this.accumulator = 0;
    this.projectiles = [];
    this.history.length = 0;
    return this.computeSnapshot();
  }

  public reset(dropHeight = 0): void {
    this.physics.reset(this.terrain, dropHeight);
    this.spineSolver.reset();
    this.secondaryDynamics.reset();
    this.gaitController.initializeFeet(
      this.physics.state.pos,
      this.physics.state.euler.yaw,
      this.terrain,
      this.command.gait
    );
    this.projectiles = [];
    this.latestSnapshot = this.computeSnapshot();
  }

  public setGait(mode: GaitMode, setNominalSpeed = true): void {
    this.command.gait = mode;
    if (setNominalSpeed) {
      this.command.desiredSpeed = GAIT_SPECS[mode].nominalSpeed;
    }
  }

  public setTerrain(mode: TerrainMode): void {
    this.terrain.setMode(mode);
    // Re-seat feet cleanly onto new terrain height under current body position
    const groundY = this.terrain.getHeight(this.physics.state.pos.x, this.physics.state.pos.z);
    this.physics.state.pos.y = groundY + GAIT_SPECS[this.command.gait].bodyHeight;
    this.physics.state.vel.y = 0;
    this.gaitController.initializeFeet(
      this.physics.state.pos,
      this.physics.state.euler.yaw,
      this.terrain,
      this.command.gait
    );
    this.latestSnapshot = this.computeSnapshot();
  }

  public fireRailgun(): boolean {
    return this.secondaryDynamics.requestFire();
  }

  public applyLateralShove(direction: 1 | -1 = 1, magnitudeNs = 115): void {
    // Lateral impulse along body +Z or -Z acting slightly above CoM to excite roll + lateral recovery
    const impBody: Vec3 = { x: 0, y: 12, z: direction * magnitudeNs };
    const impWorld = quatRotateVec(this.physics.state.quat, impBody);
    this.physics.applyWorldImpulse(impWorld, { x: 0.04, y: 0.14, z: 0 });
    this.secondaryDynamics.applyExternalShock(direction * magnitudeNs, 12);
  }

  public triggerDropTest(heightMeters = 0.55): void {
    this.reset(heightMeters);
  }

  /** Advance simulation from variable render frame delta (seconds) using strict 240Hz fixed substeps */
  public update(frameDeltaSec: number): SimulationSnapshot {
    if (this.command.paused) {
      return this.latestSnapshot;
    }

    const clampedFrame = Math.min(frameDeltaSec, 0.05); // max 50ms catch-up to prevent spiral of death
    this.accumulator += clampedFrame * this.command.timeScale;

    const dt = UNIT_D10.dt; // 1 / 240 s
    let stepsThisFrame = 0;
    const maxSubsteps = 16;

    while (this.accumulator >= dt && stepsThisFrame < maxSubsteps) {
      this.stepFixed(dt);
      this.accumulator -= dt;
      stepsThisFrame++;
    }

    this.latestSnapshot = this.computeSnapshot();
    return this.latestSnapshot;
  }

  /** Advance exactly one 240Hz physics tick (4.167 ms) — useful for step-by-step inspection & tests */
  public stepOnce(): SimulationSnapshot {
    this.stepFixed(UNIT_D10.dt);
    this.latestSnapshot = this.computeSnapshot();
    return this.latestSnapshot;
  }

  public getSnapshot(): SimulationSnapshot {
    return this.latestSnapshot;
  }

  private computeSegmentsAndHips(): {
    frontSegment: SegmentPose;
    rearSegment: SegmentPose;
    hipWorld: Record<LegId, Vec3>;
  } {
    const bodyPos = this.physics.state.pos;
    const bodyQuat = this.physics.state.quat;
    const sp = this.spineSolver.state;
    const halfSpan = UNIT_D10.spine.halfSpan; // 0.091 m

    // Spine bends symmetrically: front thorax pitches +pitch/2 and yaws +yaw/2; rear pelvis pitches -pitch/2, yaws -yaw/2
    //Wait: positive spine pitch = extension (arch concave up -> front nose pitches UP, rear rump pitches DOWN-back -> front +pitch/2, rear -pitch/2)
    // Positive spine yaw = curve left -> front yaws +yaw/2 (left), rear yaws -yaw/2 (right)
    const qFrontLocal = quatFromEulerYXZ(sp.yaw * 0.5, sp.pitch * 0.5, 0);
    const qRearLocal = quatFromEulerYXZ(-sp.yaw * 0.5, -sp.pitch * 0.5, 0);

    const frontQuat = quatMultiply(bodyQuat, qFrontLocal);
    const rearQuat = quatMultiply(bodyQuat, qRearLocal);

    // Front pivot offset along bent spine center
    const fOffWorld = quatRotateVec(bodyQuat, { x: halfSpan, y: 0, z: 0 });
    const rOffWorld = quatRotateVec(bodyQuat, { x: -halfSpan, y: 0, z: 0 });

    const frontPos: Vec3 = {
      x: bodyPos.x + fOffWorld.x,
      y: bodyPos.y + fOffWorld.y,
      z: bodyPos.z + fOffWorld.z,
    };
    const rearPos: Vec3 = {
      x: bodyPos.x + rOffWorld.x,
      y: bodyPos.y + rOffWorld.y,
      z: bodyPos.z + rOffWorld.z,
    };

    const hipWorld = {} as Record<LegId, Vec3>;
    for (const id of LEG_IDS) {
      const m = UNIT_D10.mounts[id];
      const seg = m.segment === 'front' ? { pos: frontPos, quat: frontQuat } : { pos: rearPos, quat: rearQuat };
      const offW = quatRotateVec(seg.quat, { x: m.localX, y: m.localY, z: m.localZ });
      hipWorld[id] = {
        x: seg.pos.x + offW.x,
        y: seg.pos.y + offW.y,
        z: seg.pos.z + offW.z,
      };
    }

    return {
      frontSegment: { pos: frontPos, quat: frontQuat },
      rearSegment: { pos: rearPos, quat: rearQuat },
      hipWorld,
    };
  }

  private stepFixed(dt: number): void {
    const body = this.physics.state;
    const { frontSegment, hipWorld } = this.computeSegmentsAndHips();

    // 1. Update Secondary Dynamics (Railgun Turret, Recoil, Head/Neck, Ears, Tail)
    const forwardSpeed =
      body.vel.x * Math.cos(body.euler.yaw) - body.vel.z * Math.sin(body.euler.yaw);

    const recoil = this.secondaryDynamics.step(dt, {
      gait: this.command.gait,
      gaitPhase: this.gaitController.masterPhase,
      chassisPitch: body.euler.pitch,
      chassisRoll: body.euler.roll,
      chassisYawRate: body.omegaBody.y,
      localAccel: body.localAcc,
      targetTurretAzimuth: this.command.turretAzimuth,
      targetTurretElevation: this.command.turretElevation,
      forwardSpeed,
    });

    // Spawn glowing electromagnetic railgun sabot bolt on fire!
    if (recoil.fired) {
      const mountWorld = quatRotateVec(frontSegment.quat, UNIT_D10.turret.mountOffset);
      const dirWorld = quatRotateVec(frontSegment.quat, recoil.barrelDirLocal);
      const muzzleDist = 0.62;
      const origin: Vec3 = {
        x: frontSegment.pos.x + mountWorld.x + dirWorld.x * muzzleDist,
        y: frontSegment.pos.y + mountWorld.y + dirWorld.y * muzzleDist,
        z: frontSegment.pos.z + mountWorld.z + dirWorld.z * muzzleDist,
      };
      this.projectiles.push({
        id: this.nextProjectileId++,
        origin: { ...origin },
        pos: { ...origin },
        dir: dirWorld,
        age: 0,
        maxAge: 0.45,
        speed: 82.0, // blazing hypervelocity tracer
      });
    }

    // Advance active railgun bolts
    for (let i = this.projectiles.length - 1; i >= 0; i--) {
      const p = this.projectiles[i];
      p.age += dt;
      p.pos.x += p.dir.x * p.speed * dt;
      p.pos.y += p.dir.y * p.speed * dt;
      p.pos.z += p.dir.z * p.speed * dt;
      if (p.age >= p.maxAge) {
        this.projectiles.splice(i, 1);
      }
    }

    // 2. Step Dynamic 2-DOF Lumbar Spine Solver
    const spec = GAIT_SPECS[this.command.gait];
    const speedRatio =
      spec.maxSpeed > 0 ? Math.abs(forwardSpeed) / Math.max(0.5, spec.nominalSpeed) : 0;

    // Sample terrain curvature (front slope minus rear slope) so spine conforms over crests & dips
    const cosY = Math.cos(body.euler.yaw);
    const sinY = Math.sin(body.euler.yaw);
    const hFront = this.terrain.getHeight(body.pos.x + 0.45 * cosY, body.pos.z - 0.45 * sinY);
    const hMid = this.terrain.getHeight(body.pos.x, body.pos.z);
    const hRear = this.terrain.getHeight(body.pos.x - 0.45 * cosY, body.pos.z + 0.45 * sinY);
    const slopeDiff = (hFront - hMid) / 0.45 - (hMid - hRear) / 0.45;

    this.spineSolver.step(dt, {
      gait: this.command.gait,
      gaitPhase: this.gaitController.masterPhase,
      speedRatio,
      yawRate: body.omegaBody.y,
      verticalAccel: body.acc.y,
      terrainSlopeDiff: slopeDiff,
      recoilPitchTorque: recoil.spinePitchTorque,
      recoilYawTorque: recoil.spineYawTorque,
    });

    // 3. Step Gait Oscillator & Raibert Footstep Planner
    this.gaitController.step({
      dt,
      gait: this.command.gait,
      desiredSpeed: this.command.gait === 'idle' ? 0 : this.command.desiredSpeed,
      desiredYawRate: this.command.desiredYawRate,
      bodyPos: body.pos,
      bodyVel: body.vel,
      bodyYaw: body.euler.yaw,
      hipWorldPositions: hipWorld,
      terrain: this.terrain,
    });

    // 4. Step 240Hz 6-DOF Rigid-Body Physics & 4-Leg Ground Reaction Force Solver
    // Lever arm of turret recoil above CoM in body frame: x ≈ +0.13m, y ≈ +0.26m (creates authentic nose-up pitch kick!)
    const leverArm: Vec3 = { x: 0.13, y: 0.26, z: 0 };
    this.physics.step({
      dt,
      gait: this.command.gait,
      gaitPhase: this.gaitController.masterPhase,
      desiredSpeed: this.command.gait === 'idle' ? 0 : this.command.desiredSpeed,
      desiredYawRate: this.command.desiredYawRate,
      legs: this.gaitController.legs,
      terrain: this.terrain,
      externalBodyImpulse: recoil.fired ? recoil.bodyImpulse : undefined,
      externalImpulseLeverBody: leverArm,
    });

    this.simTime += dt;
    this.stepCount += 1;

    // Record telemetry at 120Hz (every 2nd 240Hz tick)
    if (this.stepCount % 2 === 0) {
      const s = this.physics.state;
      const sp = this.spineSolver.state;
      const sec = this.secondaryDynamics.state;
      const RAD2DEG = 180 / Math.PI;
      this.history.push({
        time: this.simTime,
        grfFL: s.normalForce.FL,
        grfFR: s.normalForce.FR,
        grfRL: s.normalForce.RL,
        grfRR: s.normalForce.RR,
        totalGRF: s.totalVerticalGRF,
        spinePitchDeg: sp.pitch * RAD2DEG,
        spineYawDeg: sp.yaw * RAD2DEG,
        bodyHeight: s.pos.y - s.comGroundWorld.y,
        bodyPitchDeg: s.euler.pitch * RAD2DEG,
        bodyRollDeg: s.euler.roll * RAD2DEG,
        recoilMm: sec.recoilDisp * 1000,
        headPitchDeg: sec.neckPitch * RAD2DEG,
        tailTipYawDeg: sec.tailYaw[sec.tailYaw.length - 1] * RAD2DEG,
        isAirborne: s.isAirborne,
      });
      if (this.history.length > this.historyCapacity) {
        this.history.shift();
      }
    }
  }

  private computeSnapshot(): SimulationSnapshot {
    const { frontSegment, rearSegment, hipWorld } = this.computeSegmentsAndHips();
    const ik = {} as Record<LegId, LegIKSolution>;
    let peakTorquePct = 0;

    for (const id of LEG_IDS) {
      const mount = UNIT_D10.mounts[id];
      const seg = mount.segment === 'front' ? frontSegment : rearSegment;
      const footW = this.gaitController.legs[id].footWorld;
      const hipW = hipWorld[id];

      // Convert foot vector (footW - hipW) into the segment's local frame (which is the leg mount frame!)
      const deltaW: Vec3 = {
        x: footW.x - hipW.x,
        y: footW.y - hipW.y,
        z: footW.z - hipW.z,
      };
      const targetLocal = quatInvRotateVec(seg.quat, deltaW);
      const grfLocal = quatInvRotateVec(seg.quat, this.physics.state.grfWorld[id]);

      const sol = solveLegIK(id, targetLocal, grfLocal);
      ik[id] = sol;

      const abdPct = Math.abs(sol.torques.abduction) / UNIT_D10.actuators.abduction.peakTorque;
      const hipPct = Math.abs(sol.torques.hipPitch) / UNIT_D10.actuators.hipPitch.peakTorque;
      const kneePct = Math.abs(sol.torques.kneePitch) / UNIT_D10.actuators.elbowFlexion.peakTorque;
      peakTorquePct = Math.max(peakTorquePct, abdPct, hipPct, kneePct);
    }

    return {
      simTime: this.simTime,
      stepCount: this.stepCount,
      physicsHz: UNIT_D10.physicsHz,
      command: { ...this.command },
      body: this.physics.state,
      frontSegment,
      rearSegment,
      hipWorld,
      spine: this.spineSolver.state,
      secondary: this.secondaryDynamics.state,
      legs: this.gaitController.legs,
      ik,
      projectiles: this.projectiles,
      peakTorquePct: peakTorquePct * 100,
      gaitPhase: this.gaitController.masterPhase,
    };
  }
}
