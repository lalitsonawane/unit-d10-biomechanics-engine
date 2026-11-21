import { GAIT_SPECS, LEG_IDS, UNIT_D10, type GaitMode, type LegId } from '../specs/unitD10Specs';
import type { TerrainField, Vec3 } from '../terrain/TerrainField';

function clamp(v: number, min: number, max: number): number {
  return v < min ? min : v > max ? max : v;
}

/** Smooth quintic interpolation s(u) in [0..1] with zero velocity & acceleration at endpoints */
function smooth5(u: number): number {
  const t = clamp(u, 0, 1);
  return t * t * t * (t * (t * 6 - 15) + 10);
}

/** Smooth bell curve for vertical swing clearance peaking at u = 0.5 with zero touchdown velocity */
function swingHeightCurve(u: number): number {
  const t = clamp(u, 0, 1);
  // 16 * t^2 * (1-t)^2 gives C1-continuous lift-off and soft landing
  const s = Math.sin(Math.PI * t);
  return s * s;
}

export interface LegGaitState {
  legId: LegId;
  /** Phase in [0..1) */
  phase: number;
  /** True when scheduled to be in stance */
  scheduledContact: boolean;
  /** Actual ground contact (stance AND within contact tolerance of terrain) */
  inContact: boolean;
  /** Progress within stance [0..1] or swing [0..1] */
  subPhase: number;
  /** Foot position in world coordinates */
  footWorld: Vec3;
  /** Foot velocity in world coordinates */
  footVelWorld: Vec3;
  /** Swing lift-off origin in world coordinates */
  liftOffWorld: Vec3;
  /** Raibert planned landing target in world coordinates */
  targetWorld: Vec3;
  /** Terrain height under paw */
  terrainHeight: number;
  /** Remaining swing height clearance (m) */
  clearance: number;
}

export interface GaitUpdateContext {
  dt: number;
  gait: GaitMode;
  desiredSpeed: number; // m/s (forward in body frame)
  desiredYawRate: number; // rad/s
  bodyPos: Vec3;
  bodyVel: Vec3;
  bodyYaw: number;
  hipWorldPositions: Record<LegId, Vec3>;
  terrain: TerrainField;
}

/**
 * Quadruped Gait Oscillator & Raibert Footstep Planner.
 * Drives realistic 4-beat Walk, 2-beat Diagonal Trot, and asymmetric Rotary Gallop
 * (with genuine aerial suspension phase!) and smooth transitions between gaits.
 */
export class GaitController {
  public masterPhase = 0; // [0..1)
  public currentMode: GaitMode = 'trot';
  public legs: Record<LegId, LegGaitState>;
  private blendedFrequency = GAIT_SPECS.trot.frequencyHz;
  private blendedDuty = GAIT_SPECS.trot.dutyFactor;
  private blendedOffsets: Record<LegId, number> = { ...GAIT_SPECS.trot.phaseOffsets };

  constructor() {
    this.legs = {
      FL: this.makeLegState('FL'),
      FR: this.makeLegState('FR'),
      RL: this.makeLegState('RL'),
      RR: this.makeLegState('RR'),
    };
  }

  private makeLegState(legId: LegId): LegGaitState {
    return {
      legId,
      phase: 0,
      scheduledContact: true,
      inContact: true,
      subPhase: 0,
      footWorld: { x: 0, y: 0, z: 0 },
      footVelWorld: { x: 0, y: 0, z: 0 },
      liftOffWorld: { x: 0, y: 0, z: 0 },
      targetWorld: { x: 0, y: 0, z: 0 },
      terrainHeight: 0,
      clearance: 0,
    };
  }

  public initializeFeet(
    bodyPos: Vec3,
    bodyYaw: number,
    terrain: TerrainField,
    mode: GaitMode
  ): void {
    this.currentMode = mode;
    this.masterPhase = 0;
    const spec = GAIT_SPECS[mode];
    this.blendedFrequency = spec.frequencyHz;
    this.blendedDuty = spec.dutyFactor;
    this.blendedOffsets = { ...spec.phaseOffsets };

    const cosY = Math.cos(bodyYaw);
    const sinY = Math.sin(bodyYaw);

    for (const id of LEG_IDS) {
      const m = UNIT_D10.mounts[id];
      // Nominal stance footprint directly under shoulders/hips at exact 585mm wheelbase & 364mm track
      const nomX = m.isFront ? UNIT_D10.envelope.wheelbase * 0.5 : -UNIT_D10.envelope.wheelbase * 0.5;
      const nomZ = m.side * UNIT_D10.envelope.trackWidth * 0.5;

      const wx = bodyPos.x + nomX * cosY + nomZ * sinY;
      const wz = bodyPos.z - nomX * sinY + nomZ * cosY;
      const wy = terrain.getHeight(wx, wz);

      const leg = this.legs[id];
      leg.phase = spec.phaseOffsets[id];
      leg.scheduledContact = true;
      leg.inContact = true;
      leg.subPhase = 0.5;
      leg.footWorld = { x: wx, y: wy, z: wz };
      leg.footVelWorld = { x: 0, y: 0, z: 0 };
      leg.liftOffWorld = { x: wx, y: wy, z: wz };
      leg.targetWorld = { x: wx, y: wy, z: wz };
      leg.terrainHeight = wy;
      leg.clearance = 0;
    }
  }

  public step(ctx: GaitUpdateContext): void {
    const spec = GAIT_SPECS[ctx.gait];
    this.currentMode = ctx.gait;

    // Smoothly blend gait parameters so mode switches never cause jarring discontinuities
    const blendRate = Math.min(1, ctx.dt * 5.5);

    // Modulate cadence slightly with commanded speed for lifelike stride scaling
    const speedAbs = Math.abs(ctx.desiredSpeed);
    const turnAbs = Math.abs(ctx.desiredYawRate);
    const speedFactor =
      ctx.gait === 'idle'
        ? 0.5
        : clamp(0.78 + 0.28 * (speedAbs / Math.max(0.5, spec.nominalSpeed)) + turnAbs * 0.12, 0.72, 1.25);

    const targetFreq = spec.frequencyHz * speedFactor;
    this.blendedFrequency += (targetFreq - this.blendedFrequency) * blendRate;
    this.blendedDuty += (spec.dutyFactor - this.blendedDuty) * blendRate;

    // Advance master phase
    this.masterPhase = (this.masterPhase + this.blendedFrequency * ctx.dt) % 1.0;

    // Shortest circular interpolation for phase offsets
    for (const id of LEG_IDS) {
      let diff = spec.phaseOffsets[id] - this.blendedOffsets[id];
      if (diff > 0.5) diff -= 1;
      if (diff < -0.5) diff += 1;
      this.blendedOffsets[id] = (this.blendedOffsets[id] + diff * blendRate + 1) % 1;
    }

    const cosY = Math.cos(ctx.bodyYaw);
    const sinY = Math.sin(ctx.bodyYaw);
    const stanceDuration = this.blendedDuty / Math.max(0.25, this.blendedFrequency);

    // Forward & right horizontal unit vectors in world frame
    // Remember: +X fwd rotated by +yaw around +Y -> fwd = (cosY, 0, -sinY), right = (sinY, 0, cosY)
    const fwdX = cosY;
    const fwdZ = -sinY;
    const rightX = sinY;
    const rightZ = cosY;

    // Desired world horizontal velocity
    const desVx = fwdX * ctx.desiredSpeed;
    const desVz = fwdZ * ctx.desiredSpeed;

    for (const id of LEG_IDS) {
      const leg = this.legs[id];
      const mount = UNIT_D10.mounts[id];
      const hipW = ctx.hipWorldPositions[id];

      // Compute nominal hip footprint in world XZ (including lateral track offset!)
      const nomX = mount.isFront ? UNIT_D10.envelope.wheelbase * 0.5 : -UNIT_D10.envelope.wheelbase * 0.5;
      const nomZ = mount.side * UNIT_D10.envelope.trackWidth * 0.5;

      // Also blend with actual moving hip world XZ (which shifts as spine bends!)
      const idealHipX = ctx.bodyPos.x + nomX * fwdX + nomZ * rightX;
      const idealHipZ = ctx.bodyPos.z + nomX * fwdZ + nomZ * rightZ;
      const hipRefX = 0.65 * idealHipX + 0.35 * (hipW.x + mount.side * 0.05 * rightX);
      const hipRefZ = 0.65 * idealHipZ + 0.35 * (hipW.z + mount.side * 0.05 * rightZ);

      // Rotational velocity of this hip due to yawRate: v_rot = ω × r (ω = (0, yawRate, 0))
      const rx = hipRefX - ctx.bodyPos.x;
      const rz = hipRefZ - ctx.bodyPos.z;
      const rotVx = ctx.desiredYawRate * rz;
      const rotVz = -ctx.desiredYawRate * rx;

      // Raibert Foot Placement Heuristic:
      // p_target = p_hip + 0.5 * T_stance * (v_body + v_rot) + k_v * (v_body - v_des)
      const kv = ctx.gait === 'gallop' ? 0.08 : 0.11;
      const raibertGain = ctx.gait === 'gallop' ? 0.58 : 0.52;
      let targetX =
        hipRefX +
        raibertGain * stanceDuration * (ctx.bodyVel.x + rotVx) +
        kv * (ctx.bodyVel.x - desVx);
      let targetZ =
        hipRefZ +
        raibertGain * stanceDuration * (ctx.bodyVel.z + rotVz) +
        kv * (ctx.bodyVel.z - desVz);

      // Clamp maximum stride reach from hip so IK never hyper-extends
      const maxStride = ctx.gait === 'gallop' ? 0.36 : 0.28;
      const dX = targetX - hipRefX;
      const dZ = targetZ - hipRefZ;
      const dHoriz = Math.hypot(dX, dZ);
      if (dHoriz > maxStride) {
        const s = maxStride / dHoriz;
        targetX = hipRefX + dX * s;
        targetZ = hipRefZ + dZ * s;
      }

      const targetY = ctx.terrain.getHeight(targetX, targetZ);
      leg.targetWorld = { x: targetX, y: targetY, z: targetZ };

      // Check phase & stance/swing scheduling
      const prevScheduled = leg.scheduledContact;
      const legPhase = (this.masterPhase + this.blendedOffsets[id]) % 1.0;
      leg.phase = legPhase;

      // In idle mode, keep all 4 feet planted unless a foot drifted far away (recovery step!)
      let isScheduledStance: boolean;
      if (ctx.gait === 'idle') {
        const drift = Math.hypot(leg.footWorld.x - idealHipX, leg.footWorld.z - idealHipZ);
        if (drift > 0.19 && !prevScheduled) {
          isScheduledStance = false;
        } else if (drift > 0.24 && legPhase > 0.65) {
          isScheduledStance = false;
        } else {
          isScheduledStance = true;
        }
      } else {
        isScheduledStance = legPhase < this.blendedDuty;
      }

      leg.scheduledContact = isScheduledStance;

      const prevFootX = leg.footWorld.x;
      const prevFootY = leg.footWorld.y;
      const prevFootZ = leg.footWorld.z;

      if (isScheduledStance) {
        leg.subPhase = clamp(legPhase / Math.max(0.05, this.blendedDuty), 0, 1);
        // Stance paw is planted on terrain (zero slip, clamped to terrain height)
        const groundY = ctx.terrain.getHeight(leg.footWorld.x, leg.footWorld.z);
        leg.terrainHeight = groundY;

        // If body jumped/dropped far above paw during gallop flight transition, keep paw within leg reach
        const maxLegLen = mount.isFront ? 0.68 : 0.65;
        const hipToFootDist = Math.hypot(
          leg.footWorld.x - hipW.x,
          groundY - hipW.y,
          leg.footWorld.z - hipW.z
        );

        if (hipToFootDist <= maxLegLen) {
          leg.footWorld.y = groundY;
          leg.inContact = true;
          leg.clearance = 0;
        } else {
          // Body is airborne (e.g., high drop test or ballistic peak) — dangle paw toward ground
          const dirX = (leg.footWorld.x - hipW.x) / hipToFootDist;
          const dirY = (groundY - hipW.y) / hipToFootDist;
          const dirZ = (leg.footWorld.z - hipW.z) / hipToFootDist;
          leg.footWorld.x = hipW.x + dirX * maxLegLen;
          leg.footWorld.y = Math.max(groundY, hipW.y + dirY * maxLegLen);
          leg.footWorld.z = hipW.z + dirZ * maxLegLen;
          leg.inContact = leg.footWorld.y <= groundY + 0.015;
          leg.clearance = Math.max(0, leg.footWorld.y - groundY);
        }
      } else {
        // Swing phase!
        if (prevScheduled) {
          // Just lifted off — record liftoff anchor
          leg.liftOffWorld = { ...leg.footWorld };
        }
        const swingSpan = Math.max(0.05, 1.0 - this.blendedDuty);
        const u = clamp((legPhase - this.blendedDuty) / swingSpan, 0, 1);
        leg.subPhase = u;

        const sH = smooth5(u);
        const fx = leg.liftOffWorld.x + (targetX - leg.liftOffWorld.x) * sH;
        const fz = leg.liftOffWorld.z + (targetZ - leg.liftOffWorld.z) * sH;

        const baseGroundY = ctx.terrain.getHeight(fx, fz);
        const linearY = leg.liftOffWorld.y + (targetY - leg.liftOffWorld.y) * sH;
        const minGroundClear = Math.max(linearY, baseGroundY);

        // Adapt step height to obstacles/hills + gait preset
        const stepClearance = spec.stepHeight * (ctx.gait === 'idle' ? 0.0 : 1.0) + (ctx.gait === 'idle' ? 0.07 : 0);
        const arcY = stepClearance * swingHeightCurve(u);
        const fy = minGroundClear + arcY;

        leg.footWorld = { x: fx, y: fy, z: fz };
        leg.terrainHeight = baseGroundY;
        leg.clearance = Math.max(0, fy - baseGroundY);
        // Paw makes early physical touchdown if terrain rises sharply into swing path near landing
        leg.inContact = u > 0.85 && fy <= baseGroundY + 0.01;
      }

      leg.footVelWorld = {
        x: (leg.footWorld.x - prevFootX) / ctx.dt,
        y: (leg.footWorld.y - prevFootY) / ctx.dt,
        z: (leg.footWorld.z - prevFootZ) / ctx.dt,
      };
    }
  }
}
