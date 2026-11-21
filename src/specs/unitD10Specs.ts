// Unit D-10 Electromagnetic Dog — Exact Specifications from DWG NO. D10-GA-002 (SI units: meters, kg, radians, N·m)

export type LegId = 'FL' | 'FR' | 'RL' | 'RR';
export type GaitMode = 'idle' | 'walk' | 'trot' | 'gallop';
export type TerrainMode = 'flat' | 'hills' | 'obstacles';
export type CameraPreset = 'orbit' | 'side' | 'front' | 'top' | 'turret';

export const LEG_IDS: readonly LegId[] = ['FL', 'FR', 'RL', 'RR'] as const;

export const DEG2RAD = Math.PI / 180;
export const RAD2DEG = 180 / Math.PI;

export const UNIT_D10 = {
  // Physics & Mass Properties (Sheet 02/03)
  physicsHz: 240,
  dt: 1 / 240,
  gravity: 9.81,
  mass: 55.0, // kg total dry mass
  inertia: {
    Ixx: 2.4, // Roll inertia (kg·m²)
    Iyy: 6.2, // Pitch inertia (kg·m²) -> Note: in our body frame +X fwd, +Y up, +Z right:
              // Roll is around X (Ixx = 2.4), Pitch is around Z (Izz_pitch = 6.2), Yaw is around Y (Iyy_yaw = 7.1)
    Izz: 7.1, // Yaw inertia (kg·m²)
  },

  // Overall Envelope (meters)
  envelope: {
    overallLength: 1.51,
    maxWidth: 0.468,
    overallHeight: 1.313,
    wheelbase: 0.585,
    trackWidth: 0.364,
    nominalShoulderHeight: 0.68,
    nominalHipHeight: 0.625,
    nominalElbowHeight: 0.435,
    nominalComHeight: 0.665,
  },

  // Forelimb (FL / FR) Kinematic Links (meters)
  forelimb: {
    L1: 0.287, // Humerus
    L2: 0.359, // Radius / Ulna
    wristToPad: 0.095, // Wrist to Pad Sole
    maxReach: 0.646,
    lateralOffset: 0.05, // Abduction pivot to limb sagittal plane
  },

  // Hindlimb (RL / RR) Kinematic Links + 4-Bar Pantograph (meters)
  hindlimb: {
    L1: 0.19, // Femur
    L2: 0.298, // Tibia / Crus
    L3: 0.146, // Metatarsus (coupled via 4-bar pantograph)
    ankleToPad: 0.068, // Ankle to Pad Sole
    lateralOffset: 0.05, // Abduction pivot to limb sagittal plane
    pantographOffsetRad: -8 * DEG2RAD, // Fixed kinematic phase offset between femur and metatarsus
    pantographCrankLen: 0.048, // 4-bar crank arm length for visual linkage
  },

  // Spine & Turret (meters & radians)
  spine: {
    lumbarSpan: 0.182, // Total lumbar span (half front, half rear)
    halfSpan: 0.091,
    pitchLimit: 16 * DEG2RAD,
    yawLimit: 15 * DEG2RAD,
    peakTorque: 45, // N·m Harmonic Drive
  },

  turret: {
    azimuthLimit: 148 * DEG2RAD,
    elevationMin: -24 * DEG2RAD,
    elevationMax: 34 * DEG2RAD,
    restElevation: 17.5 * DEG2RAD,
    recoilStroke: 0.125, // 125 mm linear recoil stroke
    peakTorque: 35, // N·m Slew Ring + Servo
    // Mount offset in thorax frame relative to spine front pivot
    mountOffset: { x: 0.04, y: 0.25, z: 0 },
    recoilImpulseNs: 58, // Electromagnetic sabot recoil impulse (~42g @ 1380 m/s = 58 N·s)
    cooldownSec: 0.55,
  },

  // Actuator Limits (ROM & Peak Torque)
  actuators: {
    abduction: {
      peakTorque: 65, // N·m QDD Brushless (9:1)
      min: -23 * DEG2RAD,
      max: 23 * DEG2RAD,
    },
    hipPitch: {
      peakTorque: 110, // N·m QDD Brushless (10:1)
      min: -74 * DEG2RAD,
      max: 74 * DEG2RAD,
    },
    elbowFlexion: {
      peakTorque: 95, // N·m QDD + HTD-5M Belt
      min: -90 * DEG2RAD,
      max: 85 * DEG2RAD,
    },
    hockPantograph: {
      min: -45 * DEG2RAD,
      max: 82 * DEG2RAD,
    },
    neckTail: {
      peakTorqueNeck: 18,
      peakTorqueTail: 12,
    },
  },

  // Hip/Shoulder Mount Offsets relative to Front Thorax pivot (+0.091, 0, 0) and Rear Pelvis pivot (-0.091, 0, 0)
  // Neutral world-relative to CoM: Front X = +0.2925, Y = +0.015; Rear X = -0.2925, Y = -0.040; Z = ±0.132 (pivot) + ±0.050 (link offset) = ±0.182 (364mm track)
  mounts: {
    FL: { segment: 'front' as const, side: -1 as const, localX: 0.2015, localY: 0.015, localZ: -0.132, isFront: true },
    FR: { segment: 'front' as const, side: 1 as const, localX: 0.2015, localY: 0.015, localZ: 0.132, isFront: true },
    RL: { segment: 'rear' as const, side: -1 as const, localX: -0.2015, localY: -0.04, localZ: -0.132, isFront: false },
    RR: { segment: 'rear' as const, side: 1 as const, localX: -0.2015, localY: -0.04, localZ: 0.132, isFront: false },
  },

  // Ground Contact & Paw Load Cells
  contact: {
    frictionCoeff: 0.85,
    maxNormalForce: 1150, // N peak dynamic impact per paw during gallop landings
    loadCellNominal: 200, // 4x 200N Paw Load Cells (rated nominal static)
    pawRadius: 0.028,
  },
} as const;

export interface GaitSpec {
  mode: GaitMode;
  label: string;
  frequencyHz: number;
  dutyFactor: number; // Fraction of cycle spent in stance [0..1]
  phaseOffsets: Record<LegId, number>; // [0..1)
  nominalSpeed: number; // m/s
  minSpeed: number;
  maxSpeed: number;
  stepHeight: number; // m
  bodyHeight: number; // m
  spinePitchAmp: number; // rad (dynamic flexion/extension)
  spineYawAmp: number; // rad (counter-yaw sway)
  spinePhaseOffset: number; // [0..1)
}

export const GAIT_SPECS: Record<GaitMode, GaitSpec> = {
  idle: {
    mode: 'idle',
    label: 'IDLE STANCE',
    frequencyHz: 0.5,
    dutyFactor: 1.0,
    phaseOffsets: { FL: 0, FR: 0, RL: 0, RR: 0 },
    nominalSpeed: 0.0,
    minSpeed: 0.0,
    maxSpeed: 0.0,
    stepHeight: 0.0,
    bodyHeight: 0.665,
    spinePitchAmp: 0.01,
    spineYawAmp: 0.0,
    spinePhaseOffset: 0,
  },
  walk: {
    mode: 'walk',
    label: '4-BEAT WALK',
    frequencyHz: 1.15,
    dutyFactor: 0.74,
    // Lateral sequence canine walk: RL -> FL -> RR -> FR
    phaseOffsets: { RL: 0.0, FL: 0.25, RR: 0.5, FR: 0.75 },
    nominalSpeed: 0.85,
    minSpeed: 0.35,
    maxSpeed: 1.4,
    stepHeight: 0.085,
    bodyHeight: 0.662,
    spinePitchAmp: 0.025,
    spineYawAmp: 0.065,
    spinePhaseOffset: 0.0,
  },
  trot: {
    mode: 'trot',
    label: 'DIAGONAL TROT',
    frequencyHz: 2.05,
    dutyFactor: 0.52,
    // Diagonal pairs: (FL + RR) vs (FR + RL)
    phaseOffsets: { FL: 0.0, RR: 0.0, FR: 0.5, RL: 0.5 },
    nominalSpeed: 2.1,
    minSpeed: 1.0,
    maxSpeed: 3.1,
    stepHeight: 0.12,
    bodyHeight: 0.652,
    spinePitchAmp: 0.045,
    spineYawAmp: 0.095,
    spinePhaseOffset: 0.0,
  },
  gallop: {
    mode: 'gallop',
    label: 'ROTARY GALLOP',
    frequencyHz: 2.65,
    dutyFactor: 0.34,
    // Asymmetric transverse/rotary gallop with full ballistic aerial suspension phase!
    // Hind pair pushes off (RL 0.00, RR 0.13), aerial gathering, fore pair strikes (FL 0.48, FR 0.61), extended suspension
    phaseOffsets: { RL: 0.0, RR: 0.13, FL: 0.48, FR: 0.61 },
    nominalSpeed: 4.1,
    minSpeed: 2.6,
    maxSpeed: 5.5,
    stepHeight: 0.165,
    bodyHeight: 0.642,
    spinePitchAmp: 0.22, // Strong ~12.6° rhythmic lumbar flexion/extension!
    spineYawAmp: 0.055,
    spinePhaseOffset: 0.22,
  },
};
