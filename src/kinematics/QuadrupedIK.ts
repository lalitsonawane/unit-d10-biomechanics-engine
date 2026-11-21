import { UNIT_D10, type LegId } from '../specs/unitD10Specs';
import type { Vec3 } from '../terrain/TerrainField';

export interface LegIKSolution {
  legId: LegId;
  isFront: boolean;
  /** Shoulder / Hip abduction angle (rad, rotation around local +X) */
  abduction: number;
  /** Shoulder / Hip pitch angle (rad, in leg sagittal plane, 0 = straight down -Y, + = forward +X) */
  hipPitch: number;
  /** Elbow / Stifle flexion angle (rad, relative bend of L2 from L1) */
  kneePitch: number;
  /** Hind hock angle (rad, coupled via 4-bar pantograph; 0 for forelimbs) */
  hockPitch: number;
  /** Joint positions in the leg's mount local frame (mount origin = 0,0,0) */
  shoulderJoint: Vec3;
  hipOffsetJoint: Vec3;
  elbowOrStifle: Vec3;
  wristOrHock: Vec3;
  ankle: Vec3;
  pawSole: Vec3;
  /** 4-bar pantograph pushrod endpoints in mount local frame (hindlimbs only) */
  pantographRodStart: Vec3;
  pantographRodEnd: Vec3;
  /** True if target exceeded physical workspace and was clamped */
  clamped: boolean;
  /** Reach ratio [0..1] */
  reachRatio: number;
  /** Analytical joint torques (N·m) computed via J^T * (-F_grf) */
  torques: {
    abduction: number;
    hipPitch: number;
    kneePitch: number;
  };
}

function clamp(v: number, min: number, max: number): number {
  return v < min ? min : v > max ? max : v;
}

/**
 * Rotate a 2D vector (x, y) in the leg sagittal plane and transform by abduction angle
 * around the mount +X axis into the mount's local 3D frame.
 * Convention:
 * - Local frame at mount: +X forward, +Y up, +Z right.
 * - Side = -1 for left legs (FL, RL), +1 for right legs (FR, RR).
 * - Abduction angle θ_abd rotates the leg plane around +X:
 *     y_local =  y_plane * cos(θ_abd) - z_plane * sin(θ_abd)
 *     z_local =  y_plane * sin(θ_abd) + z_plane * cos(θ_abd)
 *   where z_plane = side * lateralOffset.
 */
function planeToMountLocal(
  xPlane: number,
  yPlane: number,
  zPlane: number,
  abd: number
): Vec3 {
  const c = Math.cos(abd);
  const s = Math.sin(abd);
  return {
    x: xPlane,
    y: yPlane * c - zPlane * s,
    z: yPlane * s + zPlane * c,
  };
}

/**
 * Exact 3D Analytical Inverse Kinematics Solver for Unit D-10.
 *
 * Forelimbs (FL/FR):
 *   - 3-DOF: Abduction (±23°), Shoulder Pitch (±74°), Elbow Flexion (-90°..+85°)
 *   - Rearward-pointing canine elbow (L1 Humerus = 287mm, L2 Radius/Ulna = 359mm, Wrist-to-Pad = 95mm)
 *
 * Hindlimbs (RL/RR):
 *   - 3-DOF + 4-Bar Coupled Hock Pantograph
 *   - L1 Femur = 190mm (angles down-forward to stifle), L2 Tibia = 298mm (angles down-back to hock),
 *     L3 Metatarsus = 146mm (coupled parallel-ish to femur via 4-bar pantograph rod), Ankle-to-Pad = 68mm.
 *   - Because θ_meta = θ_femur + δ0, V_femur + V_meta = L_eff * u(θ_femur + φ0), reducing the 3-link
 *     digitigrade chain to an exact closed-form analytical solve!
 */
export function solveLegIK(
  legId: LegId,
  targetLocal: Vec3,
  grfLocal: Vec3 = { x: 0, y: 0, z: 0 }
): LegIKSolution {
  const mount = UNIT_D10.mounts[legId];
  const side = mount.side;
  const isFront = mount.isFront;
  const latOffset = isFront ? UNIT_D10.forelimb.lateralOffset : UNIT_D10.hindlimb.lateralOffset;
  const padDrop = isFront ? UNIT_D10.forelimb.wristToPad : UNIT_D10.hindlimb.ankleToPad;

  // Target wrist/ankle center sits `padDrop` above the paw sole in local +Y
  const tx = targetLocal.x;
  const ty = targetLocal.y + padDrop;
  const tz = targetLocal.z;

  // 1. Solve Abduction angle in the YZ plane accounting for rigid lateral offset `zOff = side * latOffset`
  // We want:
  //   ty = yPlane * cos(abd) - zOff * sin(abd)
  //   tz = yPlane * sin(abd) + zOff * cos(abd)
  // Notice ty^2 + tz^2 = yPlane^2 + zOff^2 -> yPlane = -sqrt(max(eps, ty^2 + tz^2 - zOff^2)) (negative because leg hangs down)
  const zOff = side * latOffset;
  const rYZ2 = ty * ty + tz * tz;
  const yPlaneRaw = -Math.sqrt(Math.max(0.0025, rYZ2 - zOff * zOff));

  // Solve for abd:
  //   cos(abd) = (ty * yPlaneRaw + tz * zOff) / (yPlaneRaw^2 + zOff^2)
  //   sin(abd) = (tz * yPlaneRaw - ty * zOff) / (yPlaneRaw^2 + zOff^2)
  const denom = yPlaneRaw * yPlaneRaw + zOff * zOff;
  const cosAbd = (ty * yPlaneRaw + tz * zOff) / denom;
  const sinAbd = (tz * yPlaneRaw - ty * zOff) / denom;
  const abdRaw = Math.atan2(sinAbd, cosAbd);
  const abduction = clamp(abdRaw, UNIT_D10.actuators.abduction.min, UNIT_D10.actuators.abduction.max);

  // Project target into the actual clamped abduction sagittal plane
  const cA = Math.cos(abduction);
  const sA = Math.sin(abduction);
  const px = tx;
  const py = ty * cA + tz * sA; // negative when below hip/shoulder

  // Vector from hip/shoulder in sagittal plane (px forward, py up -> downDist = -py)
  const down = -py;
  let reach = Math.hypot(px, down);
  let clamped = Math.abs(abdRaw - abduction) > 1e-4;

  const shoulderJoint: Vec3 = { x: 0, y: 0, z: 0 };
  const hipOffsetJoint = planeToMountLocal(0, 0, zOff, abduction);

  if (isFront) {
    const L1 = UNIT_D10.forelimb.L1; // 0.287 m
    const L2 = UNIT_D10.forelimb.L2; // 0.359 m
    const minReach = Math.abs(L2 - L1) + 0.04;
    const maxReach = (L1 + L2) * 0.992;

    if (reach < minReach) {
      reach = minReach;
      clamped = true;
    } else if (reach > maxReach) {
      reach = maxReach;
      clamped = true;
    }

    // Angle of target vector from straight-down (-Y), positive toward +X (forward)
    const targetAngle = Math.atan2(px, Math.max(0.02, down));

    // Law of cosines for triangle (L1, L2, reach)
    const cosAlpha = clamp((L1 * L1 + reach * reach - L2 * L2) / (2 * L1 * reach), -1, 1);
    const alpha = Math.acos(cosAlpha);

    // Forelimb has REARWARD-pointing elbow (matching canine anatomy & Blueprint View A):
    // Humerus angles BACKWARD from target line: θ1 = targetAngle - alpha
    const hipPitchRaw = targetAngle - alpha;
    const hipPitch = clamp(hipPitchRaw, UNIT_D10.actuators.hipPitch.min, UNIT_D10.actuators.hipPitch.max);

    const cosKnee = clamp((L1 * L1 + L2 * L2 - reach * reach) / (2 * L1 * L2), -1, 1);
    // Internal angle at elbow is acos(cosKnee); bend forward from humerus is +(π - acos(cosKnee))
    const elbowBendRaw = Math.PI - Math.acos(cosKnee);
    // Note: forelimb elbow bends FORWARD relative to humerus line (positive flexion)
    const kneePitch = clamp(elbowBendRaw, UNIT_D10.actuators.elbowFlexion.min, UNIT_D10.actuators.elbowFlexion.max);

    if (Math.abs(hipPitch - hipPitchRaw) > 1e-4 || Math.abs(kneePitch - elbowBendRaw) > 1e-4) {
      clamped = true;
    }

    // Forward kinematics in leg sagittal plane (angle measured from -Y toward +X)
    const elbowX = L1 * Math.sin(hipPitch);
    const elbowY = -L1 * Math.cos(hipPitch);

    const lowerAngle = hipPitch + kneePitch;
    const wristX = elbowX + L2 * Math.sin(lowerAngle);
    const wristY = elbowY - L2 * Math.cos(lowerAngle);

    const elbowOrStifle = planeToMountLocal(elbowX, elbowY, zOff, abduction);
    const wristOrHock = planeToMountLocal(wristX, wristY, zOff, abduction);
    const ankle: Vec3 = { x: wristOrHock.x, y: wristOrHock.y, z: wristOrHock.z };
    const pawSole: Vec3 = {
      x: wristOrHock.x,
      y: wristOrHock.y - padDrop,
      z: wristOrHock.z,
    };

    // Analytical Jacobian transpose torques: τ = J^T * (-F_grf)
    // Foot force on robot is F_grf, so robot applies -F_grf to ground -> torque required = -(r × F_grf)
    const torques = computeForelimbTorques(
      abduction,
      hipPitch,
      lowerAngle,
      L1,
      L2,
      zOff,
      pawSole,
      grfLocal
    );

    return {
      legId,
      isFront: true,
      abduction,
      hipPitch,
      kneePitch,
      hockPitch: 0,
      shoulderJoint,
      hipOffsetJoint,
      elbowOrStifle,
      wristOrHock,
      ankle,
      pawSole,
      pantographRodStart: elbowOrStifle,
      pantographRodEnd: wristOrHock,
      clamped,
      reachRatio: reach / (L1 + L2),
      torques,
    };
  } else {
    // HINDLIMB (RL / RR): 3-link digitigrade with exact 4-bar coupled pantograph!
    const L1 = UNIT_D10.hindlimb.L1; // 0.190 m Femur
    const L2 = UNIT_D10.hindlimb.L2; // 0.298 m Tibia/Crus
    const L3 = UNIT_D10.hindlimb.L3; // 0.146 m Metatarsus
    const delta0 = UNIT_D10.hindlimb.pantographOffsetRad; // -8 deg

    // Effective combined parallel vector V1 + V3 = L1 * u(θ1) + L3 * u(θ1 + δ0) = L_comb * u(θ1 + φ0)
    const combX = L1 + L3 * Math.cos(delta0);
    const combY = L3 * Math.sin(delta0);
    const Lcomb = Math.hypot(combX, combY);
    const phi0 = Math.atan2(combY, combX);

    const minReach = Math.abs(Lcomb - L2) + 0.04;
    const maxReach = (Lcomb + L2) * 0.992;

    if (reach < minReach) {
      reach = minReach;
      clamped = true;
    } else if (reach > maxReach) {
      reach = maxReach;
      clamped = true;
    }

    const targetAngle = Math.atan2(px, Math.max(0.02, down));
    const cosAlpha = clamp((Lcomb * Lcomb + reach * reach - L2 * L2) / (2 * Lcomb * reach), -1, 1);
    const alpha = Math.acos(cosAlpha);

    // Hindlimb has FORWARD-pointing stifle/knee and REARWARD-pointing hock (Blueprint View A):
    // Combined Femur+Metatarsus angles FORWARD from target line: θ_comb = targetAngle + alpha
    const thetaComb = targetAngle + alpha;
    const hipPitchRaw = thetaComb - phi0;
    const hipPitch = clamp(hipPitchRaw, UNIT_D10.actuators.hipPitch.min, UNIT_D10.actuators.hipPitch.max);

    const cosInternal = clamp((Lcomb * Lcomb + L2 * L2 - reach * reach) / (2 * Lcomb * L2), -1, 1);
    // Tibia bends BACKWARD relative to Femur (negative angle delta from θ_comb)
    const tibiaAbsAngle = thetaComb - (Math.PI - Math.acos(cosInternal));
    const stifleFlexRaw = tibiaAbsAngle - hipPitch; // negative (backward bend)
    const kneePitch = clamp(stifleFlexRaw, UNIT_D10.actuators.elbowFlexion.min, UNIT_D10.actuators.elbowFlexion.max);

    const actualTibiaAngle = hipPitch + kneePitch;
    // Coupled 4-bar pantograph keeps metatarsus at θ_meta = hipPitch + delta0
    const metaAngle = hipPitch + delta0;
    const hockPitch = clamp(
      metaAngle - actualTibiaAngle,
      UNIT_D10.actuators.hockPantograph.min,
      UNIT_D10.actuators.hockPantograph.max
    );
    const actualMetaAngle = actualTibiaAngle + hockPitch;

    if (Math.abs(hipPitch - hipPitchRaw) > 1e-4 || Math.abs(kneePitch - stifleFlexRaw) > 1e-4) {
      clamped = true;
    }

    // Forward kinematics in sagittal plane
    const stifleX = L1 * Math.sin(hipPitch);
    const stifleY = -L1 * Math.cos(hipPitch);

    const hockX = stifleX + L2 * Math.sin(actualTibiaAngle);
    const hockY = stifleY - L2 * Math.cos(actualTibiaAngle);

    const ankleX = hockX + L3 * Math.sin(actualMetaAngle);
    const ankleY = hockY - L3 * Math.cos(actualMetaAngle);

    // Exact 4-bar pantograph pushrod endpoints (parallel to tibia, offset forward-above stifle & hock)
    const crank = UNIT_D10.hindlimb.pantographCrankLen;
    // Lug A attached rigidly to femur near stifle (offset forward along femur+delta0 normal)
    const lugAx = stifleX - crank * Math.sin(hipPitch + delta0);
    const lugAy = stifleY + crank * Math.cos(hipPitch + delta0);
    // Lug B attached rigidly to metatarsus extending above hock (exact parallelogram!)
    const lugBx = hockX - crank * Math.sin(actualMetaAngle);
    const lugBy = hockY + crank * Math.cos(actualMetaAngle);

    const elbowOrStifle = planeToMountLocal(stifleX, stifleY, zOff, abduction);
    const wristOrHock = planeToMountLocal(hockX, hockY, zOff, abduction);
    const ankle = planeToMountLocal(ankleX, ankleY, zOff, abduction);
    const pawSole: Vec3 = {
      x: ankle.x,
      y: ankle.y - padDrop,
      z: ankle.z,
    };

    const pantographRodStart = planeToMountLocal(lugAx, lugAy, zOff + side * 0.016, abduction);
    const pantographRodEnd = planeToMountLocal(lugBx, lugBy, zOff + side * 0.016, abduction);

    const torques = computeHindlimbTorques(
      abduction,
      hipPitch,
      actualTibiaAngle,
      actualMetaAngle,
      L1,
      L2,
      L3,
      pawSole,
      grfLocal
    );

    return {
      legId,
      isFront: false,
      abduction,
      hipPitch,
      kneePitch,
      hockPitch,
      shoulderJoint,
      hipOffsetJoint,
      elbowOrStifle,
      wristOrHock,
      ankle,
      pawSole,
      pantographRodStart,
      pantographRodEnd,
      clamped,
      reachRatio: reach / (Lcomb + L2),
      torques,
    };
  }
}

function computeForelimbTorques(
  abd: number,
  q1: number,
  q12: number,
  L1: number,
  L2: number,
  _zOff: number,
  paw: Vec3,
  F: Vec3
): { abduction: number; hipPitch: number; kneePitch: number } {
  // Moment around X axis (abduction) = -(r_y * F_z - r_z * F_y)
  const tauAbd = -(paw.y * F.z - paw.z * F.y);

  // Rotate force into sagittal plane
  const cA = Math.cos(abd);
  const sA = Math.sin(abd);
  const Fx = F.x;
  const Fy = F.y * cA + F.z * sA;

  // Foot position in sagittal plane:
  // x = L1 sin(q1) + L2 sin(q12)
  // y = -L1 cos(q1) - L2 cos(q12)
  // dx/dq1 = L1 cos(q1) + L2 cos(q12), dy/dq1 = L1 sin(q1) + L2 sin(q12)
  // dx/dq2 = L2 cos(q12),               dy/dq2 = L2 sin(q12)
  const dxDq1 = L1 * Math.cos(q1) + L2 * Math.cos(q12);
  const dyDq1 = L1 * Math.sin(q1) + L2 * Math.sin(q12);
  const dxDq2 = L2 * Math.cos(q12);
  const dyDq2 = L2 * Math.sin(q12);

  return {
    abduction: clamp(tauAbd, -UNIT_D10.actuators.abduction.peakTorque, UNIT_D10.actuators.abduction.peakTorque),
    hipPitch: clamp(-(dxDq1 * Fx + dyDq1 * Fy), -UNIT_D10.actuators.hipPitch.peakTorque, UNIT_D10.actuators.hipPitch.peakTorque),
    kneePitch: clamp(-(dxDq2 * Fx + dyDq2 * Fy), -UNIT_D10.actuators.elbowFlexion.peakTorque, UNIT_D10.actuators.elbowFlexion.peakTorque),
  };
}

function computeHindlimbTorques(
  abd: number,
  q1: number,
  qTibia: number,
  qMeta: number,
  L1: number,
  L2: number,
  L3: number,
  paw: Vec3,
  F: Vec3
): { abduction: number; hipPitch: number; kneePitch: number } {
  const tauAbd = -(paw.y * F.z - paw.z * F.y);

  const cA = Math.cos(abd);
  const sA = Math.sin(abd);
  const Fx = F.x;
  const Fy = F.y * cA + F.z * sA;

  // Because pantograph couples qMeta = q1 + delta0 (independent of q2!),
  // dx/dq1 = L1 cos(q1) + L2 cos(qTibia) + L3 cos(qMeta)
  // dy/dq1 = L1 sin(q1) + L2 sin(qTibia) + L3 sin(qMeta)
  // dx/dq2 = L2 cos(qTibia)
  // dy/dq2 = L2 sin(qTibia)
  const dxDq1 = L1 * Math.cos(q1) + L2 * Math.cos(qTibia) + L3 * Math.cos(qMeta);
  const dyDq1 = L1 * Math.sin(q1) + L2 * Math.sin(qTibia) + L3 * Math.sin(qMeta);
  const dxDq2 = L2 * Math.cos(qTibia);
  const dyDq2 = L2 * Math.sin(qTibia);

  return {
    abduction: clamp(tauAbd, -UNIT_D10.actuators.abduction.peakTorque, UNIT_D10.actuators.abduction.peakTorque),
    hipPitch: clamp(-(dxDq1 * Fx + dyDq1 * Fy), -UNIT_D10.actuators.hipPitch.peakTorque, UNIT_D10.actuators.hipPitch.peakTorque),
    kneePitch: clamp(-(dxDq2 * Fx + dyDq2 * Fy), -UNIT_D10.actuators.elbowFlexion.peakTorque, UNIT_D10.actuators.elbowFlexion.peakTorque),
  };
}
