import { BiomechanicsSimulation } from '../engine/BiomechanicsSimulation';
import { solveLegIK } from '../kinematics/QuadrupedIK';
import { LEG_IDS, RAD2DEG, UNIT_D10 } from '../specs/unitD10Specs';

function assert(cond: boolean, msg: string): void {
  if (!cond) {
    throw new Error(`ASSERTION FAILED: ${msg}`);
  }
}

console.log('=== UNIT D-10 BIOMECHANICS ENGINE VERIFICATION ===\n');

// 1. Verify Exact 3D IK Accuracy (Forelimb 2-link & Hindlimb 3-link 4-bar Pantograph)
console.log('[1] Testing 3D Analytical IK Solver...');
const ikTestCases = [
  { id: 'FL' as const, target: { x: 0.0, y: -0.68, z: -0.05 } },
  { id: 'FR' as const, target: { x: 0.14, y: -0.61, z: 0.08 } },
  { id: 'FL' as const, target: { x: -0.15, y: -0.58, z: -0.09 } },
  { id: 'RL' as const, target: { x: 0.0, y: -0.625, z: -0.05 } },
  { id: 'RR' as const, target: { x: 0.12, y: -0.56, z: 0.09 } },
  { id: 'RL' as const, target: { x: -0.14, y: -0.54, z: -0.07 } },
];

for (const tc of ikTestCases) {
  const sol = solveLegIK(tc.id, tc.target, { x: 20, y: 180, z: -10 });
  const errMm =
    Math.hypot(
      sol.pawSole.x - tc.target.x,
      sol.pawSole.y - tc.target.y,
      sol.pawSole.z - tc.target.z
    ) * 1000;
  console.log(
    `  ${tc.id} target=(${tc.target.x.toFixed(3)}, ${tc.target.y.toFixed(3)}, ${tc.target.z.toFixed(3)}) -> err=${errMm.toFixed(3)} mm, clamped=${sol.clamped}, abd=${(sol.abduction * RAD2DEG).toFixed(1)}°, hip=${(sol.hipPitch * RAD2DEG).toFixed(1)}°, knee=${(sol.kneePitch * RAD2DEG).toFixed(1)}°`
  );
  assert(errMm < 1.5, `IK error too large on ${tc.id}: ${errMm} mm`);

  if (!sol.isFront) {
    // Verify 4-bar pantograph pushrod length matches Tibia L2 (exact parallelogram!)
    const rodLen = Math.hypot(
      sol.pantographRodEnd.x - sol.pantographRodStart.x,
      sol.pantographRodEnd.y - sol.pantographRodStart.y,
      sol.pantographRodEnd.z - sol.pantographRodStart.z
    );
    const rodErrMm = Math.abs(rodLen - UNIT_D10.hindlimb.L2) * 1000;
    assert(rodErrMm < 0.5, `4-bar pantograph rod length drifted: ${rodErrMm.toFixed(3)} mm`);
  }
}

// 2. Verify 240Hz Idle Equilibrium (Total GRF == mg = 539.55 N, Height == 0.665 m)
console.log('\n[2] Testing 240Hz Idle Equilibrium & GRF Balance...');
const sim = new BiomechanicsSimulation();
sim.setGait('idle');
for (let i = 0; i < 480; i++) sim.stepOnce(); // 2.0 seconds at 240Hz
const idleSnap = sim.getSnapshot();
const mg = UNIT_D10.mass * UNIT_D10.gravity;
console.log(
  `  Idle 2.0s: height=${idleSnap.body.pos.y.toFixed(4)}m, totalGRF=${idleSnap.body.totalVerticalGRF.toFixed(1)}N (mg=${mg.toFixed(1)}N), roll=${(idleSnap.body.euler.roll * RAD2DEG).toFixed(2)}°, pitch=${(idleSnap.body.euler.pitch * RAD2DEG).toFixed(2)}°`
);
assert(Math.abs(idleSnap.body.totalVerticalGRF - mg) < 15, 'Idle GRF must match mg');
assert(Math.abs(idleSnap.body.pos.y - 0.665) < 0.02, 'Idle CoM height must match 0.665m');
assert(Math.abs(idleSnap.body.euler.pitch * RAD2DEG) < 1.5, 'Idle pitch must be level');
assert(Math.abs(idleSnap.body.euler.roll * RAD2DEG) < 1.5, 'Idle roll must be level');

// 3. Verify Railgun Recoil Response (Chassis Pitch Kick, 125mm Stroke, Head Flinch, Ear Pin-Back, Tail Whip)
console.log('\n[3] Testing Railgun Recoil Impulse & Head/Ear/Tail Response...');
const fired = sim.fireRailgun();
assert(fired, 'Railgun should fire when charged');
let maxRecoilMm = 0;
let minHeadPitchDeg = 0;
let minEarPitchDeg = 0;
let maxTailPitchVel = 0;
let maxBodyPitchKickDeg = 0;

for (let i = 0; i < 240; i++) {
  const snap = sim.stepOnce();
  maxRecoilMm = Math.max(maxRecoilMm, snap.secondary.recoilDisp * 1000);
  minHeadPitchDeg = Math.min(minHeadPitchDeg, snap.secondary.neckPitch * RAD2DEG);
  minEarPitchDeg = Math.min(minEarPitchDeg, snap.secondary.earLeftPitch * RAD2DEG);
  maxTailPitchVel = Math.max(maxTailPitchVel, Math.abs(snap.secondary.tailPitchVel[4]));
  maxBodyPitchKickDeg = Math.max(maxBodyPitchKickDeg, Math.abs(snap.body.euler.pitch * RAD2DEG));
}
console.log(
  `  Recoil peak: barrel=${maxRecoilMm.toFixed(1)}mm (max 125mm), bodyPitchKick=${maxBodyPitchKickDeg.toFixed(2)}°, headFlinch=${minHeadPitchDeg.toFixed(1)}°, earPin=${minEarPitchDeg.toFixed(1)}°, tailTipVel=${maxTailPitchVel.toFixed(1)} rad/s`
);
assert(maxRecoilMm > 60 && maxRecoilMm <= 125.1, `Expected recoil stroke 60..125mm, got ${maxRecoilMm}`);
assert(minHeadPitchDeg < -8, `Expected noticeable head flinch, got ${minHeadPitchDeg}`);
assert(minEarPitchDeg < -18, `Expected noticeable ear pin-back, got ${minEarPitchDeg}`);
assert(maxTailPitchVel > 8, `Expected dynamic tail counter-whip, got ${maxTailPitchVel}`);
assert(maxBodyPitchKickDeg > 1.0, `Expected real rigid-body recoil pitch response, got ${maxBodyPitchKickDeg}`);

// 4. Verify Walk, Trot, and Gallop Locomotion (including Aerial Phase & Dynamic Spine Bending)
for (const gait of ['walk', 'trot', 'gallop'] as const) {
  console.log(`\n[4] Testing ${gait.toUpperCase()} Locomotion (3.0s @ 240Hz)...`);
  sim.reset();
  sim.setGait(gait);
  let sumGRF = 0;
  let airborneTicks = 0;
  let maxSpinePitchDeg = 0;
  let minSpinePitchDeg = 0;
  let maxSpineYawDeg = 0;
  let maxHeight = 0;
  let minHeight = 99;

  // Warm up 0.5s, measure 2.5s
  for (let i = 0; i < 120; i++) sim.stepOnce();
  const startX = sim.getSnapshot().body.pos.x;

  const measureTicks = 600;
  for (let i = 0; i < measureTicks; i++) {
    const snap = sim.stepOnce();
    sumGRF += snap.body.totalVerticalGRF;
    if (snap.body.isAirborne) airborneTicks++;
    const spDeg = snap.spine.pitch * RAD2DEG;
    const syDeg = snap.spine.yaw * RAD2DEG;
    maxSpinePitchDeg = Math.max(maxSpinePitchDeg, spDeg);
    minSpinePitchDeg = Math.min(minSpinePitchDeg, spDeg);
    maxSpineYawDeg = Math.max(maxSpineYawDeg, Math.abs(syDeg));
    maxHeight = Math.max(maxHeight, snap.body.pos.y);
    minHeight = Math.min(minHeight, snap.body.pos.y);
    assert(Number.isFinite(snap.body.pos.x) && Number.isFinite(snap.body.pos.y), `${gait} produced NaN!`);
  }

  const endSnap = sim.getSnapshot();
  const avgSpeed = (endSnap.body.pos.x - startX) / (measureTicks / 240);
  const avgGRF = sumGRF / measureTicks;
  console.log(
    `  ${gait}: avgSpeed=${avgSpeed.toFixed(2)} m/s, avgGRF=${avgGRF.toFixed(1)}N (mg=${mg.toFixed(1)}N), airborne=${airborneTicks}/${measureTicks} ticks, height=[${minHeight.toFixed(3)}..${maxHeight.toFixed(3)}]m, spinePitch=[${minSpinePitchDeg.toFixed(1)}°..${maxSpinePitchDeg.toFixed(1)}°], spineYawMax=${maxSpineYawDeg.toFixed(1)}°`
  );

  assert(avgSpeed > 0.3, `${gait} should make forward progress, got ${avgSpeed}`);
  assert(Math.abs(avgGRF - mg) < 95, `${gait} average vertical GRF should approximate mg, got ${avgGRF}`);
  assert(minHeight > 0.48 && maxHeight < 0.85, `${gait} body height out of stable envelope: [${minHeight}, ${maxHeight}]`);
  assert(Math.abs(maxSpinePitchDeg) <= 16.05 && Math.abs(minSpinePitchDeg) <= 16.05, 'Spine pitch exceeded ±16° ROM');
  if (gait === 'gallop') {
    assert(airborneTicks > 15, `Gallop must exhibit ballistic aerial suspension phase, got ${airborneTicks} ticks`);
    assert(maxSpinePitchDeg - minSpinePitchDeg > 10, 'Gallop must exhibit strong dynamic lumbar spine flexion/extension');
  }
}

// 5. Verify Obstacle & Hills Terrain Navigation
console.log('\n[5] Testing Hills & Obstacle Terrain Stability (Trot)...');
for (const terr of ['hills', 'obstacles'] as const) {
  sim.setTerrain(terr);
  sim.reset();
  sim.setGait('trot');
  for (let i = 0; i < 720; i++) {
    const snap = sim.stepOnce();
    assert(Number.isFinite(snap.body.pos.y), `${terr} produced NaN`);
  }
  const s = sim.getSnapshot();
  const clearance = s.body.pos.y - s.body.comGroundWorld.y;
  console.log(
    `  ${terr}: traveled x=${s.body.pos.x.toFixed(2)}m, groundClearance=${clearance.toFixed(3)}m, pitch=${(s.body.euler.pitch * RAD2DEG).toFixed(1)}°, roll=${(s.body.euler.roll * RAD2DEG).toFixed(1)}°`
  );
  assert(s.body.pos.x > 3.5, `Should traverse ${terr} terrain smoothly`);
  assert(clearance > 0.45 && clearance < 0.85, `Should maintain body height over ${terr}`);
}

console.log('\nALL BIOMECHANICS ENGINE TESTS PASSED!');
