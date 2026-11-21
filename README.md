# Unit D-10 · 240Hz Three.js WebGPU Biomechanics Engine

**Live Demo**: [https://unit-d10-biomechanics-engine.vercel.app](https://unit-d10-biomechanics-engine.vercel.app)

An interactive **240Hz quadruped biomechanics and rigid-body physics engine** built with **Three.js WebGPU (`three/webgpu` + `three/tsl`)** and TypeScript, modeled strictly after the **Unit D-10 Electromagnetic Dog** blueprint (`DWG NO. D10-GA-002`).

## Key Capabilities

- **240Hz 6-DOF Rigid-Body Physics (`src/physics/RigidBodyEngine.ts`)**: Fixed-timestep (`dt = 1/240 s`) Newton-Euler integrator using Unit D-10's exact dry mass (`55.0 kg`) and inertia tensor (`Ixx = 2.4 kg·m²`, `Iyy = 6.2 kg·m²`, `Izz = 7.1 kg·m²`), quaternion attitude integration, and gyroscopic coupling.
- **4-Leg Ground Reaction Forces (GRF) & Whole-Body Control**: Regularized 6×6 dual least-squares wrench allocator distributing body force/torque across active stance feet under unilateral normal-force constraints (`Fn >= 0`), Coulomb friction cones (`μ = 0.85`), and Hunt-Crossley touchdown impact compliance.
- **Exact 3D Analytical Inverse Kinematics (`src/kinematics/QuadrupedIK.ts`)**:
  - **Forelimbs (FL/FR)**: 3-DOF abduction (`±23°`) + rearward-elbow 2-link IK (`L1 = 287 mm` humerus, `L2 = 359 mm` radius/ulna, `95 mm` wrist-to-pad).
  - **Hindlimbs (RL/RR)**: True 3-link digitigrade leg (`L1 = 190 mm` femur, `L2 = 298 mm` tibia, `L3 = 146 mm` metatarsus, `68 mm` ankle-to-pad) with an exact closed-form **4-bar coupled hock pantograph** linkage and analytical Jacobian-transpose joint torque calculation.
- **Walk / Trot / Rotary Gallop Gait Engine (`src/locomotion/GaitController.ts`)**: Smooth phase-blended transitions across Idle Stance, 4-Beat Lateral Walk, Diagonal Trot, and Rotary Gallop (including genuine **ballistic aerial suspension phases** where all 4 paw GRFs drop to `0 N`), using Raibert foot placement and terrain-adaptive quintic swing trajectories across **Flat**, **Rolling Hills**, and **Step Obstacle** proving grounds.
- **Dynamic 2-DOF Lumbar Spine (`src/biomechanics/SpineSolver.ts`)**: `182 mm` lumbar span harmonic-drive dynamics (`±16°` pitch, `±15°` yaw, `45 N·m` peak torque) flexing rhythmically during gallop, curving into turns, conforming to terrain slope, and reacting to weapon recoil.
- **Recoil-Responsive Railgun, Head, Ears & 5-Segment Tail (`src/biomechanics/SecondaryDynamics.ts`)**: IMU-stabilized dorsal electromagnetic railgun (`±148°` az, `-24°..+34°` el, `17.5°` rest, `125 mm` linear recoil stroke) injecting real recoil momentum and pitch/yaw torque into the chassis, driving vestibulo-ocular head blast-flinch, acoustic ear pin-back, and 5-segment inertial tail counter-whip.
- **Three.js WebGPU & TSL Telemetry Visualization (`src/render/`, `src/rig/`, `src/ui/`)**: `WebGPURenderer` (with automatic WebGL2 node backend fallback), TSL procedural proving-ground grid/contour shader, animated plasma railgun core, 3D GRF vectors, support polygon + CoM/CoP markers, X-ray IK skeleton, 5 blueprint camera presets (`ORBIT`, `SIDE A`, `FRONT B`, `TOP C`, `TURRET`), and a live 240Hz 4-leg GRF oscilloscope.

## Quick Start

```bash
npm install
npm run dev
```

### Verification & Production Build

```bash
npm test      # Runs headless 240Hz physics, 3D IK, gait, and recoil verification suite
npm run build # Runs TypeScript typecheck and Vite production bundle
```

## Controls

| Input | Action |
| --- | --- |
| `1` / `2` / `3` / `4` | Switch gait: Idle / Walk / Trot / Gallop |
| `W` / `S` | Increase / decrease forward velocity |
| `A` / `D` | Steer yaw left / right |
| `Space` | Fire dorsal electromagnetic railgun |
| `P` | Apply lateral perturbation shove |
| `C` | Cycle camera views (Orbit / Side A / Front B / Top C / Turret) |
| `X` | Toggle X-Ray kinematic skeleton overlay |
| `R` | Reset simulation pose |
