import * as THREE from 'three/webgpu';
import { color, float, mix, sin, time, uniform } from 'three/tsl';
import { LEG_IDS, UNIT_D10, type LegId } from '../specs/unitD10Specs';
import type { SimulationSnapshot } from '../engine/BiomechanicsSimulation';
import { TAIL_SEGMENTS } from '../biomechanics/SecondaryDynamics';
import type { Vec3 } from '../terrain/TerrainField';

interface ForelimbRig {
  root: THREE.Group;
  abductionDrum: THREE.Mesh;
  upperStrut: THREE.Group;
  elbowDrum: THREE.Mesh;
  lowerStrut: THREE.Group;
  wristJoint: THREE.Group;
  pawGroup: THREE.Group;
  pawLedMat: THREE.MeshStandardNodeMaterial;
  pawLoadUniform: THREE.UniformNode<'float', number>;
}

interface HindlimbRig {
  root: THREE.Group;
  abductionDrum: THREE.Mesh;
  femurStrut: THREE.Group;
  stifleDrum: THREE.Mesh;
  tibiaStrut: THREE.Group;
  hockDrum: THREE.Mesh;
  metaStrut: THREE.Group;
  pantographRod: THREE.Group;
  pawGroup: THREE.Group;
  pawLedMat: THREE.MeshStandardNodeMaterial;
  pawLoadUniform: THREE.UniformNode<'float', number>;
}

const _vA = new THREE.Vector3();
const _vB = new THREE.Vector3();
const _vDir = new THREE.Vector3();
const _vMid = new THREE.Vector3();
const _upY = new THREE.Vector3(0, 1, 0);
const _qAlign = new THREE.Quaternion();

/** Position and orient a Y-axis-aligned strut group between local points A and B */
function orientStrutBetween(group: THREE.Object3D, a: Vec3, b: Vec3): number {
  _vA.set(a.x, a.y, a.z);
  _vB.set(b.x, b.y, b.z);
  _vDir.subVectors(_vB, _vA);
  const len = _vDir.length();
  _vMid.addVectors(_vA, _vB).multiplyScalar(0.5);
  group.position.copy(_vMid);
  if (len > 1e-5) {
    _vDir.divideScalar(len);
    _qAlign.setFromUnitVectors(_upY, _vDir);
    group.quaternion.copy(_qAlign);
  }
  return len;
}

/**
 * High-detail procedural CAD model of Unit D-10 Electromagnetic Dog
 * matching Blueprint DWG NO. D10-GA-002 dimensions & visual silhouette.
 */
export class UnitD10Model {
  public readonly root = new THREE.Group();
  public readonly bodyCenterGroup = new THREE.Group();
  public readonly frontThorax = new THREE.Group();
  public readonly rearPelvis = new THREE.Group();
  public readonly lumbarBridge = new THREE.Group();

  // Secondary articulated assemblies
  private readonly neckPitchGroup = new THREE.Group();
  private readonly headGroup = new THREE.Group();
  private readonly earLeftPivot = new THREE.Group();
  private readonly earRightPivot = new THREE.Group();
  private readonly tailPivots: THREE.Group[] = [];

  // Dorsal Railgun Turret
  private readonly turretAzimuthGroup = new THREE.Group();
  private readonly turretElevationGroup = new THREE.Group();
  private readonly railRecoilSled = new THREE.Group();
  private readonly muzzleFlashMesh: THREE.Mesh;
  private readonly muzzleRingMesh: THREE.Mesh;
  private readonly muzzleLight: THREE.PointLight;

  // Limb rigs
  private readonly foreRigs: Record<'FL' | 'FR', ForelimbRig>;
  private readonly hindRigs: Record<'RL' | 'RR', HindlimbRig>;

  // TSL dynamic uniforms for WebGPU shader glow
  private readonly chargeUniform = uniform(1.0);
  private readonly flashUniform = uniform(0.0);

  // Materials
  private readonly matArmor: THREE.MeshStandardNodeMaterial;
  private readonly matArmorDark: THREE.MeshStandardNodeMaterial;
  private readonly matChassis: THREE.MeshStandardNodeMaterial;
  private readonly matSteel: THREE.MeshStandardNodeMaterial;
  private readonly matCopper: THREE.MeshStandardNodeMaterial;
  private readonly matRubber: THREE.MeshStandardNodeMaterial;
  private readonly matAccentOrange: THREE.MeshStandardNodeMaterial;
  private readonly matPlasmaCore: THREE.MeshStandardNodeMaterial;
  private readonly matOpticLens: THREE.MeshStandardNodeMaterial;
  private readonly matBatteryLed: THREE.MeshStandardNodeMaterial;
  private readonly solidMeshes: THREE.Mesh[] = [];

  constructor() {
    this.root.name = 'UnitD10';

    // Shared NodeMaterials (WebGPU + WebGL2 compatible)
    this.matArmor = new THREE.MeshStandardNodeMaterial({
      color: 0xd9dde3,
      roughness: 0.36,
      metalness: 0.18,
    });
    this.matArmorDark = new THREE.MeshStandardNodeMaterial({
      color: 0x3b4452,
      roughness: 0.44,
      metalness: 0.48,
    });
    this.matChassis = new THREE.MeshStandardNodeMaterial({
      color: 0x202630,
      roughness: 0.34,
      metalness: 0.78,
    });
    this.matSteel = new THREE.MeshStandardNodeMaterial({
      color: 0x7a8596,
      roughness: 0.24,
      metalness: 0.88,
    });
    this.matCopper = new THREE.MeshStandardNodeMaterial({
      color: 0xc87a38,
      roughness: 0.28,
      metalness: 0.9,
    });
    this.matRubber = new THREE.MeshStandardNodeMaterial({
      color: 0x16191e,
      roughness: 0.88,
      metalness: 0.05,
    });
    this.matAccentOrange = new THREE.MeshStandardNodeMaterial({
      color: 0xf59e0b,
      roughness: 0.35,
      metalness: 0.2,
    });

    // TSL Animated Railgun Plasma Core material (pulses as supercap charges, flashes cyan-white on fire)
    this.matPlasmaCore = new THREE.MeshStandardNodeMaterial({
      color: 0x0b1929,
      roughness: 0.18,
      metalness: 0.4,
    });
    const pulse = sin(time.mul(6.0)).mul(0.15).add(0.85);
    const chargeGlow = color(0x0ea5e9).mul(this.chargeUniform.mul(2.4).mul(pulse));
    const blastGlow = color(0xe0f2fe).mul(this.flashUniform.mul(14.0));
    this.matPlasmaCore.emissiveNode = chargeGlow.add(blastGlow);

    this.matOpticLens = new THREE.MeshStandardNodeMaterial({
      color: 0x091620,
      roughness: 0.08,
      metalness: 0.9,
    });
    this.matOpticLens.emissiveNode = color(0x38bdf8).mul(float(1.6));

    this.matBatteryLed = new THREE.MeshStandardNodeMaterial({
      color: 0x064e3b,
      roughness: 0.2,
      metalness: 0.2,
    });
    this.matBatteryLed.emissiveNode = mix(
      color(0xf59e0b),
      color(0x10b981),
      this.chargeUniform
    ).mul(float(1.8));

    // Attach top-level structural groups
    this.root.add(this.bodyCenterGroup);
    this.root.add(this.frontThorax);
    this.root.add(this.rearPelvis);
    this.bodyCenterGroup.add(this.lumbarBridge);

    // Build CAD sub-assemblies
    this.buildLumbarSpine();
    this.buildFrontThorax();
    this.buildRearPelvis();
    this.buildHeadAndEars();
    this.buildTail();

    const { flashMesh, ringMesh, flashLight } = this.buildRailgunTurret();
    this.muzzleFlashMesh = flashMesh;
    this.muzzleRingMesh = ringMesh;
    this.muzzleLight = flashLight;

    // Build 4 Kinematic Limbs
    this.foreRigs = {
      FL: this.buildForelimb('FL'),
      FR: this.buildForelimb('FR'),
    };
    this.hindRigs = {
      RL: this.buildHindlimb('RL'),
      RR: this.buildHindlimb('RR'),
    };

    // Enable shadow casting & collect solid meshes for X-ray toggle
    this.root.traverse((obj) => {
      if (obj instanceof THREE.Mesh) {
        obj.castShadow = true;
        obj.receiveShadow = true;
        this.solidMeshes.push(obj);
      }
    });
  }

  public setXrayMode(enabled: boolean): void {
    const opacity = enabled ? 0.22 : 1.0;
    const mats = [
      this.matArmor,
      this.matArmorDark,
      this.matChassis,
      this.matSteel,
      this.matCopper,
      this.matRubber,
      this.matAccentOrange,
    ];
    for (const m of mats) {
      m.transparent = enabled;
      m.opacity = opacity;
      m.depthWrite = !enabled;
      m.needsUpdate = true;
    }
  }

  private buildLumbarSpine(): void {
    // 182mm Harmonic Drive lumbar core at CoM center
    const coreCyl = new THREE.Mesh(
      new THREE.CylinderGeometry(0.068, 0.068, 0.175, 20),
      this.matChassis
    );
    coreCyl.rotation.z = Math.PI * 0.5;
    this.lumbarBridge.add(coreCyl);

    // Harmonic drive machined ring
    const ring = new THREE.Mesh(
      new THREE.CylinderGeometry(0.082, 0.082, 0.052, 24),
      this.matSteel
    );
    ring.rotation.z = Math.PI * 0.5;
    this.lumbarBridge.add(ring);

    // Upper & lower articulated bellows ribs
    for (let i = -2; i <= 2; i++) {
      const rib = new THREE.Mesh(
        new THREE.BoxGeometry(0.014, 0.145, 0.165),
        i % 2 === 0 ? this.matArmorDark : this.matChassis
      );
      rib.position.set(i * 0.032, 0.01, 0);
      this.lumbarBridge.add(rib);
    }

    // Twin side hydraulic / flex cable conduits along lumbar spine
    for (const side of [-1, 1]) {
      const conduit = new THREE.Mesh(
        new THREE.CylinderGeometry(0.013, 0.013, 0.21, 10),
        this.matCopper
      );
      conduit.rotation.z = Math.PI * 0.5;
      conduit.position.set(0, -0.05, side * 0.095);
      this.lumbarBridge.add(conduit);

      // Orange-White CoM Marker Disc (Exact match to Blueprint View A!)
      const comBase = new THREE.Mesh(
        new THREE.CylinderGeometry(0.024, 0.024, 0.008, 16),
        this.matArmor
      );
      comBase.rotation.x = Math.PI * 0.5;
      comBase.position.set(0.01, 0.01, side * 0.105);
      this.lumbarBridge.add(comBase);

      const comQ1 = new THREE.Mesh(
        new THREE.CylinderGeometry(0.0245, 0.0245, 0.009, 12, 1, false, 0, Math.PI * 0.5),
        this.matAccentOrange
      );
      comQ1.rotation.x = Math.PI * 0.5;
      comQ1.position.set(0.01, 0.01, side * 0.105);
      this.lumbarBridge.add(comQ1);

      const comQ3 = new THREE.Mesh(
        new THREE.CylinderGeometry(0.0245, 0.0245, 0.009, 12, 1, false, Math.PI, Math.PI * 0.5),
        this.matAccentOrange
      );
      comQ3.rotation.x = Math.PI * 0.5;
      comQ3.position.set(0.01, 0.01, side * 0.105);
      this.lumbarBridge.add(comQ3);
    }
  }

  private buildFrontThorax(): void {
    // Front Thorax sits from lumbar front pivot (x=0 in frontThorax) forward to chest (+0.34m)
    // Main armored chest block
    const mainChest = new THREE.Mesh(
      new THREE.BoxGeometry(0.32, 0.22, 0.22),
      this.matArmor
    );
    mainChest.position.set(0.15, 0.03, 0);
    this.frontThorax.add(mainChest);

    // Sloped upper dorsal cowl
    const upperCowl = new THREE.Mesh(
      new THREE.BoxGeometry(0.28, 0.075, 0.19),
      this.matArmor
    );
    upperCowl.position.set(0.14, 0.155, 0);
    upperCowl.rotation.z = -0.18;
    this.frontThorax.add(upperCowl);

    // Angled front sternum armor nose (canine chest profile from View A!)
    const sternum = new THREE.Mesh(
      new THREE.BoxGeometry(0.14, 0.19, 0.185),
      this.matArmor
    );
    sternum.position.set(0.31, 0.01, 0);
    sternum.rotation.z = 0.42;
    this.frontThorax.add(sternum);

    // Underside armored belly keel
    const keel = new THREE.Mesh(
      new THREE.BoxGeometry(0.29, 0.07, 0.16),
      this.matChassis
    );
    keel.position.set(0.16, -0.095, 0);
    keel.rotation.z = 0.24;
    this.frontThorax.add(keel);

    // Side shoulder fairings with angled cooling louvers (exact detail from View A!)
    for (const side of [-1, 1]) {
      const fairing = new THREE.Mesh(
        new THREE.BoxGeometry(0.24, 0.17, 0.045),
        this.matArmor
      );
      fairing.position.set(0.18, 0.02, side * 0.12);
      fairing.rotation.z = -0.35;
      this.frontThorax.add(fairing);

      // Vent recess
      const ventPlate = new THREE.Mesh(
        new THREE.BoxGeometry(0.11, 0.07, 0.015),
        this.matChassis
      );
      ventPlate.position.set(0.19, 0.035, side * 0.141);
      ventPlate.rotation.z = -0.35;
      this.frontThorax.add(ventPlate);

      // 4 diagonal cooling fins inside vent
      for (let v = 0; v < 4; v++) {
        const fin = new THREE.Mesh(
          new THREE.BoxGeometry(0.012, 0.058, 0.012),
          this.matSteel
        );
        fin.position.set(0.155 + v * 0.022, 0.048 - v * 0.008, side * 0.146);
        fin.rotation.z = -0.35;
        this.frontThorax.add(fin);
      }

      // Shoulder QDD actuator mount boss
      const mountBoss = new THREE.Mesh(
        new THREE.CylinderGeometry(0.062, 0.062, 0.055, 20),
        this.matArmorDark
      );
      mountBoss.rotation.x = Math.PI * 0.5;
      mountBoss.position.set(
        UNIT_D10.mounts.FL.localX,
        UNIT_D10.mounts.FL.localY,
        side * 0.125
      );
      this.frontThorax.add(mountBoss);
    }

    // Jetson Orin NX + Dual STM32H7 Compute Deck plate on dorsal spine
    const computeDeck = new THREE.Mesh(
      new THREE.BoxGeometry(0.19, 0.05, 0.17),
      this.matArmorDark
    );
    computeDeck.position.set(0.04, 0.175, 0);
    this.frontThorax.add(computeDeck);
  }

  private buildRearPelvis(): void {
    // Rear pelvis chassis from lumbar rear pivot (x=0 in rearPelvis) back to rump (-0.34m)
    const pelvisCore = new THREE.Mesh(
      new THREE.BoxGeometry(0.32, 0.16, 0.21),
      this.matArmorDark
    );
    pelvisCore.position.set(-0.16, 0.0, 0);
    this.rearPelvis.add(pelvisCore);

    const dorsalDeck = new THREE.Mesh(
      new THREE.BoxGeometry(0.30, 0.055, 0.20),
      this.matArmor
    );
    dorsalDeck.position.set(-0.16, 0.095, 0);
    this.rearPelvis.add(dorsalDeck);

    // Dual 14S6P 21700 Li-Ion Battery Panniers (signature visual feature on Views A, B, C!)
    // Max width across panniers = 468 mm (each pannier outer face at ±0.234 m!)
    for (const side of [-1, 1]) {
      const pannier = new THREE.Group();
      pannier.position.set(-0.20, 0.045, side * 0.178);

      // Main armored battery enclosure (0.285m L x 0.165m H x 0.112m W -> outer edge at 0.178 + 0.056 = 0.234m -> 468mm total!)
      const box = new THREE.Mesh(
        new THREE.BoxGeometry(0.285, 0.165, 0.112),
        this.matArmor
      );
      pannier.add(box);

      // Top/bottom bumper rails
      const topRail = new THREE.Mesh(
        new THREE.BoxGeometry(0.292, 0.018, 0.118),
        this.matChassis
      );
      topRail.position.y = 0.082;
      pannier.add(topRail);

      const botRail = new THREE.Mesh(
        new THREE.BoxGeometry(0.292, 0.018, 0.118),
        this.matChassis
      );
      botRail.position.y = -0.082;
      pannier.add(botRail);

      // 4 signature angled heat-sink louver ribs on outer face (exact match to View A!)
      for (let r = 0; r < 4; r++) {
        const rib = new THREE.Mesh(
          new THREE.BoxGeometry(0.018, 0.125, 0.014),
          this.matArmorDark
        );
        rib.position.set(-0.08 + r * 0.045, 0, side * 0.057);
        rib.rotation.z = 0.38;
        pannier.add(rib);
      }

      // Forward recessed BMS telemetry bay & glowing charge bar
      const bmsBay = new THREE.Mesh(
        new THREE.BoxGeometry(0.052, 0.115, 0.014),
        this.matChassis
      );
      bmsBay.position.set(0.105, 0, side * 0.055);
      pannier.add(bmsBay);

      const ledBar = new THREE.Mesh(
        new THREE.BoxGeometry(0.012, 0.075, 0.008),
        this.matBatteryLed
      );
      ledBar.position.set(0.105, 0, side * 0.062);
      pannier.add(ledBar);

      // HV power umbilical exiting front of pannier into chassis
      const hvCable = new THREE.Mesh(
        new THREE.CylinderGeometry(0.011, 0.011, 0.12, 8),
        this.matAccentOrange
      );
      hvCable.rotation.z = Math.PI * 0.42;
      hvCable.rotation.y = side * 0.35;
      hvCable.position.set(0.17, 0.02, -side * 0.02);
      pannier.add(hvCable);

      this.rearPelvis.add(pannier);

      // Rear hip QDD actuator mount boss
      const hipBoss = new THREE.Mesh(
        new THREE.CylinderGeometry(0.058, 0.058, 0.055, 20),
        this.matArmorDark
      );
      hipBoss.rotation.x = Math.PI * 0.5;
      hipBoss.position.set(
        UNIT_D10.mounts.RL.localX,
        UNIT_D10.mounts.RL.localY,
        side * 0.125
      );
      this.rearPelvis.add(hipBoss);
    }
  }

  private buildHeadAndEars(): void {
    // Articulated neck attaches to upper front chest
    this.neckPitchGroup.position.set(0.30, 0.12, 0);
    this.frontThorax.add(this.neckPitchGroup);

    // Neck cycloidal actuator & armored neck link
    const neckDrum = new THREE.Mesh(
      new THREE.CylinderGeometry(0.044, 0.044, 0.10, 16),
      this.matSteel
    );
    neckDrum.rotation.x = Math.PI * 0.5;
    this.neckPitchGroup.add(neckDrum);

    const neckLink = new THREE.Mesh(
      new THREE.BoxGeometry(0.13, 0.085, 0.09),
      this.matArmorDark
    );
    neckLink.position.set(0.06, 0.04, 0);
    neckLink.rotation.z = 0.42;
    this.neckPitchGroup.add(neckLink);

    // Head assembly at tip of neck (overall length reaches ~1,510 mm nose-to-tail!)
    this.headGroup.position.set(0.115, 0.075, 0);
    this.neckPitchGroup.add(this.headGroup);

    // Main faceted canine sensor cranium
    const cranium = new THREE.Mesh(
      new THREE.BoxGeometry(0.24, 0.115, 0.125),
      this.matArmor
    );
    cranium.position.set(0.11, 0.01, 0);
    this.headGroup.add(cranium);

    // Snout cheek armor plates
    const snoutTop = new THREE.Mesh(
      new THREE.BoxGeometry(0.19, 0.032, 0.11),
      this.matArmorDark
    );
    snoutTop.position.set(0.13, 0.072, 0);
    this.headGroup.add(snoutTop);

    // Underslung chin lidar / rangefinder lip (visible in View A!)
    const chinLip = new THREE.Mesh(
      new THREE.BoxGeometry(0.14, 0.022, 0.095),
      this.matChassis
    );
    chinLip.position.set(0.175, -0.056, 0);
    this.headGroup.add(chinLip);

    // Dual Vertically Stacked Front Optical Lenses (Signature front elevation detail from View B!)
    const opticFrame = new THREE.Mesh(
      new THREE.BoxGeometry(0.03, 0.105, 0.095),
      this.matChassis
    );
    opticFrame.position.set(0.232, 0.01, 0);
    this.headGroup.add(opticFrame);

    for (const ly of [0.032, -0.014]) {
      const bezel = new THREE.Mesh(
        new THREE.CylinderGeometry(0.024, 0.024, 0.02, 18),
        this.matSteel
      );
      bezel.rotation.z = Math.PI * 0.5;
      bezel.position.set(0.245, ly, 0);
      this.headGroup.add(bezel);

      const lens = new THREE.Mesh(
        new THREE.CylinderGeometry(0.017, 0.017, 0.024, 18),
        this.matOpticLens
      );
      lens.rotation.z = Math.PI * 0.5;
      lens.position.set(0.247, ly, 0);
      this.headGroup.add(lens);
    }

    // Twin Upright Alert Tactical Radar / Acoustic Ears (matching View A silhouette!)
    const buildEar = (pivot: THREE.Group, side: -1 | 1) => {
      pivot.position.set(0.045, 0.07, side * 0.058);
      this.headGroup.add(pivot);

      const base = new THREE.Mesh(
        new THREE.CylinderGeometry(0.016, 0.018, 0.022, 12),
        this.matSteel
      );
      base.rotation.x = Math.PI * 0.5;
      pivot.add(base);

      // Tall tapered ear fin (~175mm tall, matches blueprint proportion!)
      const earFin = new THREE.Mesh(
        new THREE.BoxGeometry(0.046, 0.175, 0.018),
        this.matArmor
      );
      earFin.position.set(0.008, 0.09, 0);
      earFin.rotation.z = 0.12;
      pivot.add(earFin);

      const earInner = new THREE.Mesh(
        new THREE.BoxGeometry(0.022, 0.13, 0.021),
        this.matChassis
      );
      earInner.position.set(0.014, 0.088, 0);
      earInner.rotation.z = 0.12;
      pivot.add(earInner);
    };

    buildEar(this.earLeftPivot, -1);
    buildEar(this.earRightPivot, 1);
  }

  private buildTail(): void {
    // 5-segment articulated braided inertial tail attached to rear rump
    let parent: THREE.Group = this.rearPelvis;
    const segLen = 0.072;

    for (let i = 0; i < TAIL_SEGMENTS; i++) {
      const pivot = new THREE.Group();
      if (i === 0) {
        pivot.position.set(-0.325, 0.045, 0);
      } else {
        pivot.position.set(-segLen, 0, 0);
      }
      parent.add(pivot);
      this.tailPivots.push(pivot);

      const radius = 0.019 * (1 - i * 0.11);
      const segMesh = new THREE.Mesh(
        new THREE.CylinderGeometry(radius * 0.85, radius, segLen * 0.94, 10),
        i % 2 === 0 ? this.matArmorDark : this.matChassis
      );
      segMesh.rotation.z = Math.PI * 0.5;
      segMesh.position.set(-segLen * 0.5, 0, 0);
      pivot.add(segMesh);

      // Articulated joint knuckle
      const joint = new THREE.Mesh(
        new THREE.SphereGeometry(radius * 1.05, 10, 8),
        this.matSteel
      );
      pivot.add(joint);

      if (i === TAIL_SEGMENTS - 1) {
        // Tail tip inertial beacon
        const tip = new THREE.Mesh(
          new THREE.SphereGeometry(0.014, 10, 8),
          this.matAccentOrange
        );
        tip.position.set(-segLen, 0, 0);
        pivot.add(tip);
      }

      parent = pivot;
    }
  }

  private buildRailgunTurret(): {
    flashMesh: THREE.Mesh;
    ringMesh: THREE.Mesh;
    flashLight: THREE.PointLight;
  } {
    // Turret slew ring mounted on dorsal thorax
    this.turretAzimuthGroup.position.set(
      UNIT_D10.turret.mountOffset.x,
      UNIT_D10.turret.mountOffset.y,
      UNIT_D10.turret.mountOffset.z
    );
    this.frontThorax.add(this.turretAzimuthGroup);

    // Slew ring pedestal
    const slewBase = new THREE.Mesh(
      new THREE.CylinderGeometry(0.088, 0.102, 0.055, 24),
      this.matChassis
    );
    slewBase.position.y = -0.025;
    this.turretAzimuthGroup.add(slewBase);

    // Twin trunnion yoke cheeks
    for (const side of [-1, 1]) {
      const cheek = new THREE.Mesh(
        new THREE.BoxGeometry(0.115, 0.115, 0.028),
        this.matArmor
      );
      cheek.position.set(0, 0.055, side * 0.068);
      this.turretAzimuthGroup.add(cheek);

      const pin = new THREE.Mesh(
        new THREE.CylinderGeometry(0.026, 0.026, 0.036, 16),
        this.matSteel
      );
      pin.rotation.x = Math.PI * 0.5;
      pin.position.set(0, 0.082, side * 0.076);
      this.turretAzimuthGroup.add(pin);
    }

    // Elevation trunnion pivot
    this.turretElevationGroup.position.set(0, 0.082, 0);
    this.turretAzimuthGroup.add(this.turretElevationGroup);

    // Stationary recoil cradle & breech guide rails
    const cradle = new THREE.Mesh(
      new THREE.BoxGeometry(0.46, 0.068, 0.105),
      this.matChassis
    );
    cradle.position.set(0.04, 0.02, 0);
    this.turretElevationGroup.add(cradle);

    // Hanging curved braided umbilical power cable loop under barrel (iconic detail from View A!)
    const cableCurve = new THREE.CatmullRomCurve3([
      new THREE.Vector3(-0.04, 0.0, 0),
      new THREE.Vector3(0.08, -0.095, 0),
      new THREE.Vector3(0.22, -0.055, 0),
      new THREE.Vector3(0.26, 0.02, 0),
    ]);
    const cableGeo = new THREE.TubeGeometry(cableCurve, 18, 0.017, 10, false);
    const cableMesh = new THREE.Mesh(cableGeo, this.matRubber);
    this.turretElevationGroup.add(cableMesh);

    // Recoil Sled (slides linearly -0..-125 mm along local -X!)
    this.turretElevationGroup.add(this.railRecoilSled);

    // Upper & lower armored electromagnetic rail shrouds (overall length ~0.88m, matches View A!)
    const upperRailArmor = new THREE.Mesh(
      new THREE.BoxGeometry(0.86, 0.048, 0.092),
      this.matArmor
    );
    upperRailArmor.position.set(0.18, 0.092, 0);
    this.railRecoilSled.add(upperRailArmor);

    const lowerRailArmor = new THREE.Mesh(
      new THREE.BoxGeometry(0.82, 0.042, 0.086),
      this.matArmorDark
    );
    lowerRailArmor.position.set(0.16, 0.028, 0);
    this.railRecoilSled.add(lowerRailArmor);

    // Glowing TSL Plasma Acceleration Channel between the upper & lower rails!
    const plasmaChannel = new THREE.Mesh(
      new THREE.BoxGeometry(0.83, 0.024, 0.072),
      this.matPlasmaCore
    );
    plasmaChannel.position.set(0.18, 0.06, 0);
    this.railRecoilSled.add(plasmaChannel);

    // Copper induction coil ribs along the electromagnetic barrel
    for (let c = 0; c < 7; c++) {
      const coil = new THREE.Mesh(
        new THREE.BoxGeometry(0.026, 0.076, 0.102),
        this.matCopper
      );
      coil.position.set(-0.14 + c * 0.11, 0.062, 0);
      this.railRecoilSled.add(coil);
    }

    // Breech capacitor heatsink fins at rear of turret
    const breech = new THREE.Mesh(
      new THREE.BoxGeometry(0.18, 0.088, 0.112),
      this.matArmor
    );
    breech.position.set(-0.26, 0.06, 0);
    this.railRecoilSled.add(breech);

    // Muzzle brake & twin electromagnetic rail tips
    const muzzleCollar = new THREE.Mesh(
      new THREE.BoxGeometry(0.085, 0.098, 0.106),
      this.matSteel
    );
    muzzleCollar.position.set(0.61, 0.06, 0);
    this.railRecoilSled.add(muzzleCollar);

    // Muzzle plasma flash bloom sphere + point light
    const flashMat = new THREE.MeshBasicNodeMaterial({
      color: 0xbae6fd,
      transparent: true,
      opacity: 0.9,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    });
    const flashMesh = new THREE.Mesh(new THREE.OctahedronGeometry(0.14, 1), flashMat);
    flashMesh.position.set(0.71, 0.06, 0);
    flashMesh.visible = false;
    this.railRecoilSled.add(flashMesh);

    // Expanding electromagnetic muzzle blast ring
    const ringGeo = new THREE.TorusGeometry(0.11, 0.014, 10, 28);
    ringGeo.rotateY(Math.PI * 0.5);
    const ringMat = new THREE.MeshBasicNodeMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.85,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    });
    const ringMesh = new THREE.Mesh(ringGeo, ringMat);
    ringMesh.position.set(0.70, 0.06, 0);
    ringMesh.visible = false;
    this.railRecoilSled.add(ringMesh);

    const flashLight = new THREE.PointLight(0x38bdf8, 0, 7.5, 2.0);
    flashLight.position.set(0.74, 0.06, 0);
    this.railRecoilSled.add(flashLight);

    return { flashMesh, ringMesh, flashLight };
  }

  private createPawVisual(isFront: boolean): {
    pawGroup: THREE.Group;
    pawLedMat: THREE.MeshStandardNodeMaterial;
    pawLoadUniform: THREE.UniformNode<'float', number>;
  } {
    const pawGroup = new THREE.Group();
    const pawLoadUniform = uniform(0.0);

    const pawLedMat = new THREE.MeshStandardNodeMaterial({
      color: 0x0f172a,
      roughness: 0.25,
      metalness: 0.3,
    });
    // Glow shifts from cool cyan (light load) -> amber (nominal) -> crimson (heavy dynamic impact!)
    const lowCol = color(0x38bdf8);
    const midCol = color(0xf59e0b);
    const hiCol = color(0xef4444);
    const loadColor = mix(
      mix(lowCol, midCol, pawLoadUniform.mul(2.0).clamp(0, 1)),
      hiCol,
      pawLoadUniform.sub(0.5).mul(2.0).clamp(0, 1)
    );
    pawLedMat.emissiveNode = loadColor.mul(pawLoadUniform.mul(3.2).add(0.15));

    const padDrop = isFront ? UNIT_D10.forelimb.wristToPad : UNIT_D10.hindlimb.ankleToPad;

    // Articulated wrist/ankle clevis
    const clevis = new THREE.Mesh(
      new THREE.CylinderGeometry(0.022, 0.022, 0.056, 14),
      this.matSteel
    );
    clevis.rotation.x = Math.PI * 0.5;
    clevis.position.y = padDrop;
    pawGroup.add(clevis);

    // Vertical load-cell piston column
    const column = new THREE.Mesh(
      new THREE.BoxGeometry(0.044, padDrop * 0.72, 0.046),
      this.matArmor
    );
    column.position.y = padDrop * 0.56;
    pawGroup.add(column);

    // Glowing load-cell pressure collar
    const sensorRing = new THREE.Mesh(
      new THREE.BoxGeometry(0.05, 0.016, 0.054),
      pawLedMat
    );
    sensorRing.position.y = padDrop * 0.32;
    pawGroup.add(sensorRing);

    // Tactical ribbed rubber paw pad sole sitting flush on ground (y = 0)
    const padSole = new THREE.Mesh(
      new THREE.BoxGeometry(0.088, 0.026, 0.064),
      this.matRubber
    );
    padSole.position.set(0.012, 0.013, 0);
    pawGroup.add(padSole);

    // Front toe bumper
    const toe = new THREE.Mesh(
      new THREE.CylinderGeometry(0.013, 0.013, 0.062, 10),
      this.matRubber
    );
    toe.rotation.x = Math.PI * 0.5;
    toe.position.set(0.054, 0.013, 0);
    pawGroup.add(toe);

    return { pawGroup, pawLedMat, pawLoadUniform };
  }

  private buildStrutVisual(
    length: number,
    widthX: number,
    widthZ: number,
    primaryMat: THREE.Material,
    secondaryMat: THREE.Material
  ): THREE.Group {
    const g = new THREE.Group();
    const core = new THREE.Mesh(
      new THREE.BoxGeometry(widthX, length, widthZ * 0.65),
      secondaryMat
    );
    g.add(core);

    const armorShell = new THREE.Mesh(
      new THREE.BoxGeometry(widthX * 1.15, length * 0.76, widthZ),
      primaryMat
    );
    g.add(armorShell);

    // Side weight-relief machined pocket accent
    const accent = new THREE.Mesh(
      new THREE.BoxGeometry(widthX * 0.45, length * 0.55, widthZ * 1.06),
      this.matChassis
    );
    g.add(accent);
    return g;
  }

  private buildForelimb(legId: 'FL' | 'FR'): ForelimbRig {
    const mount = UNIT_D10.mounts[legId];
    const root = new THREE.Group();
    root.position.set(mount.localX, mount.localY, mount.localZ);
    this.frontThorax.add(root);

    const abductionDrum = new THREE.Mesh(
      new THREE.CylinderGeometry(0.048, 0.048, 0.068, 20),
      this.matSteel
    );
    abductionDrum.rotation.z = Math.PI * 0.5;
    root.add(abductionDrum);

    const upperStrut = this.buildStrutVisual(
      UNIT_D10.forelimb.L1,
      0.068,
      0.056,
      this.matArmor,
      this.matChassis
    );
    root.add(upperStrut);

    const elbowDrum = new THREE.Mesh(
      new THREE.CylinderGeometry(0.042, 0.042, 0.064, 18),
      this.matSteel
    );
    elbowDrum.rotation.x = Math.PI * 0.5;
    root.add(elbowDrum);

    const lowerStrut = this.buildStrutVisual(
      UNIT_D10.forelimb.L2,
      0.052,
      0.046,
      this.matArmor,
      this.matArmorDark
    );
    root.add(lowerStrut);

    const wristJoint = new THREE.Group();
    const wristPin = new THREE.Mesh(
      new THREE.CylinderGeometry(0.024, 0.024, 0.052, 14),
      this.matSteel
    );
    wristPin.rotation.x = Math.PI * 0.5;
    wristJoint.add(wristPin);
    root.add(wristJoint);

    const { pawGroup, pawLedMat, pawLoadUniform } = this.createPawVisual(true);
    root.add(pawGroup);

    return {
      root,
      abductionDrum,
      upperStrut,
      elbowDrum,
      lowerStrut,
      wristJoint,
      pawGroup,
      pawLedMat,
      pawLoadUniform,
    };
  }

  private buildHindlimb(legId: 'RL' | 'RR'): HindlimbRig {
    const mount = UNIT_D10.mounts[legId];
    const root = new THREE.Group();
    root.position.set(mount.localX, mount.localY, mount.localZ);
    this.rearPelvis.add(root);

    const abductionDrum = new THREE.Mesh(
      new THREE.CylinderGeometry(0.048, 0.048, 0.068, 20),
      this.matSteel
    );
    abductionDrum.rotation.z = Math.PI * 0.5;
    root.add(abductionDrum);

    const femurStrut = this.buildStrutVisual(
      UNIT_D10.hindlimb.L1,
      0.07,
      0.056,
      this.matArmor,
      this.matChassis
    );
    root.add(femurStrut);

    const stifleDrum = new THREE.Mesh(
      new THREE.CylinderGeometry(0.042, 0.042, 0.064, 18),
      this.matSteel
    );
    stifleDrum.rotation.x = Math.PI * 0.5;
    root.add(stifleDrum);

    const tibiaStrut = this.buildStrutVisual(
      UNIT_D10.hindlimb.L2,
      0.052,
      0.046,
      this.matArmor,
      this.matArmorDark
    );
    root.add(tibiaStrut);

    const hockDrum = new THREE.Mesh(
      new THREE.CylinderGeometry(0.032, 0.032, 0.054, 16),
      this.matSteel
    );
    hockDrum.rotation.x = Math.PI * 0.5;
    root.add(hockDrum);

    const metaStrut = this.buildStrutVisual(
      UNIT_D10.hindlimb.L3,
      0.044,
      0.042,
      this.matArmor,
      this.matChassis
    );
    root.add(metaStrut);

    // Signature 4-Bar Pantograph Coupled Pushrod (copper/steel turnbuckle rod!)
    const pantographRod = new THREE.Group();
    const rodShaft = new THREE.Mesh(
      new THREE.CylinderGeometry(0.0085, 0.0085, UNIT_D10.hindlimb.L2, 10),
      this.matCopper
    );
    pantographRod.add(rodShaft);
    const turnbuckle = new THREE.Mesh(
      new THREE.CylinderGeometry(0.013, 0.013, 0.065, 8),
      this.matSteel
    );
    pantographRod.add(turnbuckle);
    root.add(pantographRod);

    const { pawGroup, pawLedMat, pawLoadUniform } = this.createPawVisual(false);
    root.add(pawGroup);

    return {
      root,
      abductionDrum,
      femurStrut,
      stifleDrum,
      tibiaStrut,
      hockDrum,
      metaStrut,
      pantographRod,
      pawGroup,
      pawLedMat,
      pawLoadUniform,
    };
  }

  /** Synchronize every articulated bone, spine joint, 4-bar pantograph rod, turret, head, ears & tail to the 240Hz snapshot */
  public sync(snap: SimulationSnapshot): void {
    // 1. Center lumbar core & bent front/rear segments
    this.bodyCenterGroup.position.set(snap.body.pos.x, snap.body.pos.y, snap.body.pos.z);
    this.bodyCenterGroup.quaternion.set(
      snap.body.quat.x,
      snap.body.quat.y,
      snap.body.quat.z,
      snap.body.quat.w
    );

    this.frontThorax.position.set(
      snap.frontSegment.pos.x,
      snap.frontSegment.pos.y,
      snap.frontSegment.pos.z
    );
    this.frontThorax.quaternion.set(
      snap.frontSegment.quat.x,
      snap.frontSegment.quat.y,
      snap.frontSegment.quat.z,
      snap.frontSegment.quat.w
    );

    this.rearPelvis.position.set(
      snap.rearSegment.pos.x,
      snap.rearSegment.pos.y,
      snap.rearSegment.pos.z
    );
    this.rearPelvis.quaternion.set(
      snap.rearSegment.quat.x,
      snap.rearSegment.quat.y,
      snap.rearSegment.quat.z,
      snap.rearSegment.quat.w
    );

    // 2. Dorsal Railgun Turret & 125mm Linear Recoil Stroke
    const sec = snap.secondary;
    this.turretAzimuthGroup.rotation.set(0, sec.turretAzimuth, 0);
    this.turretElevationGroup.rotation.set(0, 0, sec.turretElevation);
    // Slide barrel backward along local -X by exact recoilDisp [0..0.125 m]
    this.railRecoilSled.position.set(-sec.recoilDisp, 0, 0);

    this.chargeUniform.value = sec.capacitorCharge;
    this.flashUniform.value = sec.muzzleFlash;

    if (sec.muzzleFlash > 0.04) {
      this.muzzleFlashMesh.visible = true;
      this.muzzleRingMesh.visible = true;
      const s = 0.6 + sec.muzzleFlash * 1.4;
      this.muzzleFlashMesh.scale.set(s * 1.9, s * 0.85, s * 0.85);
      const expand = 1.0 + (1.0 - sec.muzzleFlash) * 2.6;
      this.muzzleRingMesh.scale.set(1, expand, expand);
      this.muzzleRingMesh.position.set(0.70 + (1.0 - sec.muzzleFlash) * 0.25, 0.06, 0);
      this.muzzleLight.intensity = sec.muzzleFlash * 18.0;
    } else {
      this.muzzleFlashMesh.visible = false;
      this.muzzleRingMesh.visible = false;
      this.muzzleLight.intensity = 0;
    }

    // 3. Recoil-Responsive Head, Neck & Acoustic Ears
    this.neckPitchGroup.rotation.set(0, sec.neckYaw * 0.45, sec.neckPitch * 0.55);
    this.headGroup.rotation.set(sec.headRoll, sec.neckYaw * 0.55, sec.neckPitch * 0.45);
    this.headGroup.position.set(0.115 - sec.headRecoilOffset, 0.075, 0);

    // Ears pin back (+Z tilts forward, -Z tilts backward)
    this.earLeftPivot.rotation.set(-0.08, sec.earSwivel, sec.earLeftPitch);
    this.earRightPivot.rotation.set(0.08, sec.earSwivel, sec.earRightPitch);

    // 4. 5-Segment Coupled Inertial Tail
    // Tail extends along -X: positive Z rotation tilts -X downward, so negate pitch so negative = natural droop!
    for (let i = 0; i < TAIL_SEGMENTS; i++) {
      this.tailPivots[i].rotation.set(0, sec.tailYaw[i], -sec.tailPitch[i]);
    }

    // 5. All 4 Kinematic Limbs + 4-Bar Pantograph Linkage + Paw Load-Cell Glow
    for (const id of LEG_IDS) {
      const sol = snap.ik[id];
      const normLoad = Math.min(1, snap.body.normalForce[id] / 360);

      if (id === 'FL' || id === 'FR') {
        const rig = this.foreRigs[id];
        orientStrutBetween(rig.upperStrut, sol.hipOffsetJoint, sol.elbowOrStifle);
        rig.elbowDrum.position.set(
          sol.elbowOrStifle.x,
          sol.elbowOrStifle.y,
          sol.elbowOrStifle.z
        );
        orientStrutBetween(rig.lowerStrut, sol.elbowOrStifle, sol.wristOrHock);
        rig.wristJoint.position.set(
          sol.wristOrHock.x,
          sol.wristOrHock.y,
          sol.wristOrHock.z
        );
        rig.pawGroup.position.set(sol.pawSole.x, sol.pawSole.y, sol.pawSole.z);
        rig.pawLoadUniform.value = normLoad;
      } else {
        const rig = this.hindRigs[id];
        orientStrutBetween(rig.femurStrut, sol.hipOffsetJoint, sol.elbowOrStifle);
        rig.stifleDrum.position.set(
          sol.elbowOrStifle.x,
          sol.elbowOrStifle.y,
          sol.elbowOrStifle.z
        );
        orientStrutBetween(rig.tibiaStrut, sol.elbowOrStifle, sol.wristOrHock);
        rig.hockDrum.position.set(sol.wristOrHock.x, sol.wristOrHock.y, sol.wristOrHock.z);
        orientStrutBetween(rig.metaStrut, sol.wristOrHock, sol.ankle);
        // Exact 4-bar coupled pantograph pushrod connecting stifle lug to hock crank!
        orientStrutBetween(rig.pantographRod, sol.pantographRodStart, sol.pantographRodEnd);
        rig.pawGroup.position.set(sol.pawSole.x, sol.pawSole.y, sol.pawSole.z);
        rig.pawLoadUniform.value = normLoad;
      }
    }
  }
}
