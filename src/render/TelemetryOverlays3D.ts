import * as THREE from 'three/webgpu';
import { LEG_IDS, UNIT_D10, type LegId } from '../specs/unitD10Specs';
import type { SimulationSnapshot } from '../engine/BiomechanicsSimulation';
import type { Vec3 } from '../terrain/TerrainField';

const _v1 = new THREE.Vector3();
const _v2 = new THREE.Vector3();
const _upY = new THREE.Vector3(0, 1, 0);
const _quat = new THREE.Quaternion();

interface GRFArrowVisual {
  group: THREE.Group;
  shaft: THREE.Mesh;
  head: THREE.Mesh;
  mat: THREE.MeshBasicNodeMaterial;
  ring: THREE.Mesh;
  ringMat: THREE.MeshBasicNodeMaterial;
  targetRing: THREE.Mesh;
}

export interface OverlayVisibility {
  grfVectors: boolean;
  supportPolygon: boolean;
  footTargets: boolean;
  skeleton: boolean;
}

/**
 * Real-Time 3D Biomechanical Visualization Overlays:
 * - Scaled 3D Ground Reaction Force (GRF) arrows & floor pressure rings per paw
 * - Dynamic Support Polygon convex hull + CoM plumb line + Center of Pressure (ZMP) marker
 * - Raibert Footstep landing targets
 * - X-Ray Kinematic Skeleton (spine + all 4 leg link chains + 4-bar pantograph)
 */
export class TelemetryOverlays3D {
  public readonly root = new THREE.Group();
  public visibility: OverlayVisibility = {
    grfVectors: true,
    supportPolygon: true,
    footTargets: true,
    skeleton: false,
  };

  private readonly grfGroup = new THREE.Group();
  private readonly supportGroup = new THREE.Group();
  private readonly targetsGroup = new THREE.Group();
  private readonly skeletonGroup = new THREE.Group();

  private readonly arrows: Record<LegId, GRFArrowVisual>;

  // Support polygon outline & fill
  private readonly supportLineGeo: THREE.BufferGeometry;
  private readonly supportLine: THREE.Line;
  private readonly comPlumbGeo: THREE.BufferGeometry;
  private readonly comPlumbLine: THREE.Line;
  private readonly comGroundDisc: THREE.Mesh;
  private readonly copMarker: THREE.Mesh;

  // Skeleton line segments
  private readonly skelGeo: THREE.BufferGeometry;
  private readonly skelLines: THREE.LineSegments;
  private readonly skelJoints: THREE.Mesh[] = [];

  constructor() {
    this.root.add(this.grfGroup);
    this.root.add(this.supportGroup);
    this.root.add(this.targetsGroup);
    this.root.add(this.skeletonGroup);

    // 1. Per-leg GRF 3D Vector Arrows & Contact Pressure Rings
    const shaftGeo = new THREE.CylinderGeometry(0.011, 0.011, 1.0, 10);
    shaftGeo.translate(0, 0.5, 0);
    const headGeo = new THREE.ConeGeometry(0.028, 0.085, 12);
    headGeo.translate(0, 0.0425, 0);
    const ringGeo = new THREE.RingGeometry(0.042, 0.068, 24);
    ringGeo.rotateX(-Math.PI * 0.5);
    const targetRingGeo = new THREE.RingGeometry(0.032, 0.044, 16);
    targetRingGeo.rotateX(-Math.PI * 0.5);

    const arrows = {} as Record<LegId, GRFArrowVisual>;
    const targetMat = new THREE.MeshBasicNodeMaterial({
      color: 0x38bdf8,
      transparent: true,
      opacity: 0.65,
      side: THREE.DoubleSide,
      depthWrite: false,
    });

    for (const id of LEG_IDS) {
      const mat = new THREE.MeshBasicNodeMaterial({
        color: 0x22d3ee,
        transparent: true,
        opacity: 0.92,
      });
      const group = new THREE.Group();
      const shaft = new THREE.Mesh(shaftGeo, mat);
      const head = new THREE.Mesh(headGeo, mat);
      group.add(shaft, head);
      this.grfGroup.add(group);

      const ringMat = new THREE.MeshBasicNodeMaterial({
        color: 0x22d3ee,
        transparent: true,
        opacity: 0.72,
        side: THREE.DoubleSide,
        depthWrite: false,
      });
      const ring = new THREE.Mesh(ringGeo, ringMat);
      this.grfGroup.add(ring);

      const targetRing = new THREE.Mesh(targetRingGeo, targetMat);
      this.targetsGroup.add(targetRing);

      arrows[id] = { group, shaft, head, mat, ring, ringMat, targetRing };
    }
    this.arrows = arrows;

    // 2. Support Polygon + CoM Ground Projection + ZMP / CoP Indicator
    // WebGPU renderer does not support LineLoop — close the polygon with an extra vertex
    this.supportLineGeo = new THREE.BufferGeometry();
    this.supportLineGeo.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array(5 * 3), 3)
    );
    this.supportLine = new THREE.Line(
      this.supportLineGeo,
      new THREE.LineBasicNodeMaterial({ color: 0x10b981, transparent: true, opacity: 0.85 })
    );
    this.supportGroup.add(this.supportLine);

    this.comPlumbGeo = new THREE.BufferGeometry();
    this.comPlumbGeo.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array(2 * 3), 3)
    );
    this.comPlumbLine = new THREE.Line(
      this.comPlumbGeo,
      new THREE.LineBasicNodeMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.85 })
    );
    this.supportGroup.add(this.comPlumbLine);

    const comDiscGeo = new THREE.RingGeometry(0.025, 0.048, 20);
    comDiscGeo.rotateX(-Math.PI * 0.5);
    this.comGroundDisc = new THREE.Mesh(
      comDiscGeo,
      new THREE.MeshBasicNodeMaterial({
        color: 0xf59e0b,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.9,
        depthWrite: false,
      })
    );
    this.supportGroup.add(this.comGroundDisc);

    this.copMarker = new THREE.Mesh(
      new THREE.SphereGeometry(0.022, 12, 10),
      new THREE.MeshBasicNodeMaterial({ color: 0x10b981 })
    );
    this.supportGroup.add(this.copMarker);

    // 3. Kinematic X-Ray Skeleton (Spine + 4 Limb Chains + 4-Bar Pantograph Rods)
    // Total segments: 2 spine + 4*4 limb segments + 2 pantograph rods = 20 segments = 40 vertices
    this.skelGeo = new THREE.BufferGeometry();
    this.skelGeo.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array(40 * 3), 3)
    );
    this.skelLines = new THREE.LineSegments(
      this.skelGeo,
      new THREE.LineBasicNodeMaterial({ color: 0x38bdf8 })
    );
    this.skeletonGroup.add(this.skelLines);

    const jointGeo = new THREE.SphereGeometry(0.019, 10, 8);
    const jointMat = new THREE.MeshBasicNodeMaterial({ color: 0xf59e0b });
    for (let i = 0; i < 22; i++) {
      const m = new THREE.Mesh(jointGeo, jointMat);
      this.skeletonGroup.add(m);
      this.skelJoints.push(m);
    }
  }

  public sync(snap: SimulationSnapshot): void {
    this.grfGroup.visible = this.visibility.grfVectors;
    this.supportGroup.visible = this.visibility.supportPolygon;
    this.targetsGroup.visible = this.visibility.footTargets;
    this.skeletonGroup.visible = this.visibility.skeleton;

    const contactPoints: Vec3[] = [];

    // 1. Update 3D GRF Vectors & Contact Rings
    for (const id of LEG_IDS) {
      const leg = snap.legs[id];
      const f = snap.body.grfWorld[id];
      const mag = Math.hypot(f.x, f.y, f.z);
      const vis = this.arrows[id];

      if (leg.inContact) {
        contactPoints.push(leg.footWorld);
      }

      if (this.visibility.grfVectors && mag > 4.0) {
        vis.group.visible = true;
        vis.ring.visible = true;

        // Scale arrow length: 180N -> ~0.32m, 500N -> ~0.88m
        const len = Math.min(1.05, mag * 0.00175);
        vis.group.position.set(leg.footWorld.x, leg.footWorld.y + 0.01, leg.footWorld.z);
        _v1.set(f.x / mag, f.y / mag, f.z / mag);
        _quat.setFromUnitVectors(_upY, _v1);
        vis.group.quaternion.copy(_quat);

        const shaftLen = Math.max(0.02, len - 0.085);
        vis.shaft.scale.set(1, shaftLen, 1);
        vis.head.position.set(0, shaftLen, 0);

        // Color gradient by load intensity: cyan -> emerald -> amber -> red
        const loadT = Math.min(1, mag / 420);
        const col = new THREE.Color().setHSL(0.54 - loadT * 0.54, 0.92, 0.58);
        vis.mat.color.copy(col);
        vis.ringMat.color.copy(col);

        vis.ring.position.set(leg.footWorld.x, leg.terrainHeight + 0.006, leg.footWorld.z);
        const ringScale = 0.75 + loadT * 1.15;
        vis.ring.scale.set(ringScale, ringScale, ringScale);
      } else {
        vis.group.visible = false;
        vis.ring.visible = false;
      }

      // Raibert planned touchdown reticle (shown primarily during swing)
      if (this.visibility.footTargets && !leg.inContact) {
        vis.targetRing.visible = true;
        vis.targetRing.position.set(leg.targetWorld.x, leg.targetWorld.y + 0.008, leg.targetWorld.z);
      } else {
        vis.targetRing.visible = false;
      }
    }

    // 2. Support Polygon Convex Hull (ordered CCW around centroid in XZ) & CoM Plumb Line
    if (this.visibility.supportPolygon) {
      const posAttr = this.supportLineGeo.getAttribute('position') as THREE.BufferAttribute;
      if (contactPoints.length >= 2) {
        this.supportLine.visible = true;
        let cx = 0;
        let cz = 0;
        for (const p of contactPoints) {
          cx += p.x;
          cz += p.z;
        }
        cx /= contactPoints.length;
        cz /= contactPoints.length;
        contactPoints.sort(
          (a, b) => Math.atan2(a.z - cz, a.x - cx) - Math.atan2(b.z - cz, b.x - cx)
        );
        for (let i = 0; i < 4; i++) {
          const p = contactPoints[Math.min(i, contactPoints.length - 1)];
          posAttr.setXYZ(i, p.x, p.y + 0.012, p.z);
        }
        // Duplicate first vertex to close the loop (Line, not LineLoop)
        const first = contactPoints[0];
        posAttr.setXYZ(4, first.x, first.y + 0.012, first.z);
        posAttr.needsUpdate = true;
      } else {
        this.supportLine.visible = false;
      }

      const plumbAttr = this.comPlumbGeo.getAttribute('position') as THREE.BufferAttribute;
      plumbAttr.setXYZ(0, snap.body.pos.x, snap.body.pos.y, snap.body.pos.z);
      plumbAttr.setXYZ(
        1,
        snap.body.comGroundWorld.x,
        snap.body.comGroundWorld.y + 0.01,
        snap.body.comGroundWorld.z
      );
      plumbAttr.needsUpdate = true;

      this.comGroundDisc.position.set(
        snap.body.comGroundWorld.x,
        snap.body.comGroundWorld.y + 0.012,
        snap.body.comGroundWorld.z
      );
      this.copMarker.visible = !snap.body.isAirborne;
      this.copMarker.position.set(
        snap.body.copWorld.x,
        snap.body.copWorld.y + 0.022,
        snap.body.copWorld.z
      );
    }

    // 3. Kinematic X-Ray Skeleton
    if (this.visibility.skeleton) {
      const attr = this.skelGeo.getAttribute('position') as THREE.BufferAttribute;
      let vIdx = 0;
      let jIdx = 0;

      const addSeg = (a: THREE.Vector3, b: THREE.Vector3) => {
        attr.setXYZ(vIdx++, a.x, a.y, a.z);
        attr.setXYZ(vIdx++, b.x, b.y, b.z);
      };
      const setJoint = (p: THREE.Vector3) => {
        if (jIdx < this.skelJoints.length) {
          this.skelJoints[jIdx].visible = true;
          this.skelJoints[jIdx].position.copy(p);
          jIdx++;
        }
      };

      const pCom = new THREE.Vector3(snap.body.pos.x, snap.body.pos.y, snap.body.pos.z);
      const pFront = new THREE.Vector3(
        snap.frontSegment.pos.x,
        snap.frontSegment.pos.y,
        snap.frontSegment.pos.z
      );
      const pRear = new THREE.Vector3(
        snap.rearSegment.pos.x,
        snap.rearSegment.pos.y,
        snap.rearSegment.pos.z
      );
      addSeg(pRear, pCom);
      addSeg(pCom, pFront);
      setJoint(pCom);
      setJoint(pFront);
      setJoint(pRear);

      const qFront = new THREE.Quaternion(
        snap.frontSegment.quat.x,
        snap.frontSegment.quat.y,
        snap.frontSegment.quat.z,
        snap.frontSegment.quat.w
      );
      const qRear = new THREE.Quaternion(
        snap.rearSegment.quat.x,
        snap.rearSegment.quat.y,
        snap.rearSegment.quat.z,
        snap.rearSegment.quat.w
      );

      for (const id of LEG_IDS) {
        const m = UNIT_D10.mounts[id];
        const segQ = m.segment === 'front' ? qFront : qRear;
        const hipW = snap.hipWorld[id];
        const sol = snap.ik[id];

        const toWorld = (local: Vec3): THREE.Vector3 => {
          return new THREE.Vector3(local.x, local.y, local.z)
            .applyQuaternion(segQ)
            .add(_v2.set(hipW.x, hipW.y, hipW.z));
        };

        const pSh = new THREE.Vector3(hipW.x, hipW.y, hipW.z);
        const pHipOff = toWorld(sol.hipOffsetJoint);
        const pElb = toWorld(sol.elbowOrStifle);
        const pWri = toWorld(sol.wristOrHock);
        const pAnk = toWorld(sol.ankle);
        const pPaw = toWorld(sol.pawSole);

        addSeg(pSh, pHipOff);
        addSeg(pHipOff, pElb);
        addSeg(pElb, pWri);
        if (sol.isFront) {
          addSeg(pWri, pPaw);
        } else {
          addSeg(pWri, pAnk);
          const pRodA = toWorld(sol.pantographRodStart);
          const pRodB = toWorld(sol.pantographRodEnd);
          addSeg(pRodA, pRodB);
        }

        setJoint(pSh);
        setJoint(pElb);
        setJoint(pWri);
        setJoint(pPaw);
      }

      while (vIdx < 40) {
        attr.setXYZ(vIdx++, 0, -100, 0);
      }
      while (jIdx < this.skelJoints.length) {
        this.skelJoints[jIdx++].visible = false;
      }
      attr.needsUpdate = true;
    }
  }
}
