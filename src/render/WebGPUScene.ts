import * as THREE from 'three/webgpu';
import {
  abs,
  clamp,
  color,
  float,
  fract,
  length,
  max,
  mix,
  positionWorld,
  smoothstep,
  uniform,
  vec2,
} from 'three/tsl';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import type { CameraPreset, TerrainMode } from '../specs/unitD10Specs';
import type { SimulationSnapshot } from '../engine/BiomechanicsSimulation';
import type { TerrainField } from '../terrain/TerrainField';
import { UnitD10Model } from '../rig/UnitD10Model';
import { TelemetryOverlays3D } from './TelemetryOverlays3D';

export class WebGPUScene {
  public renderer!: THREE.WebGPURenderer;
  public readonly scene = new THREE.Scene();
  public readonly camera: THREE.PerspectiveCamera;
  public controls!: OrbitControls;
  public readonly robot = new UnitD10Model();
  public readonly overlays = new TelemetryOverlays3D();
  public backendLabel = 'Initializing...';
  public cameraPreset: CameraPreset = 'orbit';

  private readonly container: HTMLElement;
  private readonly keyLight: THREE.DirectionalLight;
  private readonly keyTarget = new THREE.Object3D();
  private readonly terrainMesh: THREE.Mesh;
  private readonly terrainGeo: THREE.PlaneGeometry;
  private readonly terrainSize = 56;
  private readonly terrainSegments = 128;
  private lastTerrainSnapX = Number.NaN;
  private lastTerrainSnapZ = Number.NaN;
  private lastTerrainMode: TerrainMode | null = null;

  // TSL uniforms for proving ground shader
  private readonly robotPosUniform = uniform(new THREE.Vector3(0, 0, 0));

  // Pooled railgun sabot tracer bolts
  private readonly boltPool: THREE.Mesh[] = [];
  private readonly boltGroup = new THREE.Group();

  // Chase camera tracking
  private prevBodyPos = new THREE.Vector3(0, 0.665, 0);
  private cameraInitialized = false;

  constructor(container: HTMLElement) {
    this.container = container;
    const width = container.clientWidth || window.innerWidth;
    const height = container.clientHeight || window.innerHeight;

    this.scene.background = new THREE.Color(0x0b0f16);
    this.scene.fog = new THREE.FogExp2(0x0b0f16, 0.026);

    this.camera = new THREE.PerspectiveCamera(42, width / height, 0.05, 160);
    this.camera.position.set(1.85, 1.38, -2.35);

    // 3-Point Aerospace Proving-Ground Lighting
    const hemi = new THREE.HemisphereLight(0xcbe2ff, 0x1c222d, 0.85);
    this.scene.add(hemi);

    this.keyLight = new THREE.DirectionalLight(0xfff4e4, 2.4);
    this.keyLight.position.set(4.5, 7.5, -4.5);
    this.keyLight.castShadow = true;
    this.keyLight.shadow.mapSize.set(2048, 2048);
    this.keyLight.shadow.camera.near = 1.0;
    this.keyLight.shadow.camera.far = 22.0;
    const d = 4.2;
    this.keyLight.shadow.camera.left = -d;
    this.keyLight.shadow.camera.right = d;
    this.keyLight.shadow.camera.top = d;
    this.keyLight.shadow.camera.bottom = -d;
    this.keyLight.shadow.bias = -0.0005;
    this.scene.add(this.keyLight);
    this.scene.add(this.keyTarget);
    this.keyLight.target = this.keyTarget;

    // Cool cyan technical rim light from opposite rear quadrant
    const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.95);
    rimLight.position.set(-5.0, 3.2, 5.5);
    this.scene.add(rimLight);

    // Subtle warm under-fill
    const fillLight = new THREE.DirectionalLight(0x94a3b8, 0.45);
    fillLight.position.set(2.0, 1.5, 4.0);
    this.scene.add(fillLight);

    // Procedural TSL Proving-Ground Terrain Mesh
    this.terrainGeo = new THREE.PlaneGeometry(
      this.terrainSize,
      this.terrainSize,
      this.terrainSegments,
      this.terrainSegments
    );
    this.terrainGeo.rotateX(-Math.PI * 0.5);

    const terrainMat = this.buildTerrainNodeMaterial();
    this.terrainMesh = new THREE.Mesh(this.terrainGeo, terrainMat);
    this.terrainMesh.receiveShadow = true;
    this.scene.add(this.terrainMesh);

    // Add Unit D-10 & 3D Telemetry Overlays
    this.scene.add(this.robot.root);
    this.scene.add(this.overlays.root);
    this.scene.add(this.boltGroup);

    // Pre-allocate glowing railgun sabot tracer bolts
    const boltGeo = new THREE.CylinderGeometry(0.012, 0.022, 1.35, 8);
    boltGeo.rotateZ(Math.PI * 0.5);
    const boltMat = new THREE.MeshBasicNodeMaterial({
      color: 0x7dd3fc,
      transparent: true,
      opacity: 0.95,
      blending: THREE.AdditiveBlending,
      depthWrite: false,
    });
    for (let i = 0; i < 8; i++) {
      const m = new THREE.Mesh(boltGeo, boltMat);
      m.visible = false;
      this.boltGroup.add(m);
      this.boltPool.push(m);
    }
  }

  private buildTerrainNodeMaterial(): THREE.MeshStandardNodeMaterial {
    const mat = new THREE.MeshStandardNodeMaterial({
      roughness: 0.72,
      metalness: 0.22,
    });

    // World XZ coordinates for infinite crisp metric grid (1.0m major, 0.25m minor)
    const wx = positionWorld.x;
    const wy = positionWorld.y;
    const wz = positionWorld.z;

    // Distance from moving robot for gentle radial spotlight focus
    const dx = wx.sub(this.robotPosUniform.x);
    const dz = wz.sub(this.robotPosUniform.z);
    const distFromBot = length(vec2(dx, dz));
    const radialFade = float(1.0).sub(smoothstep(float(8.0), float(26.0), distFromBot));

    // Major 1.0m grid lines
    const gxMajor = abs(fract(wx.add(0.5)).sub(0.5));
    const gzMajor = abs(fract(wz.add(0.5)).sub(0.5));
    const majorLine = max(
      float(1.0).sub(smoothstep(float(0.008), float(0.02), gxMajor)),
      float(1.0).sub(smoothstep(float(0.008), float(0.02), gzMajor))
    );

    // Minor 0.25m sub-grid lines
    const gxMinor = abs(fract(wx.mul(4.0).add(0.5)).sub(0.5));
    const gzMinor = abs(fract(wz.mul(4.0).add(0.5)).sub(0.5));
    const minorLine = max(
      float(1.0).sub(smoothstep(float(0.02), float(0.05), gxMinor)),
      float(1.0).sub(smoothstep(float(0.02), float(0.05), gzMinor))
    ).mul(float(1.0).sub(smoothstep(float(3.0), float(12.0), distFromBot)));

    // Subtle elevation contour bands on hills & obstacles
    const elevBand = abs(fract(wy.mul(20.0)).sub(0.5));
    const contour = float(1.0)
      .sub(smoothstep(float(0.03), float(0.08), elevBand))
      .mul(smoothstep(float(0.005), float(0.03), abs(wy)));

    const baseFloor = color(0x111722);
    const elevatedFloor = color(0x1b2636);
    const elevTint = mix(baseFloor, elevatedFloor, clamp(wy.mul(4.0), float(0.0), float(1.0)));

    const minorCol = mix(elevTint, color(0x1e2c3e), minorLine.mul(0.55));
    const majorCol = mix(minorCol, color(0x2d4866), majorLine.mul(radialFade).mul(0.9));
    const finalCol = mix(majorCol, color(0x38bdf8), contour.mul(0.32).mul(radialFade));

    mat.colorNode = finalCol;
    mat.emissiveNode = color(0x0ea5e9)
      .mul(majorLine.mul(0.08).add(contour.mul(0.18)))
      .mul(radialFade);

    return mat;
  }

  public async init(): Promise<void> {
    this.renderer = new THREE.WebGPURenderer({
      antialias: true,
      powerPreference: 'high-performance',
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(
      this.container.clientWidth || window.innerWidth,
      this.container.clientHeight || window.innerHeight
    );
    this.renderer.shadowMap.enabled = true;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.12;

    await this.renderer.init();

    // Detect whether WebGPU or WebGL2 fallback backend initialized
    const backendObj = this.renderer.backend as { isWebGPUBackend?: boolean };
    this.backendLabel = backendObj?.isWebGPUBackend ? 'WebGPU Native' : 'WebGPU Engine (WebGL2 Backend)';

    this.container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.minDistance = 0.85;
    this.controls.maxDistance = 12.0;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.target.set(0, 0.65, 0);
    this.controls.update();

    window.addEventListener('resize', this.handleResize);
  }

  private handleResize = (): void => {
    const w = this.container.clientWidth || window.innerWidth;
    const h = this.container.clientHeight || window.innerHeight;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  };

  public setCameraPreset(preset: CameraPreset): void {
    this.cameraPreset = preset;
    this.controls.enabled = preset === 'orbit';
  }

  private updateTerrainGeometry(centerX: number, centerZ: number, terrain: TerrainField): void {
    const snapStep = 2.0;
    const snapX = Math.round(centerX / snapStep) * snapStep;
    const snapZ = Math.round(centerZ / snapStep) * snapStep;

    if (
      snapX === this.lastTerrainSnapX &&
      snapZ === this.lastTerrainSnapZ &&
      terrain.mode === this.lastTerrainMode
    ) {
      return;
    }

    this.lastTerrainSnapX = snapX;
    this.lastTerrainSnapZ = snapZ;
    this.lastTerrainMode = terrain.mode;

    this.terrainMesh.position.set(snapX, 0, snapZ);

    const pos = this.terrainGeo.getAttribute('position') as THREE.BufferAttribute;
    for (let i = 0; i < pos.count; i++) {
      const lx = pos.getX(i);
      const lz = pos.getZ(i);
      const wy = terrain.getHeight(snapX + lx, snapZ + lz);
      pos.setY(i, wy);
    }
    pos.needsUpdate = true;
    this.terrainGeo.computeVertexNormals();
  }

  public syncAndRender(snap: SimulationSnapshot, terrain: TerrainField): void {
    const bp = snap.body.pos;

    // 1. Update infinite moving terrain window & shader uniform
    this.updateTerrainGeometry(bp.x, bp.z, terrain);
    this.robotPosUniform.value.set(bp.x, bp.y, bp.z);

    // 2. Keep key directional shadow frustum centered on Unit D-10
    this.keyLight.position.set(bp.x + 4.5, bp.y + 7.2, bp.z - 4.5);
    this.keyTarget.position.set(bp.x, bp.y, bp.z);
    this.keyTarget.updateMatrixWorld();

    // 3. Sync Unit D-10 procedural CAD rig & 3D telemetry overlays
    this.robot.sync(snap);
    this.overlays.sync(snap);

    // 4. Sync hypervelocity railgun tracer bolts
    const xAxis = new THREE.Vector3(1, 0, 0);
    const dirV = new THREE.Vector3();
    for (let i = 0; i < this.boltPool.length; i++) {
      const mesh = this.boltPool[i];
      if (i < snap.projectiles.length) {
        const p = snap.projectiles[i];
        mesh.visible = true;
        mesh.position.set(p.pos.x, p.pos.y, p.pos.z);
        dirV.set(p.dir.x, p.dir.y, p.dir.z).normalize();
        mesh.quaternion.setFromUnitVectors(xAxis, dirV);
      } else {
        mesh.visible = false;
      }
    }

    // 5. Update Camera Tracking
    this.updateCamera(snap);

    // 6. Render frame via Three.js WebGPU pipeline!
    this.renderer.render(this.scene, this.camera);
  }

  private updateCamera(snap: SimulationSnapshot): void {
    const bp = snap.body.pos;
    const yaw = snap.body.euler.yaw;
    const fwdX = Math.cos(yaw);
    const fwdZ = -Math.sin(yaw);
    const rightX = Math.sin(yaw);
    const rightZ = Math.cos(yaw);

    if (!this.cameraInitialized) {
      this.prevBodyPos.set(bp.x, bp.y, bp.z);
      this.camera.position.set(bp.x + 1.85, bp.y + 0.72, bp.z - 2.25);
      this.controls.target.set(bp.x, bp.y * 0.85 + 0.1, bp.z);
      this.controls.update();
      this.cameraInitialized = true;
    }

    switch (this.cameraPreset) {
      case 'orbit': {
        // Delta-follow body translation smoothly while giving user full 3D OrbitControls freedom
        const dx = bp.x - this.prevBodyPos.x;
        const dy = (bp.y - this.prevBodyPos.y) * 0.35;
        const dz = bp.z - this.prevBodyPos.z;
        this.camera.position.x += dx;
        this.camera.position.y += dy;
        this.camera.position.z += dz;
        this.controls.target.set(bp.x, bp.y * 0.85 + 0.1, bp.z);
        this.controls.update();
        break;
      }

      case 'side': {
        // Blueprint View A — Left Profile Elevation (looking from left -Z side toward right +Z)
        const dist = 2.65;
        const desiredPos = new THREE.Vector3(
          bp.x - rightX * dist,
          bp.y + 0.08,
          bp.z - rightZ * dist
        );
        this.camera.position.lerp(desiredPos, 0.16);
        this.camera.lookAt(bp.x + fwdX * 0.06, bp.y + 0.04, bp.z + fwdZ * 0.06);
        break;
      }

      case 'front': {
        // Blueprint View B — Front Elevation
        const dist = 2.45;
        const desiredPos = new THREE.Vector3(
          bp.x + fwdX * dist,
          bp.y + 0.12,
          bp.z + fwdZ * dist
        );
        this.camera.position.lerp(desiredPos, 0.16);
        this.camera.lookAt(bp.x, bp.y + 0.05, bp.z);
        break;
      }

      case 'top': {
        // Blueprint View C — Top Plan View & Turret Slew Envelope
        const desiredPos = new THREE.Vector3(bp.x - fwdX * 0.05, bp.y + 3.05, bp.z - fwdZ * 0.05);
        this.camera.position.lerp(desiredPos, 0.16);
        this.camera.lookAt(bp.x, bp.y, bp.z);
        break;
      }

      case 'turret': {
        // Dramatic over-the-railgun barrel chase view!
        const azTotal = yaw + snap.secondary.turretAzimuth;
        const tFwdX = Math.cos(azTotal);
        const tFwdZ = -Math.sin(azTotal);
        const desiredPos = new THREE.Vector3(
          bp.x - tFwdX * 1.45,
          bp.y + 0.82,
          bp.z - tFwdZ * 1.45
        );
        this.camera.position.lerp(desiredPos, 0.18);
        this.camera.lookAt(
          bp.x + tFwdX * 1.8,
          bp.y + 0.32 + Math.sin(snap.secondary.turretElevation) * 1.5,
          bp.z + tFwdZ * 1.8
        );
        break;
      }

      default: {
        const _exhaustive: never = this.cameraPreset;
        return _exhaustive;
      }
    }

    this.prevBodyPos.set(bp.x, bp.y, bp.z);
  }
}
