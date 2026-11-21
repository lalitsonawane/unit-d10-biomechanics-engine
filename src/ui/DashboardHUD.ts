import {
  DEG2RAD,
  GAIT_SPECS,
  LEG_IDS,
  RAD2DEG,
  UNIT_D10,
  type CameraPreset,
  type GaitMode,
  type LegId,
  type TerrainMode,
} from '../specs/unitD10Specs';
import type {
  BiomechanicsSimulation,
  SimulationSnapshot,
} from '../engine/BiomechanicsSimulation';
import type { WebGPUScene } from '../render/WebGPUScene';

const LEG_COLORS: Record<LegId, string> = {
  FL: '#38bdf8',
  FR: '#22d3ee',
  RL: '#f59e0b',
  RR: '#a78bfa',
};

export class DashboardHUD {
  private readonly sim: BiomechanicsSimulation;
  private scene: WebGPUScene | null = null;
  private readonly root: HTMLElement;

  // Dynamic readout elements
  private backendBadge!: HTMLElement;
  private fpsBadge!: HTMLElement;
  private speedSlider!: HTMLInputElement;
  private speedVal!: HTMLElement;
  private yawSlider!: HTMLInputElement;
  private yawVal!: HTMLElement;
  private azSlider!: HTMLInputElement;
  private azVal!: HTMLElement;
  private elSlider!: HTMLInputElement;
  private elVal!: HTMLElement;
  private capFill!: HTMLElement;
  private recoilText!: HTMLElement;
  private airbornePill!: HTMLElement;
  private totalGrfText!: HTMLElement;
  private grfCanvas!: HTMLCanvasElement;
  private grfCtx!: CanvasRenderingContext2D;

  private legForceEls = {} as Record<LegId, HTMLElement>;
  private legSubEls = {} as Record<LegId, HTMLElement>;
  private legCellEls = {} as Record<LegId, HTMLElement>;

  private mHeight!: HTMLElement;
  private mPitchRoll!: HTMLElement;
  private mSpeed!: HTMLElement;
  private mSpinePitch!: HTMLElement;
  private mSpineYaw!: HTMLElement;
  private mPeakTorque!: HTMLElement;
  private mHeadPitch!: HTMLElement;
  private mEarPin!: HTMLElement;
  private mTailWhip!: HTMLElement;

  private gaitButtons = new Map<GaitMode, HTMLButtonElement>();
  private terrainButtons = new Map<TerrainMode, HTMLButtonElement>();
  private camButtons = new Map<CameraPreset, HTMLButtonElement>();
  private timeButtons = new Map<number, HTMLButtonElement>();
  private pauseBtn!: HTMLButtonElement;
  private panelsToggleBtn!: HTMLButtonElement;
  private telemetryToggleBtn!: HTMLButtonElement;
  private workspaceEl!: HTMLElement;
  private bottomEl!: HTMLElement;

  private panelsCollapsed = false;
  private telemetryCollapsed = false;

  constructor(root: HTMLElement, sim: BiomechanicsSimulation) {
    this.root = root;
    this.sim = sim;
    this.buildDOM();
    this.bindSimEvents();
  }

  /** Bind the WebGPU scene after the #viewport stage exists in the layout. */
  public attachScene(scene: WebGPUScene): void {
    this.scene = scene;
    this.bindSceneEvents();
  }

  public getViewport(): HTMLElement {
    const el = document.getElementById('viewport');
    if (!el) throw new Error('Missing #viewport stage in HUD layout');
    return el;
  }

  private buildDOM(): void {
    this.root.innerHTML = `
      <header class="hud-topbar">
        <div class="topbar-row topbar-brand-row">
          <div class="brand-group">
            <div class="brand-dot"></div>
            <div>
              <div class="brand-title">UNIT D-10 · ELECTROMAGNETIC DOG</div>
              <div class="brand-sub">DWG D10-GA-002 · 55.0 KG · 240 HZ WBC BIOMECHANICS ENGINE</div>
            </div>
            <span class="badge" id="hud-backend">WebGPU</span>
            <span class="badge emerald" id="hud-fps">60 FPS · 240 Hz</span>
          </div>

          <div class="pill-row" id="time-pills">
            <button class="pill-btn active" data-scale="1">1.0x</button>
            <button class="pill-btn" data-scale="0.5">0.5x</button>
            <button class="pill-btn" data-scale="0.25">0.25x</button>
            <button class="pill-btn" data-scale="0.1">0.1x</button>
            <button class="pill-btn warn" id="btn-pause">PAUSE</button>
            <button class="pill-btn" id="btn-step">+4.17ms</button>
            <button class="pill-btn" id="btn-reset">RESET</button>
            <button class="pill-btn" id="btn-panels" title="Toggle side panels (H)">PANELS</button>
            <button class="pill-btn" id="btn-telemetry" title="Toggle telemetry strip (H)">TELEM</button>
          </div>
        </div>

        <div class="topbar-row topbar-cam-row">
          <div class="pill-row" id="cam-pills">
            <button class="pill-btn active" data-cam="orbit">ORBIT</button>
            <button class="pill-btn" data-cam="side">SIDE A</button>
            <button class="pill-btn" data-cam="front">FRONT B</button>
            <button class="pill-btn" data-cam="top">TOP C</button>
            <button class="pill-btn" data-cam="turret">TURRET</button>
          </div>
        </div>
      </header>

      <div class="hud-workspace" id="hud-workspace">
        <div class="left-hints">
          <div class="spec-card">
            <div class="spec-card-title">
              <span>KINEMATIC RIG SPEC</span>
              <span>MM / SI</span>
            </div>
            <div class="spec-row"><span>Wheelbase / Track</span><span>585 / 364 mm</span></div>
            <div class="spec-row"><span>Fore L1 / L2 / Pad</span><span>287 / 359 / 95</span></div>
            <div class="spec-row"><span>Hind L1 / L2 / L3</span><span>190 / 298 / 146</span></div>
            <div class="spec-row"><span>Hind Hock Linkage</span><span>4-Bar Pantograph</span></div>
            <div class="spec-row"><span>Lumbar Spine ROM</span><span>±16° P / ±15° Y</span></div>
            <div class="spec-row"><span>Inertia Ixx / Iyy</span><span>2.4 / 6.2 kg·m²</span></div>
          </div>

          <div class="spec-card">
            <div class="spec-card-title"><span>CONTROLS</span><span>LIVE</span></div>
            <div class="kbd-grid">
              <span class="kbd">1–4</span><span>Idle / Walk / Trot / Gallop</span>
              <span class="kbd">W / S</span><span>Forward / Reverse Speed</span>
              <span class="kbd">A / D</span><span>Steer Yaw Left / Right</span>
              <span class="kbd">SPACE</span><span>Fire Dorsal Railgun</span>
              <span class="kbd">P</span><span>Apply Lateral Shove</span>
              <span class="kbd">C / X</span><span>Camera / X-Ray Skeleton</span>
              <span class="kbd">H</span><span>Toggle Panels / Telemetry</span>
            </div>
          </div>
        </div>

        <div id="viewport" aria-label="Unit D-10 3D viewport"></div>

        <aside class="control-dock">
          <div>
            <div class="section-label">
              <span>LOCOMOTION GAIT</span>
              <span style="color: var(--cyan)">RAIBERT WBC</span>
            </div>
            <div class="gait-grid">
              <button class="gait-btn" data-gait="idle">IDLE</button>
              <button class="gait-btn" data-gait="walk">WALK</button>
              <button class="gait-btn active" data-gait="trot">TROT</button>
              <button class="gait-btn" data-gait="gallop">GALLOP</button>
            </div>

            <div class="slider-block">
              <div class="slider-header">
                <span>FORWARD VELOCITY</span>
                <span class="slider-val" id="val-speed">2.10 m/s</span>
              </div>
              <input type="range" id="sl-speed" min="-1.2" max="5.2" step="0.05" value="2.1" />
            </div>

            <div class="slider-block">
              <div class="slider-header">
                <span>YAW TURN RATE</span>
                <span class="slider-val" id="val-yaw">0.00 rad/s</span>
              </div>
              <input type="range" id="sl-yaw" min="-1.4" max="1.4" step="0.05" value="0" />
            </div>
          </div>

          <div class="divider"></div>

          <div>
            <div class="section-label">
              <span>DORSAL ELECTROMAGNETIC RAILGUN</span>
              <span id="val-recoil" style="color: var(--amber)">0.0 / 125 mm</span>
            </div>
            <button class="fire-btn" id="btn-fire">
              <span>FIRE RAILGUN</span>
              <span style="font-size: 10px; opacity: 0.85">[SPACE]</span>
            </button>
            <div class="cap-bar"><div class="cap-fill" id="cap-fill"></div></div>

            <div class="slider-block">
              <div class="slider-header">
                <span>TURRET AZIMUTH (±148°)</span>
                <span class="slider-val" id="val-az">0°</span>
              </div>
              <input type="range" id="sl-az" min="-148" max="148" step="1" value="0" />
            </div>

            <div class="slider-block">
              <div class="slider-header">
                <span>TURRET ELEVATION (-24°..+34°)</span>
                <span class="slider-val" id="val-el">17.5°</span>
              </div>
              <input type="range" id="sl-el" min="-24" max="34" step="0.5" value="17.5" />
            </div>

            <div class="action-row">
              <button class="action-btn" id="btn-shove">SHOVE [P]</button>
              <button class="action-btn" id="btn-drop">DROP TEST</button>
            </div>
          </div>

          <div class="divider"></div>

          <div>
            <div class="section-label"><span>PROVING GROUND TERRAIN</span></div>
            <div class="terrain-grid">
              <button class="pill-btn active" data-terrain="flat">FLAT</button>
              <button class="pill-btn" data-terrain="hills">HILLS</button>
              <button class="pill-btn" data-terrain="obstacles">OBSTACLES</button>
            </div>
          </div>

          <div class="divider"></div>

          <div>
            <div class="section-label"><span>3D TELEMETRY OVERLAYS</span></div>
            <div class="toggle-grid">
              <button class="toggle-btn on" data-ov="grfVectors">GRF VECTORS</button>
              <button class="toggle-btn on" data-ov="supportPolygon">SUPPORT POLY</button>
              <button class="toggle-btn on" data-ov="footTargets">FOOT TARGETS</button>
              <button class="toggle-btn" data-ov="skeleton">X-RAY IK [X]</button>
            </div>
          </div>
        </aside>
      </div>

      <footer class="hud-bottom" id="hud-bottom">
        <div class="telemetry-panel">
          <div class="section-label" style="margin-bottom: 0">
            <span>4-LEG GROUND REACTION FORCES (240 HZ OSCILLOSCOPE)</span>
            <div style="display: flex; align-items: center; gap: 8px">
              <span class="airborne-pill" id="pill-airborne">AERIAL PHASE</span>
              <span style="color: #f8fafc" id="val-total-grf">540 N</span>
            </div>
          </div>
          <div class="scope-wrap">
            <canvas id="grf-scope" width="520" height="64"></canvas>
          </div>
        </div>

        <div class="telemetry-panel">
          <div class="section-label" style="margin-bottom: 0">
            <span>PAW LOAD CELLS &amp; FRICTION</span>
            <span style="color: var(--emerald)">μ = 0.85</span>
          </div>
          <div class="leg-grid">
            ${LEG_IDS.map(
              (id) => `
              <div class="leg-cell contact" id="leg-cell-${id}">
                <div class="leg-head">
                  <span style="font-weight:700;color:${LEG_COLORS[id]}">${id}</span>
                  <span class="leg-dot" style="background:${LEG_COLORS[id]}"></span>
                </div>
                <div class="leg-force" id="leg-f-${id}">135 N</div>
                <div class="leg-sub" id="leg-s-${id}">STANCE</div>
              </div>
            `
            ).join('')}
          </div>
        </div>

        <div class="telemetry-panel">
          <div class="section-label" style="margin-bottom: 0">
            <span>SPINE, CHASSIS &amp; RECOIL DYNAMICS</span>
            <span style="color: var(--amber)">WBC LIVE</span>
          </div>
          <div class="metrics-grid">
            <div class="metric-box"><div class="metric-k">COM HEIGHT</div><div class="metric-v" id="m-height">0.665 m</div></div>
            <div class="metric-box"><div class="metric-k">PITCH / ROLL</div><div class="metric-v" id="m-pr">0.0° / 0.0°</div></div>
            <div class="metric-box"><div class="metric-k">GROUND SPD</div><div class="metric-v" id="m-spd">0.00 m/s</div></div>
            <div class="metric-box"><div class="metric-k">SPINE PITCH</div><div class="metric-v" id="m-sp">0.0°</div></div>
            <div class="metric-box"><div class="metric-k">SPINE YAW</div><div class="metric-v" id="m-sy">0.0°</div></div>
            <div class="metric-box"><div class="metric-k">PEAK TORQUE</div><div class="metric-v" id="m-tq">0%</div></div>
            <div class="metric-box"><div class="metric-k">HEAD FLINCH</div><div class="metric-v" id="m-head">0.0°</div></div>
            <div class="metric-box"><div class="metric-k">EAR PIN</div><div class="metric-v" id="m-ear">0.0°</div></div>
            <div class="metric-box"><div class="metric-k">TAIL WHIP</div><div class="metric-v" id="m-tail">0.0°</div></div>
          </div>
        </div>
      </footer>
    `;

    // Cache element references
    this.backendBadge = document.getElementById('hud-backend')!;
    this.fpsBadge = document.getElementById('hud-fps')!;
    this.speedSlider = document.getElementById('sl-speed') as HTMLInputElement;
    this.speedVal = document.getElementById('val-speed')!;
    this.yawSlider = document.getElementById('sl-yaw') as HTMLInputElement;
    this.yawVal = document.getElementById('val-yaw')!;
    this.azSlider = document.getElementById('sl-az') as HTMLInputElement;
    this.azVal = document.getElementById('val-az')!;
    this.elSlider = document.getElementById('sl-el') as HTMLInputElement;
    this.elVal = document.getElementById('val-el')!;
    this.capFill = document.getElementById('cap-fill')!;
    this.recoilText = document.getElementById('val-recoil')!;
    this.airbornePill = document.getElementById('pill-airborne')!;
    this.totalGrfText = document.getElementById('val-total-grf')!;
    this.grfCanvas = document.getElementById('grf-scope') as HTMLCanvasElement;
    this.grfCtx = this.grfCanvas.getContext('2d')!;
    this.pauseBtn = document.getElementById('btn-pause') as HTMLButtonElement;
    this.panelsToggleBtn = document.getElementById('btn-panels') as HTMLButtonElement;
    this.telemetryToggleBtn = document.getElementById('btn-telemetry') as HTMLButtonElement;
    this.workspaceEl = document.getElementById('hud-workspace')!;
    this.bottomEl = document.getElementById('hud-bottom')!;

    for (const id of LEG_IDS) {
      this.legForceEls[id] = document.getElementById(`leg-f-${id}`)!;
      this.legSubEls[id] = document.getElementById(`leg-s-${id}`)!;
      this.legCellEls[id] = document.getElementById(`leg-cell-${id}`)!;
    }

    this.mHeight = document.getElementById('m-height')!;
    this.mPitchRoll = document.getElementById('m-pr')!;
    this.mSpeed = document.getElementById('m-spd')!;
    this.mSpinePitch = document.getElementById('m-sp')!;
    this.mSpineYaw = document.getElementById('m-sy')!;
    this.mPeakTorque = document.getElementById('m-tq')!;
    this.mHeadPitch = document.getElementById('m-head')!;
    this.mEarPin = document.getElementById('m-ear')!;
    this.mTailWhip = document.getElementById('m-tail')!;

    this.root.querySelectorAll<HTMLButtonElement>('[data-gait]').forEach((btn) => {
      this.gaitButtons.set(btn.dataset.gait as GaitMode, btn);
    });
    this.root.querySelectorAll<HTMLButtonElement>('[data-terrain]').forEach((btn) => {
      this.terrainButtons.set(btn.dataset.terrain as TerrainMode, btn);
    });
    this.root.querySelectorAll<HTMLButtonElement>('[data-cam]').forEach((btn) => {
      this.camButtons.set(btn.dataset.cam as CameraPreset, btn);
    });
    this.root.querySelectorAll<HTMLButtonElement>('[data-scale]').forEach((btn) => {
      this.timeButtons.set(Number(btn.dataset.scale), btn);
    });
  }

  private setPanelsCollapsed(collapsed: boolean): void {
    this.panelsCollapsed = collapsed;
    this.workspaceEl.classList.toggle('panels-collapsed', collapsed);
    this.panelsToggleBtn.classList.toggle('active', collapsed);
    this.panelsToggleBtn.textContent = collapsed ? 'PANELS+' : 'PANELS';
  }

  private setTelemetryCollapsed(collapsed: boolean): void {
    this.telemetryCollapsed = collapsed;
    this.bottomEl.classList.toggle('collapsed', collapsed);
    this.telemetryToggleBtn.classList.toggle('active', collapsed);
    this.telemetryToggleBtn.textContent = collapsed ? 'TELEM+' : 'TELEM';
  }

  private toggleHudChrome(): void {
    // H cycles: both open → collapse both → restore both
    if (!this.panelsCollapsed && !this.telemetryCollapsed) {
      this.setPanelsCollapsed(true);
      this.setTelemetryCollapsed(true);
    } else {
      this.setPanelsCollapsed(false);
      this.setTelemetryCollapsed(false);
    }
  }

  private bindSimEvents(): void {
    this.gaitButtons.forEach((btn, mode) => {
      btn.addEventListener('click', () => this.selectGait(mode));
    });

    this.terrainButtons.forEach((btn, mode) => {
      btn.addEventListener('click', () => {
        this.sim.setTerrain(mode);
        this.terrainButtons.forEach((b, m) => b.classList.toggle('active', m === mode));
      });
    });

    this.timeButtons.forEach((btn, scale) => {
      btn.addEventListener('click', () => {
        this.sim.command.timeScale = scale;
        this.sim.command.paused = false;
        this.pauseBtn.classList.remove('active');
        this.timeButtons.forEach((b, s) => b.classList.toggle('active', s === scale));
      });
    });

    this.pauseBtn.addEventListener('click', () => {
      this.sim.command.paused = !this.sim.command.paused;
      this.pauseBtn.classList.toggle('active', this.sim.command.paused);
    });

    document.getElementById('btn-step')!.addEventListener('click', () => {
      this.sim.command.paused = true;
      this.pauseBtn.classList.add('active');
      this.sim.stepOnce();
    });

    document.getElementById('btn-reset')!.addEventListener('click', () => {
      this.sim.reset(0);
    });

    this.panelsToggleBtn.addEventListener('click', () => {
      this.setPanelsCollapsed(!this.panelsCollapsed);
    });

    this.telemetryToggleBtn.addEventListener('click', () => {
      this.setTelemetryCollapsed(!this.telemetryCollapsed);
    });

    this.speedSlider.addEventListener('input', () => {
      const v = Number(this.speedSlider.value);
      this.sim.command.desiredSpeed = v;
      this.speedVal.textContent = `${v.toFixed(2)} m/s`;
    });

    this.yawSlider.addEventListener('input', () => {
      const v = Number(this.yawSlider.value);
      this.sim.command.desiredYawRate = v;
      this.yawVal.textContent = `${v.toFixed(2)} rad/s`;
    });

    this.azSlider.addEventListener('input', () => {
      const deg = Number(this.azSlider.value);
      this.sim.command.turretAzimuth = deg * DEG2RAD;
      this.azVal.textContent = `${deg.toFixed(0)}°`;
    });

    this.elSlider.addEventListener('input', () => {
      const deg = Number(this.elSlider.value);
      this.sim.command.turretElevation = deg * DEG2RAD;
      this.elVal.textContent = `${deg.toFixed(1)}°`;
    });

    document.getElementById('btn-fire')!.addEventListener('click', () => {
      this.sim.fireRailgun();
    });

    let shoveDir: 1 | -1 = 1;
    document.getElementById('btn-shove')!.addEventListener('click', () => {
      this.sim.applyLateralShove(shoveDir, 125);
      shoveDir = shoveDir === 1 ? -1 : 1;
    });

    document.getElementById('btn-drop')!.addEventListener('click', () => {
      this.sim.triggerDropTest(0.55);
    });

    const keysDown = new Set<string>();
    window.addEventListener('keydown', (e) => {
      if (e.target instanceof HTMLInputElement) return;
      const code = e.code;
      keysDown.add(code);

      if (code === 'Space') {
        e.preventDefault();
        this.sim.fireRailgun();
      } else if (code === 'Digit1') {
        this.selectGait('idle');
      } else if (code === 'Digit2') {
        this.selectGait('walk');
      } else if (code === 'Digit3') {
        this.selectGait('trot');
      } else if (code === 'Digit4') {
        this.selectGait('gallop');
      } else if (code === 'KeyP') {
        this.sim.applyLateralShove(shoveDir, 125);
        shoveDir = shoveDir === 1 ? -1 : 1;
      } else if (code === 'KeyR') {
        this.sim.reset(0);
      } else if (code === 'KeyH') {
        this.toggleHudChrome();
      } else if (code === 'KeyC') {
        if (!this.scene) return;
        const order: CameraPreset[] = ['orbit', 'side', 'front', 'top', 'turret'];
        const next = order[(order.indexOf(this.scene.cameraPreset) + 1) % order.length];
        this.selectCamera(next);
      } else if (code === 'KeyX') {
        const btn = this.root.querySelector<HTMLButtonElement>('[data-ov="skeleton"]');
        btn?.click();
      } else if (code === 'KeyW' || code === 'ArrowUp') {
        const nextSpd = Math.min(5.2, this.sim.command.desiredSpeed + 0.35);
        this.sim.command.desiredSpeed = nextSpd;
        this.speedSlider.value = nextSpd.toFixed(2);
        this.speedVal.textContent = `${nextSpd.toFixed(2)} m/s`;
      } else if (code === 'KeyS' || code === 'ArrowDown') {
        const nextSpd = Math.max(-1.2, this.sim.command.desiredSpeed - 0.35);
        this.sim.command.desiredSpeed = nextSpd;
        this.speedSlider.value = nextSpd.toFixed(2);
        this.speedVal.textContent = `${nextSpd.toFixed(2)} m/s`;
      } else if (code === 'KeyA' || code === 'ArrowLeft') {
        this.sim.command.desiredYawRate = 0.85;
        this.yawSlider.value = '0.85';
        this.yawVal.textContent = '0.85 rad/s';
      } else if (code === 'KeyD' || code === 'ArrowRight') {
        this.sim.command.desiredYawRate = -0.85;
        this.yawSlider.value = '-0.85';
        this.yawVal.textContent = '-0.85 rad/s';
      }
    });

    window.addEventListener('keyup', (e) => {
      keysDown.delete(e.code);
      if (
        e.code === 'KeyA' ||
        e.code === 'KeyD' ||
        e.code === 'ArrowLeft' ||
        e.code === 'ArrowRight'
      ) {
        if (
          !keysDown.has('KeyA') &&
          !keysDown.has('KeyD') &&
          !keysDown.has('ArrowLeft') &&
          !keysDown.has('ArrowRight')
        ) {
          this.sim.command.desiredYawRate = 0;
          this.yawSlider.value = '0';
          this.yawVal.textContent = '0.00 rad/s';
        }
      }
    });
  }

  private bindSceneEvents(): void {
    if (!this.scene) return;

    this.camButtons.forEach((btn, preset) => {
      btn.addEventListener('click', () => this.selectCamera(preset));
    });

    this.root.querySelectorAll<HTMLButtonElement>('[data-ov]').forEach((btn) => {
      btn.addEventListener('click', () => {
        if (!this.scene) return;
        const key = btn.dataset.ov as keyof typeof this.scene.overlays.visibility;
        const next = !this.scene.overlays.visibility[key];
        this.scene.overlays.visibility[key] = next;
        btn.classList.toggle('on', next);
        if (key === 'skeleton') {
          this.scene.robot.setXrayMode(next);
        }
      });
    });
  }

  public selectGait(mode: GaitMode): void {
    this.sim.setGait(mode, true);
    this.gaitButtons.forEach((b, m) => b.classList.toggle('active', m === mode));
    const spd = GAIT_SPECS[mode].nominalSpeed;
    this.speedSlider.value = spd.toFixed(2);
    this.speedVal.textContent = `${spd.toFixed(2)} m/s`;
  }

  public selectCamera(preset: CameraPreset): void {
    if (!this.scene) return;
    this.scene.setCameraPreset(preset);
    this.camButtons.forEach((b, p) => b.classList.toggle('active', p === preset));
  }

  public setBackendLabel(label: string): void {
    this.backendBadge.textContent = label;
  }

  public update(snap: SimulationSnapshot, fps: number): void {
    this.fpsBadge.textContent = `${Math.round(fps)} FPS · ${snap.physicsHz} Hz`;

    // Capacitor & recoil
    this.capFill.style.width = `${(snap.secondary.capacitorCharge * 100).toFixed(0)}%`;
    this.recoilText.textContent = `${(snap.secondary.recoilDisp * 1000).toFixed(1)} / 125 mm`;

    // Total GRF & Aerial Phase badge
    this.totalGrfText.textContent = `${snap.body.totalVerticalGRF.toFixed(0)} N / ${(
      UNIT_D10.mass * UNIT_D10.gravity
    ).toFixed(0)} N`;
    this.airbornePill.classList.toggle('active', snap.body.isAirborne);

    // Per-leg load cells
    for (const id of LEG_IDS) {
      const fn = snap.body.normalForce[id];
      const muPct = Math.round(snap.body.frictionRatio[id] * 100);
      const inContact = snap.legs[id].inContact;
      this.legForceEls[id].textContent = `${fn.toFixed(0)} N`;
      this.legSubEls[id].textContent = inContact
        ? `STANCE · μ ${muPct}%`
        : `SWING ${(snap.legs[id].clearance * 100).toFixed(0)}cm`;
      this.legCellEls[id].classList.toggle('contact', inContact);
    }

    // Chassis, spine & secondary recoil readouts
    const clearance = snap.body.pos.y - snap.body.comGroundWorld.y;
    const horizSpeed = Math.hypot(snap.body.vel.x, snap.body.vel.z);
    this.mHeight.textContent = `${clearance.toFixed(3)} m`;
    this.mPitchRoll.textContent = `${(snap.body.euler.pitch * RAD2DEG).toFixed(1)}° / ${(
      snap.body.euler.roll * RAD2DEG
    ).toFixed(1)}°`;
    this.mSpeed.textContent = `${horizSpeed.toFixed(2)} m/s`;
    this.mSpinePitch.textContent = `${(snap.spine.pitch * RAD2DEG).toFixed(1)}°`;
    this.mSpineYaw.textContent = `${(snap.spine.yaw * RAD2DEG).toFixed(1)}°`;
    this.mPeakTorque.textContent = `${Math.min(100, snap.peakTorquePct).toFixed(0)}%`;
    this.mHeadPitch.textContent = `${(snap.secondary.neckPitch * RAD2DEG).toFixed(1)}°`;
    this.mEarPin.textContent = `${(snap.secondary.earLeftPitch * RAD2DEG).toFixed(1)}°`;
    const tailTip = snap.secondary.tailYaw[snap.secondary.tailYaw.length - 1] * RAD2DEG;
    this.mTailWhip.textContent = `${tailTip.toFixed(1)}°`;

    // Render 240Hz GRF Oscilloscope
    this.drawGRFScope();
  }

  private drawGRFScope(): void {
    const ctx = this.grfCtx;
    const w = this.grfCanvas.width;
    const h = this.grfCanvas.height;
    const hist = this.sim.history;

    ctx.clearRect(0, 0, w, h);

    // Subtle horizontal grid lines (200N, 400N, 600N)
    const maxForce = 680;
    ctx.strokeStyle = 'rgba(148, 163, 184, 0.12)';
    ctx.lineWidth = 1;
    for (const refN of [200, 400, 600]) {
      const y = h - (refN / maxForce) * (h - 8) - 4;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }

    if (hist.length < 2) return;

    const cap = this.sim.historyCapacity;
    const dx = w / (cap - 1);

    // Highlight ballistic aerial flight windows (all 4 GRFs == 0 N)
    ctx.fillStyle = 'rgba(244, 63, 94, 0.16)';
    for (let i = 0; i < hist.length; i++) {
      if (hist[i].isAirborne) {
        const x = (cap - hist.length + i) * dx;
        ctx.fillRect(x, 0, Math.max(2, dx + 0.5), h);
      }
    }

    // Draw 4 leg vertical GRF traces
    const series: { key: 'grfFL' | 'grfFR' | 'grfRL' | 'grfRR'; color: string }[] = [
      { key: 'grfFL', color: LEG_COLORS.FL },
      { key: 'grfFR', color: LEG_COLORS.FR },
      { key: 'grfRL', color: LEG_COLORS.RL },
      { key: 'grfRR', color: LEG_COLORS.RR },
    ];

    ctx.lineWidth = 1.6;
    for (const s of series) {
      ctx.strokeStyle = s.color;
      ctx.beginPath();
      for (let i = 0; i < hist.length; i++) {
        const x = (cap - hist.length + i) * dx;
        const val = Math.min(maxForce, hist[i][s.key]);
        const y = h - (val / maxForce) * (h - 8) - 4;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }

    // Small legend in top-left of scope
    ctx.font = '600 9px "JetBrains Mono", monospace';
    let lx = 8;
    for (const id of LEG_IDS) {
      ctx.fillStyle = LEG_COLORS[id];
      ctx.fillText(id, lx, 12);
      lx += 24;
    }
    ctx.fillStyle = 'rgba(148, 163, 184, 0.65)';
    ctx.fillText('0..680 N', w - 54, 12);
  }
}
