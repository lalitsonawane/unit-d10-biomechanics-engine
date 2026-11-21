import './style.css';
import { BiomechanicsSimulation } from './engine/BiomechanicsSimulation';
import { WebGPUScene } from './render/WebGPUScene';
import { DashboardHUD } from './ui/DashboardHUD';

async function bootstrap(): Promise<void> {
  const hudRoot = document.getElementById('hud');
  if (!hudRoot) {
    throw new Error('Missing #hud container');
  }

  const sim = new BiomechanicsSimulation();

  // Build non-overlapping HUD shell first so #viewport gets real layout bounds
  const hud = new DashboardHUD(hudRoot, sim);
  const viewport = hud.getViewport();

  const scene = new WebGPUScene(viewport);
  hud.attachScene(scene);

  // Expose debug handle for automated browser verification
  (window as unknown as Record<string, unknown>).__UNIT_D10__ = { sim, scene, hud };

  await scene.init();
  hud.setBackendLabel(scene.backendLabel);

  // Warm up 260 ticks (~1.08s at 240Hz) so Unit D-10 starts in full steady-state Trot with live GRF traces
  for (let i = 0; i < 260; i++) {
    sim.stepOnce();
  }

  let lastTime = performance.now();
  let smoothedFps = 60;

  const animate = (now: number) => {
    const dtSec = Math.max(0.0001, (now - lastTime) / 1000);
    lastTime = now;

    const instFps = Math.min(240, 1 / dtSec);
    smoothedFps = smoothedFps * 0.92 + instFps * 0.08;

    const snap = sim.update(dtSec);
    scene.syncAndRender(snap, sim.terrain);
    hud.update(snap, smoothedFps);

    requestAnimationFrame(animate);
  };

  requestAnimationFrame(animate);
}

bootstrap().catch((err) => {
  console.error('Unit D-10 Biomechanics Engine initialization failed:', err);
  const hud = document.getElementById('hud');
  if (hud) {
    hud.innerHTML = `<div style="padding:24px;font-family:monospace;color:#fda4af;background:#0b0f16;">
      <h2>Renderer Init Error</h2>
      <pre>${String(err)}</pre>
    </div>`;
  }
});
