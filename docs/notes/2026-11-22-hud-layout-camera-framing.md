# 2026-11-22 — Unit D-10 HUD layout & camera framing fix

- Notion page: https://app.notion.com/p/41f89f54d2c28154ace7e3d17c9722b2?pvs=204
- Status: Active
- Type: Session summary
- Date: 2026-11-22
- Repo mirror: `docs/notes/2026-11-22-hud-layout-camera-framing.md`

## Outcome

Docked `#viewport` into a non-overlapping CSS grid shell above a compact/collapsible bottom telemetry strip and beside side panels. `WebGPUScene` now sizes exclusively from the `#viewport` container via `ResizeObserver`. Camera presets (`orbit`, `side`, `front`, `top`, `turret`) pull back and lower look-at Y so paws, foot targets, contact rings, support polygon, and legs stay framed with margin. Fixed `1280×720` topbar `#hud-fps` vs camera-pill collision and control-dock clipping of 3D telemetry overlay toggles. Converted support-polygon `LineLoop` → closed `Line` to eliminate WebGPU console errors.

## Key decisions (+ rationale)

- **Non-overlapping shell over full-bleed HUD overlay**: Previous `#viewport { position: fixed; inset: 0 }` sat under opaque bottom telemetry (~20–26% of window). Switching `#hud` to a 3-row grid (`topbar / workspace / telemetry`) with `#viewport` as the workspace center cell gives `0 px²` geometric overlap.
- **ResizeObserver on `#viewport`**: Panel collapse (`H` / PANELS / TELEM) and responsive dock widths change the stage size without a window resize; observing the container keeps `camera.aspect` and renderer bounds correct.
- **Two-row topbar**: Brand + time controls on row 1, camera presets on row 2 — eliminates FPS badge vs ORBIT/SIDE collision at `1280px`.
- **Closed `THREE.Line` for support polygon**: WebGPU backend rejects `LineLoop`; closing with a duplicated first vertex preserves the convex hull outline without console spam.

## Artifacts/links

- Verified after-screenshots:
  - `/cursor/stores/bc-01a1fe90-71e4-7928-ae9b-8ce63cf5c61c/media/unit-d10-fixed-1440.png`
  - `/cursor/stores/bc-01a1fe90-71e4-7928-ae9b-8ce63cf5c61c/media/unit-d10-fixed-1280.png`
- Checks: `tsc --noEmit`, `npm test`, `npm run build` all passed; headless Chrome verify at `1440×900` and `1280×720` reported zero layout overlaps and zero console/page errors.

## Open follow-ups

- After merge, confirm Vercel production deploy at https://unit-d10-biomechanics-engine.vercel.app matches the verified preview screenshots.
