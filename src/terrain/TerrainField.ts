import type { TerrainMode } from '../specs/unitD10Specs';

export interface Vec3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Deterministic continuous terrain field for 240Hz contact queries and WebGPU mesh generation.
 */
export class TerrainField {
  public mode: TerrainMode = 'flat';

  public setMode(mode: TerrainMode): void {
    this.mode = mode;
  }

  public getHeight(x: number, z: number): number {
    switch (this.mode) {
      case 'flat':
        return 0;

      case 'hills': {
        // Gentle rolling proving ground hills navigable at walk/trot/gallop
        const h1 = 0.14 * Math.sin(x * 0.42) * Math.cos(z * 0.36);
        const h2 = 0.07 * Math.sin(x * 0.95 + z * 0.55);
        const h3 = 0.035 * Math.cos(x * 1.65 - z * 1.15);
        // Smooth flat pad near origin (r < 0.8m) so reset starts level
        const r2 = x * x + z * z;
        const blend = Math.min(1, r2 / 3.5);
        return (h1 + h2 + h3) * blend;
      }

      case 'obstacles': {
        // Repeating smooth tactical speed bumps, asymmetric step planks & low ramps along X and Z
        const r2 = x * x + z * z;
        if (r2 < 1.0) return 0;

        // Periodic smooth trapezoidal planks spaced every 2.8m along X
        const cellX = ((x % 2.8) + 2.8) % 2.8;
        let h = 0;

        // Full-width low ramp beam at cellX in [0.9..1.55]
        if (cellX > 0.85 && cellX < 1.6) {
          const t = (cellX - 0.85) / 0.75;
          const bump = Math.sin(t * Math.PI);
          h = Math.max(h, 0.095 * Math.pow(bump, 0.7));
        }

        // Asymmetric left/right alternating step blocks at cellX in [2.0..2.65]
        if (cellX > 1.95 && cellX < 2.65) {
          const t = (cellX - 1.95) / 0.7;
          const bump = Math.sin(t * Math.PI);
          const sideWave = Math.sin(z * 1.6 + Math.floor(x / 2.8) * 1.7);
          if (sideWave > -0.15) {
            const sideEnv = Math.min(1, (sideWave + 0.15) / 0.5);
            h = Math.max(h, 0.115 * Math.pow(bump, 0.65) * sideEnv);
          }
        }

        const fade = Math.min(1, Math.max(0, (Math.sqrt(r2) - 1.0) / 0.8));
        return h * fade;
      }

      default: {
        const _exhaustive: never = this.mode;
        return _exhaustive;
      }
    }
  }

  public getNormal(x: number, z: number): Vec3 {
    if (this.mode === 'flat') {
      return { x: 0, y: 1, z: 0 };
    }
    const eps = 0.03;
    const hL = this.getHeight(x - eps, z);
    const hR = this.getHeight(x + eps, z);
    const hD = this.getHeight(x, z - eps);
    const hU = this.getHeight(x, z + eps);

    const dhdx = (hR - hL) / (2 * eps);
    const dhdz = (hU - hD) / (2 * eps);
    const nx = -dhdx;
    const ny = 1.0;
    const nz = -dhdz;
    const len = Math.hypot(nx, ny, nz) || 1;
    return { x: nx / len, y: ny / len, z: nz / len };
  }
}
