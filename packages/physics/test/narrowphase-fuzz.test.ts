import { describe, expect, it } from 'vitest';
import {
  BoxShape,
  CapsuleShape,
  type CollideConfig,
  ConvexHullShape,
  CylinderShape,
  DistanceOutput,
  DistanceProxy,
  Manifold,
  Quat,
  type Shape,
  SphereShape,
  Transform,
  Vec3,
  collideShapes,
  gjkDistance,
} from '../src';

const config: CollideConfig = { speculativeDistance: 0.02, linearSlop: 0.005 };

function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

describe('窄相随机一致性', () => {
  it('分离时流形最小分离 ≈ GJK 距离；无 NaN；法线为单位向量', () => {
    const rand = rng(1234);
    const shapes: Shape[] = [
      new SphereShape(0.4),
      new CapsuleShape(0.2, 0.4),
      new BoxShape(new Vec3(0.3, 0.5, 0.4)),
      new CylinderShape(0.4, 0.3, 12),
      new ConvexHullShape([
        new Vec3(0, 0.6, 0),
        new Vec3(0.5, -0.3, 0.2),
        new Vec3(-0.4, -0.3, 0.4),
        new Vec3(-0.1, -0.3, -0.5),
        new Vec3(0.2, 0.1, 0.3),
      ]),
    ];
    const m = new Manifold();
    const pa = new DistanceProxy();
    const pb = new DistanceProxy();
    const out = new DistanceOutput();
    let separatedChecks = 0;
    let contacts = 0;
    for (let iter = 0; iter < 4000; iter++) {
      const a = shapes[Math.floor(rand() * shapes.length)]!;
      const b = shapes[Math.floor(rand() * shapes.length)]!;
      const qa = new Quat().setFromEuler(rand() * 6, rand() * 6, rand() * 6);
      const qb = new Quat().setFromEuler(rand() * 6, rand() * 6, rand() * 6);
      const ta = new Transform(new Vec3(rand() - 0.5, rand() - 0.5, rand() - 0.5), qa);
      const tb = new Transform(new Vec3(rand() * 2 - 1, rand() * 2 - 1, rand() * 2 - 1), qb);
      collideShapes(a, ta, b, tb, m, config);
      pa.setShape(a, ta);
      pb.setShape(b, tb);
      const dist = gjkDistance(pa, pb, true, out).distance;
      if (m.pointCount === 0) {
        // 没有接触点：真实距离应大于推测距离（允许少量容差）
        expect(dist).toBeGreaterThan(config.speculativeDistance - 1e-6);
        continue;
      }
      contacts++;
      expect(Math.abs(m.normal.length() - 1)).toBeLessThan(1e-9);
      for (let i = 0; i < m.pointCount; i++) {
        const p = m.points[i]!;
        expect(Number.isFinite(p.separation)).toBe(true);
        expect(p.point.isFinite()).toBe(true);
      }
      if (dist > 0) {
        // 分离情形：流形最小分离应与真实距离一致
        separatedChecks++;
        expect(Math.abs(m.minSeparation() - dist)).toBeLessThan(1e-3);
      } else {
        expect(m.minSeparation()).toBeLessThan(1e-3);
      }
    }
    expect(contacts).toBeGreaterThan(500);
    expect(separatedChecks).toBeGreaterThan(5);
  });
});
