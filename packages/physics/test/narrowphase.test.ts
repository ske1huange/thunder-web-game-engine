import { describe, expect, it } from 'vitest';
import {
  BoxShape,
  CapsuleShape,
  type CollideConfig,
  CylinderShape,
  DistanceOutput,
  DistanceProxy,
  Manifold,
  PlaneShape,
  Quat,
  type Shape,
  SphereShape,
  Transform,
  Vec3,
  collideShapes,
  gjkDistance,
  testOverlap,
} from '../src';

const config: CollideConfig = { speculativeDistance: 0.02, linearSlop: 0.005 };

function xf(x: number, y: number, z: number, q?: Quat): Transform {
  return new Transform(new Vec3(x, y, z), q);
}

function collide(a: Shape, ta: Transform, b: Shape, tb: Transform): Manifold {
  const m = new Manifold();
  collideShapes(a, ta, b, tb, m, config);
  return m;
}

function distance(a: Shape, ta: Transform, b: Shape, tb: Transform): DistanceOutput {
  const pa = new DistanceProxy().setShape(a, ta);
  const pb = new DistanceProxy().setShape(b, tb);
  return gjkDistance(pa, pb, true, new DistanceOutput());
}

const unitBox = new BoxShape(new Vec3(0.5, 0.5, 0.5));

describe('GJK 距离', () => {
  it('球-球', () => {
    const out = distance(new SphereShape(1), xf(0, 0, 0), new SphereShape(0.5), xf(3, 0, 0));
    expect(out.distance).toBeCloseTo(1.5, 10);
    expect(out.normal.equals(new Vec3(1, 0, 0), 1e-10)).toBe(true);
    expect(out.pointA.equals(new Vec3(1, 0, 0), 1e-10)).toBe(true);
    expect(out.pointB.equals(new Vec3(2.5, 0, 0), 1e-10)).toBe(true);
  });

  it('长方体-长方体（顶点到面、棱到棱）', () => {
    let out = distance(unitBox, xf(0, 0, 0), unitBox, xf(0, 2, 0));
    expect(out.distance).toBeCloseTo(1, 10);
    const q = new Quat().setFromEuler(0, Math.PI / 4, Math.PI / 4);
    out = distance(unitBox, xf(0, 0, 0), unitBox, xf(3, 0.2, 0.1, q));
    expect(out.distance).toBeGreaterThan(0);
    // 与最近点的距离一致
    expect(out.pointA.distanceTo(out.pointB)).toBeCloseTo(out.distance, 10);
  });

  it('重叠时距离为 0', () => {
    expect(distance(unitBox, xf(0, 0, 0), unitBox, xf(0.3, 0.2, 0)).distance).toBe(0);
    expect(distance(new SphereShape(0.1), xf(0, 0, 0), unitBox, xf(0, 0, 0)).distance).toBe(0);
  });

  it('胶囊-长方体', () => {
    const out = distance(new CapsuleShape(0.25, 1), xf(0, 0, 0), unitBox, xf(2, 0, 0));
    expect(out.distance).toBeCloseTo(1.25, 10);
  });
});

describe('接触流形', () => {
  it('球-球：分离时无接触，穿透时一个点', () => {
    const s = new SphereShape(0.5);
    expect(collide(s, xf(0, 0, 0), s, xf(1.1, 0, 0)).pointCount).toBe(0);
    const m = collide(s, xf(0, 0, 0), s, xf(0.9, 0, 0));
    expect(m.pointCount).toBe(1);
    expect(m.normal.equals(new Vec3(1, 0, 0), 1e-12)).toBe(true);
    expect(m.points[0]!.separation).toBeCloseTo(-0.1, 12);
    expect(m.points[0]!.point.equals(new Vec3(0.45, 0, 0), 1e-12)).toBe(true);
    // 推测接触
    expect(collide(s, xf(0, 0, 0), s, xf(1.01, 0, 0)).pointCount).toBe(1);
  });

  it('平行胶囊：两个接触点', () => {
    const c = new CapsuleShape(0.25, 1);
    const lying = new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2);
    const m = collide(c, xf(0, 0, 0, lying), c, xf(0.5, 0.45, 0, lying));
    expect(m.pointCount).toBe(2);
    expect(m.normal.equals(new Vec3(0, 1, 0), 1e-9)).toBe(true);
    for (let i = 0; i < 2; i++) expect(m.points[i]!.separation).toBeCloseTo(-0.05, 9);
  });

  it('长方体叠放：4 个接触点', () => {
    const m = collide(unitBox, xf(0, 0, 0), unitBox, xf(0.1, 0.99, 0.05));
    expect(m.pointCount).toBe(4);
    expect(m.normal.equals(new Vec3(0, 1, 0), 1e-9)).toBe(true);
    for (let i = 0; i < 4; i++) {
      expect(m.points[i]!.separation).toBeCloseTo(-0.01, 9);
      expect(m.points[i]!.point.y).toBeCloseTo(0.495, 9);
    }
  });

  it('交换顺序时法线取反', () => {
    const a = collide(unitBox, xf(0, 0, 0), unitBox, xf(0, 0.99, 0));
    const b = collide(unitBox, xf(0, 0.99, 0), unitBox, xf(0, 0, 0));
    expect(b.normal.equals(a.normal.clone().negate(), 1e-12)).toBe(true);
    expect(b.pointCount).toBe(a.pointCount);
  });

  it('旋转 45° 的长方体叠放：约减到 4 个点', () => {
    const q = new Quat().setFromAxisAngle(new Vec3(0, 1, 0), Math.PI / 4);
    const m = collide(unitBox, xf(0, 0, 0), unitBox, xf(0, 0.99, 0, q));
    expect(m.pointCount).toBe(4);
    expect(m.normal.y).toBeGreaterThan(0.999);
  });

  it('棱-棱接触：一个点', () => {
    const qa = new Quat().setFromAxisAngle(new Vec3(1, 0, 0), Math.PI / 4);
    const qb = new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4);
    const h = Math.SQRT1_2;
    const m = collide(unitBox, xf(0, 0, 0, qa), unitBox, xf(0, 2 * h - 0.01, 0, qb));
    expect(m.pointCount).toBe(1);
    expect(m.normal.equals(new Vec3(0, 1, 0), 1e-6)).toBe(true);
    expect(m.points[0]!.separation).toBeCloseTo(-0.01, 6);
  });

  it('球-长方体：面、角与中心在内部', () => {
    const s = new SphereShape(0.25);
    let m = collide(unitBox, xf(0, 0, 0), s, xf(0.2, 0.7, 0));
    expect(m.pointCount).toBe(1);
    expect(m.normal.equals(new Vec3(0, 1, 0), 1e-9)).toBe(true);
    expect(m.points[0]!.separation).toBeCloseTo(-0.05, 9);
    // 角
    m = collide(unitBox, xf(0, 0, 0), s, xf(0.6, 0.6, 0.6));
    expect(m.pointCount).toBe(1);
    const diag = new Vec3(1, 1, 1);
    diag.normalize();
    expect(m.normal.equals(diag, 1e-6)).toBe(true);
    expect(m.points[0]!.separation).toBeCloseTo(Math.sqrt(3) * 0.1 - 0.25, 6);
    // 中心在内部
    m = collide(s, xf(0.3, 0.1, 0), unitBox, xf(0, 0, 0));
    expect(m.pointCount).toBe(1);
    expect(m.normal.equals(new Vec3(-1, 0, 0), 1e-9)).toBe(true);
    expect(m.points[0]!.separation).toBeCloseTo(-(0.2 + 0.25), 9);
  });

  it('胶囊平躺在长方体上：两个点；竖立：一个点', () => {
    const c = new CapsuleShape(0.2, 0.3);
    const lying = new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2);
    let m = collide(c, xf(0, 0.69, 0, lying), unitBox, xf(0, 0, 0));
    expect(m.pointCount).toBe(2);
    expect(m.normal.equals(new Vec3(0, -1, 0), 1e-9)).toBe(true);
    for (let i = 0; i < 2; i++) expect(m.points[i]!.separation).toBeCloseTo(-0.01, 9);
    m = collide(c, xf(0, 0.99, 0), unitBox, xf(0, 0, 0));
    expect(m.pointCount).toBe(1);
    expect(m.points[0]!.separation).toBeCloseTo(-0.01, 9);
  });

  it('胶囊深度穿透长方体（SAT 分支）', () => {
    const c = new CapsuleShape(0.1, 0.3);
    const lying = new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2);
    const m = collide(unitBox, xf(0, 0, 0), c, xf(0, 0.45, 0, lying));
    expect(m.pointCount).toBe(2);
    expect(m.normal.equals(new Vec3(0, 1, 0), 1e-9)).toBe(true);
    for (let i = 0; i < 2; i++) expect(m.points[i]!.separation).toBeCloseTo(-0.15, 9);
  });

  it('平面', () => {
    const plane = new PlaneShape();
    let m = collide(plane, xf(0, 0, 0), unitBox, xf(0, 0.49, 0));
    expect(m.pointCount).toBe(4);
    expect(m.normal.equals(new Vec3(0, 1, 0), 1e-12)).toBe(true);
    m = collide(new SphereShape(0.5), xf(0, 0.4, 0), plane, xf(0, 0, 0));
    expect(m.pointCount).toBe(1);
    expect(m.normal.equals(new Vec3(0, -1, 0), 1e-12)).toBe(true);
    expect(m.points[0]!.separation).toBeCloseTo(-0.1, 12);
    const cyl = new CylinderShape(0.5, 0.5);
    m = collide(plane, xf(0, 0, 0), cyl, xf(0, 0.49, 0));
    expect(m.pointCount).toBe(4);
    const lying = new Quat().setFromAxisAngle(new Vec3(1, 0, 0), Math.PI / 2);
    m = collide(plane, xf(0, 0, 0), new CapsuleShape(0.2, 0.5), xf(0, 0.19, 0, lying));
    expect(m.pointCount).toBe(2);
  });

  it('圆柱站立在长方体上：面接触', () => {
    const cyl = new CylinderShape(0.3, 0.5);
    const m = collide(unitBox, xf(0, 0, 0), cyl, xf(0, 0.99, 0));
    expect(m.pointCount).toBe(4);
    expect(m.normal.y).toBeGreaterThan(0.999);
  });

  it('重叠测试', () => {
    const s = new SphereShape(0.5);
    expect(testOverlap(s, xf(0, 0, 0), unitBox, xf(0.9, 0, 0))).toBe(true);
    expect(testOverlap(s, xf(0, 0, 0), unitBox, xf(1.1, 0, 0))).toBe(false);
    expect(testOverlap(new PlaneShape(), xf(0, 0, 0), s, xf(0, 0.4, 0))).toBe(true);
    expect(testOverlap(new PlaneShape(), xf(0, 0, 0), s, xf(0, 0.6, 0))).toBe(false);
  });
});
