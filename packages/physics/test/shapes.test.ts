import { describe, expect, it } from 'vitest';
import {
  AABB,
  BoxShape,
  CapsuleShape,
  ConvexHullShape,
  ConvexPolyhedron,
  CylinderShape,
  MassProperties,
  PlaneShape,
  Quat,
  type Shape,
  type ShapeRayHit,
  SphereShape,
  Transform,
  Vec3,
} from '../src';

function hit(): ShapeRayHit {
  return { t: 0, normal: new Vec3() };
}

/** 确定性的伪随机数（测试可复现） */
function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

/** 网格数值积分求质量属性，用于校验解析公式 */
function numericMass(inside: (p: Vec3) => boolean, half: Vec3, n = 64) {
  let count = 0;
  let ixx = 0,
    iyy = 0,
    izz = 0;
  const cell = new Vec3(half.x * 2 / n, half.y * 2 / n, half.z * 2 / n);
  const dv = cell.x * cell.y * cell.z;
  const p = new Vec3();
  for (let i = 0; i < n; i++)
    for (let j = 0; j < n; j++)
      for (let k = 0; k < n; k++) {
        p.set(-half.x + (i + 0.5) * cell.x, -half.y + (j + 0.5) * cell.y, -half.z + (k + 0.5) * cell.z);
        if (!inside(p)) continue;
        count++;
        ixx += p.y * p.y + p.z * p.z;
        iyy += p.x * p.x + p.z * p.z;
        izz += p.x * p.x + p.y * p.y;
      }
  return { mass: count * dv, ixx: ixx * dv, iyy: iyy * dv, izz: izz * dv };
}

function checkEuler(hull: ConvexPolyhedron) {
  expect(hull.vertices.length - hull.edges.length + hull.faces.length).toBe(2);
}

function checkConvex(hull: ConvexPolyhedron, tol = 1e-9) {
  for (const f of hull.faces) {
    for (const v of hull.vertices) {
      expect(f.normal.dot(v) - f.d).toBeLessThan(tol);
    }
  }
}

describe('ConvexPolyhedron', () => {
  it('长方体拓扑与质量属性', () => {
    const hull = ConvexPolyhedron.fromBox(1, 2, 3);
    expect(hull.vertices.length).toBe(8);
    expect(hull.faces.length).toBe(6);
    expect(hull.edges.length).toBe(12);
    checkEuler(hull);
    checkConvex(hull);
    const center = new Vec3();
    const inertia = new MassProperties().inertia;
    const volume = hull.computeMassProperties(center, inertia);
    expect(volume).toBeCloseTo(48, 10);
    expect(center.length()).toBeLessThan(1e-12);
    const box = new BoxShape(new Vec3(1, 2, 3)).computeMass(1, new MassProperties());
    expect(inertia.equals(box.inertia, 1e-9)).toBe(true);
  });

  it('凸包：去除内部点与重复点，并合并共面三角形', () => {
    const pts: Vec3[] = [];
    for (const x of [-1, 1]) for (const y of [-1, 1]) for (const z of [-1, 1]) pts.push(new Vec3(x, y, z));
    pts.push(new Vec3(0, 0, 0), new Vec3(0.5, -0.2, 0.1), new Vec3(1, 1, 1), new Vec3(1, 0, 0));
    const hull = ConvexPolyhedron.fromPoints(pts);
    expect(hull.vertices.length).toBe(8);
    expect(hull.faces.length).toBe(6);
    for (const f of hull.faces) expect(f.indices.length).toBe(4);
    checkEuler(hull);
    checkConvex(hull);
  });

  it('凸包：球面随机点', () => {
    const rand = rng(42);
    const pts: Vec3[] = [];
    for (let i = 0; i < 200; i++) {
      const v = new Vec3(rand() * 2 - 1, rand() * 2 - 1, rand() * 2 - 1);
      if (v.normalize() > 0) pts.push(v);
    }
    const hull = ConvexPolyhedron.fromPoints(pts);
    checkEuler(hull);
    checkConvex(hull, 1e-7);
    for (const p of pts) {
      for (const f of hull.faces) expect(f.normal.dot(p) - f.d).toBeLessThan(1e-6);
    }
    const volume = hull.computeMassProperties(new Vec3(), new MassProperties().inertia);
    expect(volume).toBeLessThan((4 / 3) * Math.PI);
    expect(volume).toBeGreaterThan(3.5);
  });

  it('退化点集抛出异常', () => {
    expect(() =>
      ConvexPolyhedron.fromPoints([new Vec3(0, 0, 0), new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(1, 1, 0)]),
    ).toThrow();
  });
});

describe('质量属性', () => {
  it('球体', () => {
    const m = new SphereShape(2).computeMass(3, new MassProperties());
    const mass = 3 * (4 / 3) * Math.PI * 8;
    expect(m.mass).toBeCloseTo(mass, 10);
    expect(m.inertia.m00).toBeCloseTo(0.4 * mass * 4, 10);
  });

  it('胶囊体与数值积分一致', () => {
    const cap = new CapsuleShape(0.5, 1);
    const m = cap.computeMass(1, new MassProperties());
    const num = numericMass((p) => {
      const y = Math.max(-1, Math.min(1, p.y));
      return p.x * p.x + (p.y - y) ** 2 + p.z * p.z <= 0.25;
    }, new Vec3(0.5, 1.5, 0.5), 96);
    expect(m.mass).toBeCloseTo(num.mass, 2);
    expect(Math.abs(m.inertia.m00 - num.ixx) / num.ixx).toBeLessThan(0.01);
    expect(Math.abs(m.inertia.m11 - num.iyy) / num.iyy).toBeLessThan(0.01);
  });

  it('圆柱体与凸包质量相近', () => {
    const cyl = new CylinderShape(1, 0.5, 64);
    const m = cyl.computeMass(1, new MassProperties());
    const hullMass = cyl.hull.computeMassProperties(new Vec3(), new MassProperties().inertia);
    expect(Math.abs(hullMass - m.mass) / m.mass).toBeLessThan(0.01);
    expect(cyl.hull.faces.length).toBe(64 + 2);
    checkEuler(cyl.hull);
    checkConvex(cyl.hull);
  });

  it('偏心凸包的质心', () => {
    const pts = ConvexPolyhedron.fromBox(1, 1, 1).vertices.map((v) => v.clone().add(new Vec3(3, 0, 0)));
    const m = new ConvexHullShape(pts).computeMass(2, new MassProperties());
    expect(m.mass).toBeCloseTo(16, 10);
    expect(m.center.equals(new Vec3(3, 0, 0), 1e-9)).toBe(true);
    expect(m.inertia.m00).toBeCloseTo((16 * 2) / 3, 9);
  });
});

describe('包围盒', () => {
  it('旋转长方体', () => {
    const box = new BoxShape(new Vec3(1, 2, 3));
    const t = new Transform(new Vec3(5, 0, 0), new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2));
    const aabb = box.computeAABB(t, new AABB());
    expect(aabb.min.equals(new Vec3(3, -1, -3), 1e-9)).toBe(true);
    expect(aabb.max.equals(new Vec3(7, 1, 3), 1e-9)).toBe(true);
  });

  it('各形状包围盒包含全部核心点', () => {
    const shapes: Shape[] = [
      new SphereShape(0.5),
      new CapsuleShape(0.3, 0.8),
      new BoxShape(new Vec3(0.2, 0.4, 0.6)),
      new CylinderShape(0.5, 0.5),
      new ConvexHullShape([new Vec3(0, 0, 0), new Vec3(1, 0, 0), new Vec3(0, 1, 0), new Vec3(0, 0, 1)]),
    ];
    const t = new Transform(new Vec3(1, 2, 3), new Quat().setFromEuler(0.3, 0.7, -0.2));
    const p = new Vec3();
    for (const s of shapes) {
      const aabb = s.computeAABB(t, new AABB()).expandByScalar(1e-9);
      for (const c of s.getCorePoints()) {
        expect(aabb.containsPoint(t.transformPoint(c, p))).toBe(true);
      }
    }
  });

  it('平面包围盒按法线收紧', () => {
    const aabb = new PlaneShape().computeAABB(new Transform(new Vec3(0, -1, 0)), new AABB());
    expect(aabb.max.y).toBe(-1);
  });
});

describe('射线检测', () => {
  it('球体', () => {
    const h = hit();
    expect(new SphereShape(1).raycast(new Vec3(-5, 0, 0), new Vec3(1, 0, 0), 10, h)).toBe(true);
    expect(h.t).toBeCloseTo(4, 12);
    expect(h.normal.equals(new Vec3(-1, 0, 0), 1e-12)).toBe(true);
    expect(new SphereShape(1).raycast(new Vec3(0, 0, 0), new Vec3(1, 0, 0), 10, h)).toBe(false);
  });

  it('胶囊体侧面与端部', () => {
    const cap = new CapsuleShape(0.5, 1);
    const h = hit();
    expect(cap.raycast(new Vec3(-5, 0.5, 0), new Vec3(1, 0, 0), 10, h)).toBe(true);
    expect(h.t).toBeCloseTo(4.5, 12);
    expect(cap.raycast(new Vec3(0, 5, 0), new Vec3(0, -1, 0), 10, h)).toBe(true);
    expect(h.t).toBeCloseTo(3.5, 12);
    expect(h.normal.equals(new Vec3(0, 1, 0), 1e-12)).toBe(true);
    expect(cap.raycast(new Vec3(-5, 3, 0), new Vec3(1, 0, 0), 10, h)).toBe(false);
  });

  it('长方体与凸包', () => {
    const box = new BoxShape(new Vec3(1, 1, 1));
    const h = hit();
    expect(box.raycast(new Vec3(0, 5, 0.3), new Vec3(0, -2, 0), 10, h)).toBe(true);
    expect(h.t).toBeCloseTo(2, 12);
    expect(h.normal.equals(new Vec3(0, 1, 0), 1e-12)).toBe(true);
    expect(box.raycast(new Vec3(0, 5, 0), new Vec3(0, -1, 0), 3, h)).toBe(false);
    expect(box.raycast(new Vec3(0, 0, 0), new Vec3(0, -1, 0), 3, h)).toBe(false);
    const cyl = new CylinderShape(1, 1);
    expect(cyl.raycast(new Vec3(-5, 0, 0), new Vec3(1, 0, 0), 10, h)).toBe(true);
    expect(h.t).toBeCloseTo(4, 12);
  });

  it('平面', () => {
    const h = hit();
    expect(new PlaneShape().raycast(new Vec3(0, 3, 0), new Vec3(0, -1, 0), 10, h)).toBe(true);
    expect(h.t).toBeCloseTo(3, 12);
    expect(new PlaneShape().raycast(new Vec3(0, 3, 0), new Vec3(0, 1, 0), 10, h)).toBe(false);
  });
});
