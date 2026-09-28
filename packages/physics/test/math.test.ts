import { describe, expect, it } from 'vitest';
import { AABB, Mat3, Quat, Transform, Vec3, computeBasis, wrapAngle } from '../src';

const EPS = 1e-9;

describe('Vec3', () => {
  it('基础运算', () => {
    const a = new Vec3(1, 2, 3);
    const b = new Vec3(4, 5, 6);
    expect(a.dot(b)).toBe(32);
    expect(new Vec3().crossVectors(a, b)).toEqual(new Vec3(-3, 6, -3));
    expect(a.clone().add(b)).toEqual(new Vec3(5, 7, 9));
    expect(a.clone().addScaled(b, 2)).toEqual(new Vec3(9, 12, 15));
    const n = new Vec3(3, 0, 4);
    expect(n.normalize()).toBe(5);
    expect(n.length()).toBeCloseTo(1, 12);
  });

  it('零向量归一化保持为零', () => {
    const v = new Vec3();
    expect(v.normalize()).toBe(0);
    expect(v).toEqual(new Vec3());
  });

  it('computeBasis 构造正交基', () => {
    for (const n of [new Vec3(0, 1, 0), new Vec3(1, 0, 0), new Vec3(1, 2, 3)]) {
      n.normalize();
      const t1 = new Vec3();
      const t2 = new Vec3();
      computeBasis(n, t1, t2);
      expect(Math.abs(n.dot(t1))).toBeLessThan(EPS);
      expect(Math.abs(n.dot(t2))).toBeLessThan(EPS);
      expect(Math.abs(t1.dot(t2))).toBeLessThan(EPS);
      expect(t1.length()).toBeCloseTo(1, 12);
      expect(t2.length()).toBeCloseTo(1, 12);
    }
  });
});

describe('Quat', () => {
  it('绕轴旋转向量', () => {
    const q = new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2);
    const v = q.rotate(new Vec3(1, 0, 0), new Vec3());
    expect(v.equals(new Vec3(0, 1, 0), EPS)).toBe(true);
    const back = q.invRotate(v, new Vec3());
    expect(back.equals(new Vec3(1, 0, 0), EPS)).toBe(true);
  });

  it('乘法组合旋转', () => {
    const qa = new Quat().setFromAxisAngle(new Vec3(0, 1, 0), 0.3);
    const qb = new Quat().setFromAxisAngle(new Vec3(1, 0, 0), -0.7);
    const q = new Quat().multiplyQuats(qa, qb);
    const v = new Vec3(0.2, -1, 3);
    const expected = qa.rotate(qb.rotate(v, new Vec3()), new Vec3());
    expect(q.rotate(v, new Vec3()).equals(expected, EPS)).toBe(true);
    const rel = new Quat().multiplyConjugateA(qa, q);
    expect(Math.abs(rel.dot(qb))).toBeCloseTo(1, 12);
  });

  it('按角速度积分', () => {
    const q = new Quat();
    const w = new Vec3(0, 2, 0);
    const dt = 1 / 1000;
    for (let i = 0; i < 1000; i++) q.integrate(w, dt);
    const expected = new Quat().setFromAxisAngle(new Vec3(0, 1, 0), 2);
    expect(Math.abs(q.dot(expected))).toBeCloseTo(1, 6);
  });

  it('旋转向量与欧拉角', () => {
    const q = new Quat().setFromAxisAngle(new Vec3(0, 1, 0), 1.2);
    const rv = q.toRotationVector(new Vec3());
    expect(rv.equals(new Vec3(0, 1.2, 0), 1e-12)).toBe(true);
    const e = new Quat().setFromEuler(0, 1.2, 0);
    expect(e.equals(q, 1e-12)).toBe(true);
    const u = new Quat().setFromUnitVectors(new Vec3(1, 0, 0), new Vec3(0, 0, -1));
    expect(u.rotate(new Vec3(1, 0, 0), new Vec3()).equals(new Vec3(0, 0, -1), EPS)).toBe(true);
    const opposite = new Quat().setFromUnitVectors(new Vec3(0, 1, 0), new Vec3(0, -1, 0));
    expect(opposite.rotate(new Vec3(0, 1, 0), new Vec3()).equals(new Vec3(0, -1, 0), EPS)).toBe(
      true,
    );
  });

  it('slerp', () => {
    const a = new Quat();
    const b = new Quat().setFromAxisAngle(new Vec3(0, 0, 1), 1);
    const m = new Quat().slerpQuats(a, b, 0.5);
    expect(m.getAngle()).toBeCloseTo(0.5, 12);
  });
});

describe('Mat3', () => {
  it('四元数转矩阵与四元数旋转一致', () => {
    const q = new Quat().setFromEuler(0.3, -0.4, 1.1);
    const m = new Mat3().setFromQuat(q);
    const v = new Vec3(1, -2, 0.5);
    expect(m.transformVector(v, new Vec3()).equals(q.rotate(v, new Vec3()), EPS)).toBe(true);
    expect(m.determinant()).toBeCloseTo(1, 12);
  });

  it('求逆与求解线性方程', () => {
    const m = new Mat3(4, 1, 0, 1, 3, 1, 0, 1, 2);
    const inv = m.clone();
    expect(inv.invert()).toBe(true);
    const id = new Mat3().multiplyMatrices(m, inv);
    expect(id.equals(new Mat3(), 1e-12)).toBe(true);
    const x = m.solve(new Vec3(1, 2, 3), new Vec3());
    expect(m.transformVector(x, new Vec3()).equals(new Vec3(1, 2, 3), 1e-12)).toBe(true);
    expect(new Mat3().setZero().invert()).toBe(false);
  });

  it('惯性张量旋转 R I Rᵀ', () => {
    const inertia = new Mat3().setDiagonal(1, 2, 3);
    const q = new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2);
    const world = new Mat3().setRotated(q, inertia);
    // 绕 Z 转 90°：x、y 主惯量互换
    expect(world.equals(new Mat3().setDiagonal(2, 1, 3), 1e-12)).toBe(true);
    const skew = new Mat3().setSkew(new Vec3(1, 2, 3));
    const u = new Vec3(-1, 0.5, 2);
    expect(
      skew
        .transformVector(u, new Vec3())
        .equals(new Vec3().crossVectors(new Vec3(1, 2, 3), u), EPS),
    ).toBe(true);
  });
});

describe('Transform', () => {
  it('点变换与逆变换', () => {
    const t = new Transform(new Vec3(1, 2, 3), new Quat().setFromEuler(0.1, 0.2, 0.3));
    const p = new Vec3(-1, 4, 2);
    const w = t.transformPoint(p, new Vec3());
    expect(t.inverseTransformPoint(w, new Vec3()).equals(p, EPS)).toBe(true);
  });

  it('组合与相对变换', () => {
    const a = new Transform(new Vec3(1, 0, 0), new Quat().setFromEuler(0, 0.5, 0));
    const b = new Transform(new Vec3(0, 2, 0), new Quat().setFromEuler(0.4, 0, 0));
    const ab = new Transform().multiplyTransforms(a, b);
    const p = new Vec3(0.3, 0.2, 0.1);
    const direct = a.transformPoint(b.transformPoint(p, new Vec3()), new Vec3());
    expect(ab.transformPoint(p, new Vec3()).equals(direct, EPS)).toBe(true);
    const rel = new Transform().multiplyInverseA(a, ab);
    expect(rel.position.equals(b.position, EPS)).toBe(true);
    expect(Math.abs(rel.rotation.dot(b.rotation))).toBeCloseTo(1, 12);
  });
});

describe('AABB', () => {
  it('相交、包含与合并', () => {
    const a = new AABB(new Vec3(0, 0, 0), new Vec3(1, 1, 1));
    const b = new AABB(new Vec3(0.5, 0.5, 0.5), new Vec3(2, 2, 2));
    const c = new AABB(new Vec3(3, 3, 3), new Vec3(4, 4, 4));
    expect(a.overlaps(b)).toBe(true);
    expect(a.overlaps(c)).toBe(false);
    const u = new AABB().union(a, c);
    expect(u.contains(a) && u.contains(c)).toBe(true);
    expect(a.surfaceArea()).toBe(6);
  });

  it('射线求交', () => {
    const box = new AABB(new Vec3(-1, -1, -1), new Vec3(1, 1, 1));
    expect(box.rayIntersect(new Vec3(-5, 0, 0), new Vec3(1, 0, 0), 100)).toBeCloseTo(4, 12);
    expect(box.rayIntersect(new Vec3(-5, 2, 0), new Vec3(1, 0, 0), 100)).toBe(-1);
    expect(box.rayIntersect(new Vec3(-5, 0, 0), new Vec3(1, 0, 0), 3)).toBe(-1);
    expect(box.rayIntersect(new Vec3(0, 0, 0), new Vec3(0, 1, 0), 100)).toBe(0);
  });
});

describe('MathUtil', () => {
  it('wrapAngle', () => {
    expect(wrapAngle(3 * Math.PI)).toBeCloseTo(Math.PI, 12);
    expect(wrapAngle(-3.5 * Math.PI)).toBeCloseTo(0.5 * Math.PI, 12);
  });
});
