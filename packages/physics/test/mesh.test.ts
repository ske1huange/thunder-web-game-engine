import { describe, expect, it } from 'vitest';
import {
  AABB,
  ALL_EDGES_ACTIVE,
  BoxShape,
  CapsuleShape,
  ConvexHullShape,
  HeightfieldShape,
  MeshBVH,
  PlaneShape,
  Quat,
  type RigidBody,
  type ShapeRayHit,
  SphereShape,
  Transform,
  TriMeshShape,
  Vec3,
  World,
  rayTriangle,
  triangleAabb,
} from '../src';

const DT = 1 / 60;

/** 可复现的伪随机数 */
function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

/** n×n 个格子的平坦网格（每格两个三角形），中心在原点 */
function gridMesh(n: number, size: number, height = (_x: number, _z: number) => 0) {
  const verts: number[] = [];
  const idx: number[] = [];
  const step = size / n;
  for (let r = 0; r <= n; r++) {
    for (let c = 0; c <= n; c++) {
      const x = -size / 2 + c * step;
      const z = -size / 2 + r * step;
      verts.push(x, height(x, z), z);
    }
  }
  for (let r = 0; r < n; r++) {
    for (let c = 0; c < n; c++) {
      const i = r * (n + 1) + c;
      // 从上方看逆时针（法线 +Y）
      idx.push(i, i + n + 1, i + n + 2, i, i + n + 2, i + 1);
    }
  }
  return { verts, idx };
}

function meshWorld(shape: TriMeshShape | HeightfieldShape, options = {}): World {
  const world = new World(options);
  const ground = world.createBody({ type: 'static' });
  ground.addCollider({ shape, friction: 0.6 });
  return world;
}

describe('MeshBVH', () => {
  it('AABB 查询与射线结果与暴力法一致', () => {
    const rand = rng(7);
    const n = 500;
    const tris: Vec3[][] = [];
    const bounds = new Float64Array(n * 6);
    const box = new AABB();
    for (let i = 0; i < n; i++) {
      const c = new Vec3(rand() * 40 - 20, rand() * 10, rand() * 40 - 20);
      const t = [0, 1, 2].map(() =>
        new Vec3(rand() - 0.5, rand() - 0.5, rand() - 0.5).scale(3).add(c),
      );
      tris.push(t);
      triangleAabb(t[0]!, t[1]!, t[2]!, box);
      bounds.set([box.min.x, box.min.y, box.min.z, box.max.x, box.max.y, box.max.z], i * 6);
    }
    const bvh = new MeshBVH(bounds, n);
    expect(bvh.depth).toBeLessThan(12);

    for (let q = 0; q < 50; q++) {
      const c = new Vec3(rand() * 40 - 20, rand() * 10, rand() * 40 - 20);
      const query = new AABB(c, c).expandByScalar(rand() * 4);
      const got: number[] = [];
      bvh.query(query, (t) => {
        got.push(t);
      });
      const expected: number[] = [];
      for (let i = 0; i < n; i++) {
        triangleAabb(tris[i]![0]!, tris[i]![1]!, tris[i]![2]!, box);
        if (box.overlaps(query)) expected.push(i);
      }
      // BVH 返回叶子中的候选三角形：必须包含所有真正相交的三角形
      const set = new Set(got);
      for (const e of expected) expect(set.has(e)).toBe(true);
      expect(got.length).toBeLessThan(expected.length + 40);
    }

    const hit: ShapeRayHit = { t: 0, normal: new Vec3() };
    for (let q = 0; q < 50; q++) {
      const origin = new Vec3(rand() * 60 - 30, rand() * 20 - 5, rand() * 60 - 30);
      const dir = new Vec3(rand() - 0.5, rand() - 0.5, rand() - 0.5);
      dir.normalize();
      let bvhBest = Infinity;
      bvh.raycast(origin, dir, 100, (t, maxT) => {
        const tri = tris[t]!;
        if (!rayTriangle(origin, dir, tri[0]!, tri[1]!, tri[2]!, maxT, true, hit)) return -1;
        bvhBest = Math.min(bvhBest, hit.t);
        return hit.t;
      });
      let bruteBest = Infinity;
      for (const tri of tris) {
        if (rayTriangle(origin, dir, tri[0]!, tri[1]!, tri[2]!, 100, true, hit)) {
          bruteBest = Math.min(bruteBest, hit.t);
        }
      }
      expect(bvhBest).toBe(bruteBest);
    }
  });
});

describe('TriMeshShape', () => {
  it('焊接重复顶点并计算活跃棱', () => {
    // 两个三角形组成的平坦正方形，顶点未共享（模拟导出的模型）
    const flat = new TriMeshShape([
      new Vec3(0, 0, 0),
      new Vec3(0, 0, 1),
      new Vec3(1, 0, 1),
      new Vec3(0, 0, 0),
      new Vec3(1, 0, 1),
      new Vec3(1, 0, 0),
    ]);
    expect(flat.positions.length).toBe(4 * 3);
    // 第一个三角形的 c→a 棱、第二个三角形的 a→b 棱是内部对角线：非活跃
    expect(flat.getEdgeFlags(0)).toBe(ALL_EDGES_ACTIVE & ~4);
    expect(flat.getEdgeFlags(1)).toBe(ALL_EDGES_ACTIVE & ~1);

    // 屋脊（凸）→ 活跃；山谷（凹）→ 非活跃
    const ridge = new TriMeshShape(
      [0, -1, 0, 0, -1, 1, 1, 0, 1, 1, 0, 0, 2, -1, 0, 2, -1, 1],
      [0, 1, 2, 0, 2, 3, 3, 2, 5, 3, 5, 4],
    );
    // 三角形 1 的 b→c（2→3）与三角形 2 的 a→b（3→2）是屋脊线
    expect(ridge.getEdgeFlags(1) & 2).toBe(2);
    const valley = new TriMeshShape(
      [0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 2, 1, 0, 2, 1, 1],
      [0, 1, 2, 0, 2, 3, 3, 2, 5, 3, 5, 4],
    );
    expect(valley.getEdgeFlags(1) & 2).toBe(0);
  });

  it('局部射线检测：单面只命中正面，双面都命中', () => {
    const { verts, idx } = gridMesh(8, 8);
    const oneSided = new TriMeshShape(verts, idx);
    const twoSided = new TriMeshShape(verts, idx, { doubleSided: true });
    const hit: ShapeRayHit = { t: 0, normal: new Vec3() };
    expect(oneSided.raycast(new Vec3(0.3, 5, 0.7), new Vec3(0, -1, 0), 10, hit)).toBe(true);
    expect(hit.t).toBeCloseTo(5, 9);
    expect(hit.normal.y).toBeCloseTo(1, 9);
    expect(oneSided.raycast(new Vec3(0.3, -5, 0.7), new Vec3(0, 1, 0), 10, hit)).toBe(false);
    expect(twoSided.raycast(new Vec3(0.3, -5, 0.7), new Vec3(0, 1, 0), 10, hit)).toBe(true);
    expect(hit.normal.y).toBeCloseTo(-1, 9);
  });
});

describe('HeightfieldShape', () => {
  const rows = 17;
  const cols = 21;
  const heights = new Float64Array(rows * cols);
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      heights[r * cols + c] = Math.sin(c * 0.4) * Math.cos(r * 0.3) * 1.5;
    }
  }
  const field = new HeightfieldShape({ heights, rows, cols, cellSize: { x: 0.8, z: 1.2 } });

  it('尺寸、包围盒与 heightAt', () => {
    expect(field.triangleCount).toBe((rows - 1) * (cols - 1) * 2);
    expect(field.localBounds.min.x).toBeCloseTo(-8, 9);
    expect(field.localBounds.max.z).toBeCloseTo(9.6, 9);
    const v = field.getVertex(5, 7, new Vec3());
    expect(field.heightAt(v.x, v.z)).toBeCloseTo(v.y, 9);
    expect(Number.isNaN(field.heightAt(100, 0))).toBe(true);
  });

  it('DDA 射线检测与逐三角形暴力法一致', () => {
    const rand = rng(3);
    const a = new Vec3();
    const b = new Vec3();
    const c = new Vec3();
    const hit: ShapeRayHit = { t: 0, normal: new Vec3() };
    const out: ShapeRayHit = { t: 0, normal: new Vec3() };
    let hits = 0;
    for (let q = 0; q < 200; q++) {
      const origin = new Vec3(rand() * 24 - 12, 3 + rand() * 4, rand() * 26 - 13);
      const dir = new Vec3(rand() - 0.5, -rand(), rand() - 0.5);
      if (q % 10 === 0) dir.set(0, -1, 0);
      dir.normalize();
      let best = Infinity;
      for (let t = 0; t < field.triangleCount; t++) {
        field.getTriangle(t, a, b, c);
        if (rayTriangle(origin, dir, a, b, c, 50, false, hit)) best = Math.min(best, hit.t);
      }
      const got = field.raycast(origin, dir, 50, out);
      expect(got).toBe(best < Infinity);
      if (got) {
        hits++;
        expect(out.t).toBeCloseTo(best, 9);
        expect(out.normal.y).toBeGreaterThan(0);
      }
    }
    expect(hits).toBeGreaterThan(50);
  });

  it('平坦高度场的内部棱非活跃、边界棱活跃', () => {
    const flat = new HeightfieldShape({ heights: new Float64Array(16), rows: 4, cols: 4 });
    // 中间格子（row 1, col 1）两个三角形：全部内部棱
    const cell = (1 * 3 + 1) * 2;
    expect(flat.getEdgeFlags(cell)).toBe(0);
    expect(flat.getEdgeFlags(cell + 1)).toBe(0);
    // 角落格子（row 0, col 0）：三角形 0 的 a→b（x = 0 边界）活跃
    expect(flat.getEdgeFlags(0) & 1).toBe(1);
    // 三角形 1 的 c→a（z = 0 边界）活跃
    expect(flat.getEdgeFlags(1) & 4).toBe(4);
  });
});

describe('与网格的碰撞', () => {
  it('长方体落在三角网格地面上静止，高度正确并休眠', () => {
    const { verts, idx } = gridMesh(10, 20);
    const world = meshWorld(new TriMeshShape(verts, idx));
    const box = world.createBody({ position: new Vec3(0.3, 3, -0.2) });
    box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
    for (let i = 0; i < 300; i++) world.step(DT);
    expect(box.position.y).toBeGreaterThan(0.48);
    expect(box.position.y).toBeLessThan(0.51);
    expect(Math.abs(box.position.x - 0.3)).toBeLessThan(0.01);
    expect(box.isAwake).toBe(false);
  });

  it('在三角形接缝上滑行不会被“幽灵棱”绊住', () => {
    // 细密网格：箱子会跨过很多条内部棱
    const { verts, idx } = gridMesh(40, 40);
    const world = meshWorld(new TriMeshShape(verts, idx), { enableSleep: false });
    const box = world.createBody({
      position: new Vec3(-15, 0.5, 0.25),
      linearVelocity: new Vec3(12, 0, 3),
    });
    box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)), friction: 0.1 });
    const sphere = world.createBody({
      position: new Vec3(-15, 0.4, -5),
      linearVelocity: new Vec3(10, 0, 0),
    });
    sphere.addCollider({ shape: new SphereShape(0.4), friction: 0.1 });
    let maxVy = 0;
    let maxTilt = 0;
    let maxSphereUp = 0;
    let maxSphereY = 0;
    for (let i = 0; i < 120; i++) {
      world.step(DT);
      maxVy = Math.max(maxVy, Math.abs(box.linearVelocity.y));
      // 高速滚动的球在平面上也有轻微下沉（锚点随刚体旋转），这里只检查被接缝“弹起”
      maxSphereUp = Math.max(maxSphereUp, sphere.linearVelocity.y);
      maxSphereY = Math.max(maxSphereY, sphere.position.y);
      // 箱子的上方向与 +Y 的夹角
      const up = box.rotation.rotate(new Vec3(0, 1, 0), new Vec3());
      maxTilt = Math.max(maxTilt, Math.acos(Math.min(1, up.y)));
    }
    expect(box.position.x).toBeGreaterThan(-5);
    expect(maxVy).toBeLessThan(0.05);
    expect(maxTilt).toBeLessThan(0.01);
    expect(maxSphereUp).toBeLessThan(0.05);
    expect(maxSphereY).toBeLessThan(0.401);
    expect(sphere.position.y).toBeGreaterThan(0.39);
  });

  it('多个流形：箱子卡在 V 形槽中', () => {
    // 两个 45° 斜面组成的 V 形槽
    const verts = [-2, 2, -2, -2, 2, 2, 0, 0, 2, 0, 0, -2, 2, 2, 2, 2, 2, -2];
    const idx = [0, 1, 2, 0, 2, 3, 3, 2, 4, 3, 4, 5];
    const world = meshWorld(new TriMeshShape(verts, idx));
    const box = world.createBody({
      position: new Vec3(0, 3, 0),
      rotation: new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 4),
    });
    box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
    for (let i = 0; i < 180; i++) world.step(DT);
    const contact = world.contacts.find((c) => c.touching)!;
    expect(contact.manifoldCount).toBe(2);
    // 箱子对角线竖直、底角插在槽里：中心高度 ≈ 0.5·√2
    expect(box.position.y).toBeGreaterThan(Math.SQRT1_2 - 0.02);
    expect(box.position.y).toBeLessThan(Math.SQRT1_2 + 0.02);
    expect(Math.abs(box.position.x)).toBeLessThan(0.01);
  });

  it('球在高度场斜坡上滚下且始终不穿透地面', () => {
    const rows = 33;
    const cols = 33;
    const heights = new Float64Array(rows * cols);
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) heights[r * cols + c] = (32 - c) * 0.3 + Math.sin(r) * 0.2;
    }
    const field = new HeightfieldShape({ heights, rows, cols, cellSize: 1 });
    const world = meshWorld(field);
    const ball = world.createBody({ position: new Vec3(-12, field.heightAt(-12, 0.3) + 1, 0.3) });
    ball.addCollider({ shape: new SphereShape(0.5) });
    const capsule = world.createBody({
      position: new Vec3(-12, field.heightAt(-12, 4) + 1, 4),
      rotation: new Quat().setFromAxisAngle(new Vec3(1, 0, 0), Math.PI / 2),
    });
    capsule.addCollider({ shape: new CapsuleShape(0.3, 0.5) });
    let minClearance = Infinity;
    for (let i = 0; i < 240; i++) {
      world.step(DT);
      const p = ball.position;
      const ground = field.heightAt(p.x, p.z);
      if (!Number.isNaN(ground)) minClearance = Math.min(minClearance, p.y - ground);
    }
    expect(ball.position.x).toBeGreaterThan(-6);
    expect(capsule.position.x).toBeGreaterThan(-8);
    // 斜坡上球心到地面的竖直距离 ≥ 半径（略小于半径表示穿透）
    expect(minClearance).toBeGreaterThan(0.45);
  });

  it('箱子堆叠与凸包在网格上稳定', () => {
    const { verts, idx } = gridMesh(6, 12);
    const world = meshWorld(new TriMeshShape(verts, idx));
    const bodies: RigidBody[] = [];
    for (let i = 0; i < 5; i++) {
      const b = world.createBody({ position: new Vec3(0.1, 0.5 + i * 1.02, 0.2) });
      b.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
      bodies.push(b);
    }
    const hull = world.createBody({ position: new Vec3(3, 1, 0) });
    hull.addCollider({
      shape: new ConvexHullShape([
        new Vec3(-0.5, 0, -0.5),
        new Vec3(0.5, 0, -0.5),
        new Vec3(0, 0, 0.5),
        new Vec3(0, 0.8, 0),
      ]),
    });
    for (let i = 0; i < 400; i++) world.step(DT);
    for (let i = 0; i < 5; i++) {
      expect(Math.abs(bodies[i]!.position.x - 0.1)).toBeLessThan(0.03);
      expect(bodies[i]!.position.y).toBeCloseTo(0.5 + i, 1);
    }
    expect(hull.position.y).toBeGreaterThan(-0.01);
    expect(hull.isAwake).toBe(false);
  });

  it('单面网格：从背面可以穿过，双面网格会挡住', () => {
    for (const doubleSided of [false, true]) {
      const { verts, idx } = gridMesh(4, 8);
      const world = new World({ gravity: new Vec3(0, 0, 0) });
      const ground = world.createBody({ type: 'static', position: new Vec3(0, 2, 0) });
      ground.addCollider({ shape: new TriMeshShape(verts, idx, { doubleSided }) });
      const ball = world.createBody({
        position: new Vec3(0.2, 0, 0.3),
        linearVelocity: new Vec3(0, 4, 0),
      });
      ball.addCollider({ shape: new SphereShape(0.3) });
      for (let i = 0; i < 60; i++) world.step(DT);
      if (doubleSided) expect(ball.position.y).toBeLessThan(2 - 0.29);
      else expect(ball.position.y).toBeGreaterThan(3);
    }
  });

  it('连续碰撞：高速小球不会穿过薄网格地面', () => {
    const { verts, idx } = gridMesh(4, 8);
    const world = new World();
    const ground = world.createBody({ type: 'static' });
    ground.addCollider({ shape: new TriMeshShape(verts, idx) });
    const hf = world.createBody({ type: 'static', position: new Vec3(20, 0, 0) });
    hf.addCollider({
      shape: new HeightfieldShape({ heights: new Float64Array(25), rows: 5, cols: 5 }),
    });
    const bullets = [0, 20].map((x) => {
      const b = world.createBody({
        position: new Vec3(x + 0.1, 3, 0.2),
        linearVelocity: new Vec3(0, -200, 0),
      });
      b.addCollider({ shape: new SphereShape(0.1), restitution: 0 });
      return b;
    });
    for (let i = 0; i < 60; i++) world.step(DT);
    for (const b of bullets) {
      expect(b.position.y).toBeGreaterThan(0.05);
      expect(b.position.y).toBeLessThan(0.2);
    }
  });

  it('运动学网格平台托着物体移动', () => {
    const { verts, idx } = gridMesh(2, 4);
    const world = new World();
    const platform = world.createBody({ type: 'kinematic', position: new Vec3(0, 1, 0) });
    platform.addCollider({ shape: new TriMeshShape(verts, idx) });
    const box = world.createBody({ position: new Vec3(0, 1.6, 0) });
    box.addCollider({ shape: new BoxShape(new Vec3(0.3, 0.3, 0.3)), friction: 1 });
    for (let i = 0; i < 60; i++) world.step(DT);
    platform.setLinearVelocity(new Vec3(1, 0, 0));
    for (let i = 0; i < 60; i++) world.step(DT);
    expect(box.position.x).toBeGreaterThan(0.85);
    expect(box.position.y).toBeGreaterThan(1.25);
  });
});

describe('网格查询', () => {
  const { verts, idx } = gridMesh(8, 16, (x, z) => 0.1 * x + 0.05 * z);
  const world = new World();
  const body = world.createBody({ type: 'static', position: new Vec3(0, 1, 0) });
  const mesh = new TriMeshShape(verts, idx);
  body.addCollider({ shape: mesh });
  const hfBody = world.createBody({ type: 'static', position: new Vec3(30, 0, 0) });
  hfBody.addCollider({
    shape: new HeightfieldShape({ heights: new Float64Array(9).fill(2), rows: 3, cols: 3 }),
  });

  it('world.raycast 命中三角网格与高度场', () => {
    const hit = world.raycast(new Vec3(2, 10, 1), new Vec3(0, -1, 0))!;
    expect(hit).not.toBeNull();
    expect(hit.point.y).toBeCloseTo(1 + 0.1 * 2 + 0.05 * 1, 9);
    expect(hit.normal.y).toBeGreaterThan(0.99);
    const hit2 = world.raycast(new Vec3(30.5, 10, 0.2), new Vec3(0, -1, 0))!;
    expect(hit2.point.y).toBeCloseTo(2, 9);
    expect(hit2.body).toBe(hfBody);
  });

  it('overlapShape 与 castShape', () => {
    const xf = new Transform(new Vec3(0, 1.2, 0));
    expect(world.overlapShape(new SphereShape(0.5), xf)).toHaveLength(1);
    xf.position.set(0, 3, 0);
    expect(world.overlapShape(new SphereShape(0.5), xf)).toHaveLength(0);
    const hit = world.castShape(new SphereShape(0.5), xf, new Vec3(0, -5, 0))!;
    expect(hit).not.toBeNull();
    // 球底接触平面 y = 1 时球心在 1.5 附近（加上 linearSlop 目标间距）
    expect(3 - hit.fraction * 5).toBeGreaterThan(1.49);
    expect(3 - hit.fraction * 5).toBeLessThan(1.52);
    expect(hit.normal.y).toBeLessThan(-0.99);
  });

  it('网格不能挂在动态刚体上', () => {
    const w = new World();
    const b = w.createBody();
    expect(() => b.addCollider({ shape: mesh })).toThrow();
    const ok = w.createBody({ type: 'static' });
    ok.addCollider({ shape: new PlaneShape() });
  });
});
