import { HeightfieldShape, ShapeType, Vec3, World } from '@thunder/physics';
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import {
  PhysicsView,
  collectTriangles,
  createConvexHullFromObject,
  createTriMeshFromGeometry,
  createTriMeshFromObject,
} from '../src';

/** 地面（2×2 平面，朝上）+ 一个放大平移后的盒子 */
function level(): THREE.Group {
  const root = new THREE.Group();
  root.position.set(10, 0, 0);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(2, 2).rotateX(-Math.PI / 2));
  const box = new THREE.Mesh(new THREE.BoxGeometry(1, 1, 1));
  box.position.set(0, 1, 0);
  box.scale.set(2, 1, 1);
  root.add(floor, box);
  return root;
}

describe('three.js → 碰撞体', () => {
  it('收集三角形并烘焙变换（默认相对对象自身，relativeTo: null 为世界坐标）', () => {
    const root = level();
    const local = collectTriangles(root);
    expect(local.indices.length / 3).toBe(2 + 12);
    let maxX = -Infinity;
    for (let i = 0; i < local.positions.length; i += 3) maxX = Math.max(maxX, local.positions[i]!);
    expect(maxX).toBeCloseTo(1, 6); // 盒子半宽 0.5 × 缩放 2
    const world = collectTriangles(root, { relativeTo: null });
    let minX = Infinity;
    for (let i = 0; i < world.positions.length; i += 3) minX = Math.min(minX, world.positions[i]!);
    expect(minX).toBeCloseTo(9, 6);
    const onlyFloor = collectTriangles(root, {
      filter: (m) => (m.geometry as THREE.BufferGeometry).type === 'PlaneGeometry',
    });
    expect(onlyFloor.indices.length).toBe(6);
  });

  it('镜像缩放保持三角形朝外，InstancedMesh 展开每个实例', () => {
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(2, 2).rotateX(-Math.PI / 2));
    floor.scale.set(-1, 1, 1);
    const shape = createTriMeshFromObject(floor);
    const hit = { t: 0, normal: new Vec3() };
    // 单面网格：从上方命中说明朝向正确
    expect(shape.raycast(new Vec3(0.2, 3, 0.1), new Vec3(0, -1, 0), 10, hit)).toBe(true);
    expect(hit.normal.y).toBeCloseTo(1, 9);

    const instanced = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), undefined, 3);
    for (let i = 0; i < 3; i++) {
      instanced.setMatrixAt(i, new THREE.Matrix4().makeTranslation(i * 3, 0, 0));
    }
    expect(collectTriangles(instanced).indices.length / 3).toBe(36);
  });

  it('生成的三角网格可用于射线检测与碰撞', () => {
    const world = new World();
    const body = world.createBody({ type: 'static' });
    body.addCollider({ shape: createTriMeshFromObject(level(), { relativeTo: null }) });
    const hit = world.raycast(new Vec3(10, 5, 0), new Vec3(0, -1, 0))!;
    expect(hit.point.y).toBeCloseTo(1.5, 6);
    const shape = createTriMeshFromGeometry(new THREE.TorusKnotGeometry(1, 0.3, 64, 8));
    expect(shape.triangleCount).toBe(64 * 8 * 2);
  });

  it('凸包按支撑点抽样控制顶点数', () => {
    const sphere = new THREE.Mesh(new THREE.SphereGeometry(1, 48, 32));
    const hull = createConvexHullFromObject(sphere, { maxPoints: 40 });
    expect(hull.hull.vertices.length).toBeLessThanOrEqual(40);
    // 抽样点都在球面上，凸包体积接近球体积
    expect(hull.getVolume()).toBeGreaterThan((4 / 3) * Math.PI * 0.6);
    expect(hull.getVolume()).toBeLessThan((4 / 3) * Math.PI);
  });
});

describe('PhysicsView 与网格', () => {
  it('为三角网格与高度场生成网格，visible: false 时隐藏', () => {
    const world = new World();
    const levelBody = world.createBody({ type: 'static' });
    levelBody.addCollider({ shape: createTriMeshFromObject(level()) });
    const terrain = world.createBody({ type: 'static' });
    terrain.addCollider({
      shape: new HeightfieldShape({ heights: new Float64Array(12), rows: 3, cols: 4 }),
    });
    expect(terrain.colliders[0]!.shape.type).toBe(ShapeType.Heightfield);
    const view = new PhysicsView(world);
    view.sync();
    const levelMesh = view.getObject(levelBody)!.children[0] as THREE.Mesh;
    expect(levelMesh.geometry.getAttribute('position').count).toBe(14 * 3);
    const terrainMesh = view.getObject(terrain)!.children[0] as THREE.Mesh;
    expect(terrainMesh.geometry.getAttribute('position').count).toBe(12);
    expect(terrainMesh.geometry.getIndex()!.count).toBe(2 * 3 * 2 * 3);
    view.setAppearance(levelBody, { visible: false });
    view.sync();
    expect(view.getObject(levelBody)!.visible).toBe(false);
  });
});
