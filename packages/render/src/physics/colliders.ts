import { ConvexHullShape, TriMeshShape, type TriMeshOptions, Vec3 } from '@thunder/physics';
import * as THREE from 'three';

/** 从 three.js 场景收集三角形时的选项 */
export interface CollectOptions {
  /**
   * 顶点坐标所在的参考系：默认是传入对象自身的局部坐标系（碰撞体挂在与该对象同位姿的刚体上）；
   * 传 null 表示世界坐标（刚体放在原点）。
   */
  relativeTo?: THREE.Object3D | null;
  /** 过滤网格，返回 false 跳过（例如只收集名称以 "_collider" 结尾的网格） */
  filter?: (mesh: THREE.Mesh) => boolean;
  /** 是否跳过不可见的网格，默认 false（常见做法是用隐藏的简化网格作碰撞体） */
  skipInvisible?: boolean;
}

export interface CollectedTriangles {
  /** 顶点坐标 [x, y, z, ...] */
  positions: Float32Array;
  /** 三角形下标（逆时针为正面） */
  indices: Uint32Array;
}

const tmpMatrix = new THREE.Matrix4();
const instanceMatrix = new THREE.Matrix4();
const baseInverse = new THREE.Matrix4();
const v = new THREE.Vector3();

/**
 * 收集对象及其子孙中所有网格（含 InstancedMesh 的每个实例）的三角形，
 * 变换（含缩放、镜像）已烘焙进顶点坐标。
 */
export function collectTriangles(
  object: THREE.Object3D,
  options: CollectOptions = {},
): CollectedTriangles {
  object.updateWorldMatrix(true, true);
  const relativeTo = options.relativeTo === undefined ? object : options.relativeTo;
  if (relativeTo) baseInverse.copy(relativeTo.matrixWorld).invert();
  else baseInverse.identity();

  const positions: number[] = [];
  const indices: number[] = [];
  object.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (!mesh.isMesh) return;
    if (options.skipInvisible && !isVisible(mesh)) return;
    if (options.filter && !options.filter(mesh)) return;
    const geometry = mesh.geometry as THREE.BufferGeometry | undefined;
    const position = geometry?.getAttribute('position');
    if (!geometry || !position) return;
    const instanced = (mesh as THREE.InstancedMesh).isInstancedMesh
      ? (mesh as THREE.InstancedMesh)
      : null;
    const instances = instanced ? instanced.count : 1;
    for (let k = 0; k < instances; k++) {
      tmpMatrix.multiplyMatrices(baseInverse, mesh.matrixWorld);
      if (instanced) {
        instanced.getMatrixAt(k, instanceMatrix);
        tmpMatrix.multiply(instanceMatrix);
      }
      appendGeometry(geometry, tmpMatrix, positions, indices);
    }
  });
  return { positions: Float32Array.from(positions), indices: Uint32Array.from(indices) };
}

function isVisible(o: THREE.Object3D): boolean {
  for (let p: THREE.Object3D | null = o; p; p = p.parent) if (!p.visible) return false;
  return true;
}

function appendGeometry(
  geometry: THREE.BufferGeometry,
  matrix: THREE.Matrix4,
  positions: number[],
  indices: number[],
): void {
  const position = geometry.getAttribute('position');
  const base = positions.length / 3;
  for (let i = 0; i < position.count; i++) {
    v.fromBufferAttribute(position, i).applyMatrix4(matrix);
    positions.push(v.x, v.y, v.z);
  }
  // 镜像变换（行列式为负）会翻转三角形朝向
  const flip = matrix.determinant() < 0;
  const index = geometry.getIndex();
  const start = Math.max(0, geometry.drawRange.start);
  const total = index ? index.count : position.count;
  const end = Math.min(total, start + geometry.drawRange.count);
  for (let i = start; i + 2 < end; i += 3) {
    const a = index ? index.getX(i) : i;
    const b = index ? index.getX(i + 1) : i + 1;
    const c = index ? index.getX(i + 2) : i + 2;
    if (flip) indices.push(base + a, base + c, base + b);
    else indices.push(base + a, base + b, base + c);
  }
}

/**
 * 由 three.js 对象（例如 GLTFLoader 加载的 gltf.scene）生成静态三角网格碰撞形状。
 *
 * ```ts
 * const gltf = await new GLTFLoader().loadAsync('level.glb');
 * const level = world.createBody({ type: 'static' });
 * level.addCollider({ shape: createTriMeshFromObject(gltf.scene) });
 * ```
 */
export function createTriMeshFromObject(
  object: THREE.Object3D,
  options: CollectOptions & TriMeshOptions = {},
): TriMeshShape {
  const { positions, indices } = collectTriangles(object, options);
  if (indices.length === 0) throw new Error('createTriMeshFromObject: 对象中没有三角形');
  return new TriMeshShape(positions, indices, options);
}

/** 由单个 BufferGeometry（可选变换矩阵）生成三角网格碰撞形状 */
export function createTriMeshFromGeometry(
  geometry: THREE.BufferGeometry,
  options: TriMeshOptions & { matrix?: THREE.Matrix4 } = {},
): TriMeshShape {
  const positions: number[] = [];
  const indices: number[] = [];
  appendGeometry(geometry, options.matrix ?? new THREE.Matrix4(), positions, indices);
  return new TriMeshShape(positions, indices, options);
}

/**
 * 由 three.js 对象的顶点生成凸包碰撞形状（可用于动态刚体，例如道具、石块）。
 *
 * 顶点数超过 maxPoints（默认 64）时，取球面上均匀分布的 maxPoints 个方向上的最远顶点，
 * 得到保留外形轮廓、面数可控的近似凸包（顶点越少，碰撞越快）。
 */
export function createConvexHullFromObject(
  object: THREE.Object3D,
  options: CollectOptions & { maxPoints?: number } = {},
): ConvexHullShape {
  const { positions } = collectTriangles(object, options);
  const count = positions.length / 3;
  if (count < 4) throw new Error('createConvexHullFromObject: 顶点不足');
  const maxPoints = Math.max(4, options.maxPoints ?? 64);
  const chosen = new Set<number>();
  if (count <= maxPoints) {
    for (let i = 0; i < count; i++) chosen.add(i);
  } else {
    // 斐波那契球面方向上的支撑点
    const golden = Math.PI * (3 - Math.sqrt(5));
    for (let k = 0; k < maxPoints; k++) {
      const y = 1 - (2 * (k + 0.5)) / maxPoints;
      const r = Math.sqrt(1 - y * y);
      const dx = Math.cos(golden * k) * r;
      const dz = Math.sin(golden * k) * r;
      let best = 0;
      let bestDot = -Infinity;
      for (let i = 0; i < count; i++) {
        const d = positions[i * 3]! * dx + positions[i * 3 + 1]! * y + positions[i * 3 + 2]! * dz;
        if (d > bestDot) {
          bestDot = d;
          best = i;
        }
      }
      chosen.add(best);
    }
  }
  const points: Vec3[] = [];
  for (const i of chosen) {
    points.push(new Vec3(positions[i * 3]!, positions[i * 3 + 1]!, positions[i * 3 + 2]!));
  }
  return new ConvexHullShape(points);
}
