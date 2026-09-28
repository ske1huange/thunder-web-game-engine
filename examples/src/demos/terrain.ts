import {
  BoxShape,
  CapsuleShape,
  ConvexHullShape,
  HeightfieldShape,
  type RigidBody,
  type Shape,
  SphereShape,
  Vec3,
} from '@thunder/physics';
import { createTriMeshFromObject } from '@thunder/render';
import * as THREE from 'three';
import type { Demo } from './types';
import { q, rng, v } from './helpers';

const SIZE = 96;
const CELL = 1;
const MAX_BODIES = 160;

/** 地形高度：西北角的山丘、中部盆地与起伏 */
function terrainHeight(x: number, z: number): number {
  const hill = 11 * Math.exp(-((x + 28) ** 2 + (z + 28) ** 2) / 500);
  const basin = -2.5 * Math.exp(-(x * x + z * z) / 300);
  const waves = 0.9 * Math.sin(x * 0.18) * Math.cos(z * 0.15) + 0.35 * Math.sin(x * 0.5 + z * 0.35);
  const rim = 4 * Math.max(0, Math.hypot(x, z) / 44 - 0.85) ** 2 * 10;
  return hill + basin + waves + rim;
}

/** 用 three.js 几何体搭建关卡物件，交给 createTriMeshFromObject 生成三角网格碰撞体 */
function buildLevel(): { group: THREE.Group; halfPipe: THREE.Mesh } {
  const group = new THREE.Group();
  // 台阶
  const stairs = new THREE.Group();
  for (let i = 0; i < 6; i++) {
    const step = new THREE.Mesh(new THREE.BoxGeometry(4, 0.3 * (i + 1), 0.8));
    step.position.set(0, 0.15 * (i + 1), i * 0.8);
    stairs.add(step);
  }
  stairs.position.set(-4, 0, 14);
  // 跳台（楔形）
  const ramp = new THREE.Mesh(wedgeGeometry(4, 1.6, 6));
  ramp.position.set(-10, 0, -2);
  ramp.rotation.y = Math.PI * 0.25;
  // 雕塑：环面纽结（模拟从 glTF 导入的任意模型）
  const knot = new THREE.Mesh(new THREE.TorusKnotGeometry(2.2, 0.6, 120, 12));
  knot.position.set(14, 4, -12);
  group.add(stairs, ramp, knot);
  // U 形槽（双面网格）：下半个开口圆柱，轴沿 X
  const halfPipe = new THREE.Mesh(
    new THREE.CylinderGeometry(4, 4, 12, 32, 1, true, Math.PI, Math.PI),
  );
  halfPipe.rotation.z = Math.PI / 2;
  halfPipe.position.set(12, 0, 10);
  return { group, halfPipe };
}

/** 楔形：宽 w、高 h、长 l，斜面朝 -Z */
function wedgeGeometry(w: number, h: number, l: number): THREE.BufferGeometry {
  const x = w / 2;
  const p = [
    [-x, 0, -l / 2],
    [x, 0, -l / 2],
    [x, 0, l / 2],
    [-x, 0, l / 2],
    [-x, h, l / 2],
    [x, h, l / 2],
  ];
  // 逆时针（从外侧看）
  const tris = [
    [0, 2, 1],
    [0, 3, 2], // 底
    [0, 1, 5],
    [0, 5, 4], // 斜面
    [3, 4, 5],
    [3, 5, 2], // 背面
    [0, 4, 3], // 左侧
    [1, 2, 5], // 右侧
  ];
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.Float32BufferAttribute(p.flat(), 3));
  g.setIndex(tris.flat());
  return g;
}

export const terrain: Demo = {
  id: 'terrain',
  name: '地形与关卡',
  description:
    '96×96 高度场地形 + three.js 几何体生成的三角网格关卡（台阶、跳台、U 形槽、环面纽结）。物体从山顶滚下，经过三角形接缝不会被绊住；空格发射的高速小球由连续碰撞挡在薄网格表面。',
  camera: { position: [34, 26, 36], target: [0, 1, 0] },
  ground: false,
  lighting: 'day',
  setup({ world, view, setInfo }) {
    // ---- 高度场地形 ----
    const n = SIZE + 1;
    const origin = -SIZE * CELL * 0.5;
    const heights = new Float64Array(n * n);
    for (let r = 0; r < n; r++) {
      for (let c = 0; c < n; c++) {
        heights[r * n + c] = terrainHeight(origin + c * CELL, origin + r * CELL);
      }
    }
    const terrainShape = new HeightfieldShape({ heights, rows: n, cols: n, cellSize: CELL });
    const ground = world.createBody({ type: 'static' });
    ground.addCollider({ shape: terrainShape, friction: 0.8 });
    view.setAppearance(ground, { color: 0x6f9a52, roughness: 0.95 });

    // ---- 三角网格关卡：物件按地形高度摆放 ----
    const { group, halfPipe } = buildLevel();
    for (const child of group.children) {
      child.position.y += terrainShape.heightAt(child.position.x, child.position.z);
    }
    const level = world.createBody({ type: 'static' });
    const levelShape = createTriMeshFromObject(group);
    level.addCollider({ shape: levelShape, friction: 0.6 });
    view.setAppearance(level, { color: 0xc9b99a, roughness: 0.7 });
    // 槽底略高于地面（半径 4）
    halfPipe.position.y = terrainShape.heightAt(halfPipe.position.x, halfPipe.position.z) + 4.3;
    const pipe = world.createBody({ type: 'static' });
    const pipeShape = createTriMeshFromObject(halfPipe, { relativeTo: null, doubleSided: true });
    pipe.addCollider({ shape: pipeShape, friction: 0.3 });
    view.setAppearance(pipe, { color: 0x8fb3d9, roughness: 0.4, metalness: 0.3 });

    const triangles =
      terrainShape.triangleCount + levelShape.triangleCount + pipeShape.triangleCount;

    // ---- 不断从山顶投放物体 ----
    const rand = rng(99);
    const rock = (): Shape => {
      const pts: Vec3[] = [];
      for (let i = 0; i < 12; i++) {
        const p = v(rand() * 2 - 1, rand() * 2 - 1, rand() * 2 - 1);
        p.normalize();
        pts.push(p.scale(0.4 + rand() * 0.25));
      }
      return new ConvexHullShape(pts);
    };
    const makers: (() => Shape)[] = [
      () => new SphereShape(0.35 + rand() * 0.25),
      () => new BoxShape(v(0.3 + rand() * 0.2, 0.3 + rand() * 0.2, 0.3 + rand() * 0.2)),
      () => new CapsuleShape(0.25, 0.3 + rand() * 0.2),
      rock,
    ];
    const spawned: RigidBody[] = [];
    let timer = 0;
    let total = 0;
    return (dt) => {
      // 掉出地形的物体回收
      for (let i = spawned.length - 1; i >= 0; i--) {
        const b = spawned[i]!;
        if (b.position.y < -30) {
          world.destroyBody(b);
          spawned.splice(i, 1);
        }
      }
      timer += dt;
      if (timer >= 0.25 && spawned.length < MAX_BODIES) {
        timer = 0;
        const x = -26 + (rand() - 0.5) * 8;
        const z = -26 + (rand() - 0.5) * 8;
        const body = world.createBody({
          position: v(x, terrainShape.heightAt(x, z) + 2, z),
          rotation: q(rand() * 6, rand() * 6, rand() * 6),
          linearVelocity: v(3 + rand() * 2, 0, 3 + rand() * 2),
        });
        body.addCollider({ shape: makers[total % makers.length]!(), friction: 0.6 });
        spawned.push(body);
        total++;
      }
      setInfo(`三角形 ${triangles}，物体 ${spawned.length} / ${MAX_BODIES}`);
    };
  },
};
