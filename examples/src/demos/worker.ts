import { type BodyDesc, type BodyProxy, Quat, Vec3, WorkerWorld } from '@thunder/physics';
import * as THREE from 'three';
import type { Demo } from './types';

const COUNT = 400;
const HALF = 5;

export const worker: Demo = {
  id: 'worker',
  name: 'Web Worker',
  description: `物理世界运行在 Web Worker 中：${COUNT} 个物体的模拟不占用主线程，主线程只做渲染（InstancedMesh + 插值）。位姿快照是一块来回转移的 Float64Array，不复制。空格把所有物体炸飞。`,
  camera: { position: [14, 12, 16], target: [0, 2, 0] },
  lighting: 'day',
  keys: [' '],
  setup({ view, setInfo }) {
    const physics = new WorkerWorld(
      new Worker(new URL('../physics.worker.ts', import.meta.url), { type: 'module' }),
    );

    // ---- Worker 中的场景 ----
    physics.createBody({ type: 'static', colliders: [{ shape: { type: 'plane' } }] });
    const wallMat = new THREE.MeshStandardMaterial({ color: 0x8a94a6, roughness: 0.8 });
    for (const [x, z, hx, hz] of [
      [HALF, 0, 0.3, HALF],
      [-HALF, 0, 0.3, HALF],
      [0, HALF, HALF, 0.3],
      [0, -HALF, HALF, 0.3],
    ] as const) {
      const wall: BodyDesc = {
        type: 'static',
        position: [x, 1.5, z],
        colliders: [{ shape: { type: 'box', halfExtents: [hx, 1.5, hz] } }],
      };
      physics.createBody(wall);
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(hx * 2, 3, hz * 2), wallMat);
      mesh.position.set(x, 1.5, z);
      mesh.castShadow = mesh.receiveShadow = true;
      view.root.add(mesh);
    }

    const boxes: BodyProxy[] = [];
    const spheres: BodyProxy[] = [];
    for (let i = 0; i < COUNT; i++) {
      const layer = Math.floor(i / 64);
      const x = ((i % 8) - 3.5) * 1.05;
      const z = ((Math.floor(i / 8) % 8) - 3.5) * 1.05;
      const isBox = i % 2 === 0;
      const body = physics.createBody({
        position: [x + (layer % 2) * 0.3, 2 + layer * 1.2, z],
        // 绕 Y 轴旋转 i 弧度
        rotation: [0, Math.sin(i / 2), 0, Math.cos(i / 2)],
        colliders: [
          {
            shape: isBox
              ? { type: 'box', halfExtents: [0.4, 0.4, 0.4] }
              : { type: 'sphere', radius: 0.42 },
            friction: 0.5,
          },
        ],
      });
      (isBox ? boxes : spheres).push(body);
    }

    // ---- 渲染：每种形状一个 InstancedMesh ----
    const makeInstanced = (geo: THREE.BufferGeometry, color: number, n: number) => {
      const mesh = new THREE.InstancedMesh(
        geo,
        new THREE.MeshStandardMaterial({ color, roughness: 0.5 }),
        n,
      );
      mesh.castShadow = mesh.receiveShadow = true;
      mesh.frustumCulled = false;
      const c = new THREE.Color();
      for (let i = 0; i < n; i++) mesh.setColorAt(i, c.setHSL((i * 0.137) % 1, 0.55, 0.58));
      view.root.add(mesh);
      return mesh;
    };
    const boxMesh = makeInstanced(new THREE.BoxGeometry(0.8, 0.8, 0.8), 0xffffff, boxes.length);
    const sphereMesh = makeInstanced(
      new THREE.SphereGeometry(0.42, 20, 14),
      0xffffff,
      spheres.length,
    );

    const onKey = (e: KeyboardEvent): void => {
      if (e.key !== ' ') return;
      e.preventDefault();
      for (const b of [...boxes, ...spheres]) {
        const p = b.position;
        const d = Math.max(1, Math.hypot(p.x, p.z));
        b.applyImpulse(new Vec3((p.x / d) * 3, 8 + Math.random() * 4, (p.z / d) * 3));
      }
    };
    window.addEventListener('keydown', onKey);

    const pos = new Vec3();
    const rot = new Quat();
    const m = new THREE.Matrix4();
    const tp = new THREE.Vector3();
    const tq = new THREE.Quaternion();
    const one = new THREE.Vector3(1, 1, 1);
    let last = performance.now();
    let frameMs = 0;
    let infoTimer = 0;
    const write = (mesh: THREE.InstancedMesh, list: BodyProxy[], alpha: number) => {
      for (let i = 0; i < list.length; i++) {
        list[i]!.interpolate(alpha, pos, rot);
        m.compose(tp.set(pos.x, pos.y, pos.z), tq.set(rot.x, rot.y, rot.z, rot.w), one);
        mesh.setMatrixAt(i, m);
      }
      mesh.instanceMatrix.needsUpdate = true;
    };

    return {
      render() {
        const t0 = performance.now();
        const dt = Math.min((t0 - last) / 1000, 0.1);
        last = t0;
        const alpha = physics.advance(dt);
        write(boxMesh, boxes, alpha);
        write(sphereMesh, spheres, alpha);
        frameMs = frameMs * 0.9 + (performance.now() - t0) * 0.1;
        infoTimer += dt;
        if (infoTimer > 0.25) {
          infoTimer = 0;
          const s = physics.stats;
          setInfo(
            `Worker：${s.bodies} 个刚体（唤醒 ${s.awakeBodies}），每次推进 ${s.stepTime.toFixed(2)} ms；主线程同步 ${frameMs.toFixed(2)} ms/帧`,
          );
        }
      },
      dispose() {
        window.removeEventListener('keydown', onKey);
        physics.terminate();
      },
    };
  },
};
