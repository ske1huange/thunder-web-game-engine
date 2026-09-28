import {
  BoxShape,
  CylinderShape,
  HeightfieldShape,
  Quat,
  RaycastVehicle,
  Vec3,
} from '@thunder/physics';
import * as THREE from 'three';
import type { Demo } from './types';
import { addBox, v } from './helpers';
import { createRagdoll } from './ragdoll';

const KEYS = ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' '];
const N = 97;
const CELL = 2;

function groundHeight(x: number, z: number): number {
  // 出发点附近平坦，远处起伏
  const flat = Math.min(1, Math.hypot(x, z + 20) / 40);
  return (
    flat * (2.2 * Math.sin(x * 0.045) * Math.cos(z * 0.05) + 0.8 * Math.sin(x * 0.11 + z * 0.07))
  );
}

export const vehicle: Demo = {
  id: 'vehicle',
  name: '射线车辆',
  description:
    'W / S 油门与倒车，A / D 转向，空格手刹。车轮是向下的射线：悬挂弹簧阻尼、轮胎侧向摩擦与打滑、后轮驱动。去撞箱子堆、飞跃跳台、撞倒布娃娃。',
  camera: { position: [0, 7, 14], target: [0, 1, 0] },
  ground: false,
  lighting: 'day',
  keys: KEYS,
  setup({ world, view, setInfo }) {
    // ---- 地形 ----
    const origin = (-(N - 1) * CELL) / 2;
    const heights = new Float64Array(N * N);
    for (let r = 0; r < N; r++) {
      for (let c = 0; c < N; c++)
        heights[r * N + c] = groundHeight(origin + c * CELL, origin + r * CELL);
    }
    const field = new HeightfieldShape({ heights, rows: N, cols: N, cellSize: CELL });
    const ground = world.createBody({ type: 'static' });
    ground.addCollider({ shape: field, friction: 0.9 });
    view.setAppearance(ground, { color: 0x7fa35a, roughness: 0.95 });
    const h = (x: number, z: number): number => field.heightAt(x, z);

    // ---- 跳台 ----
    const rampAngle = 0.28;
    const ramp = world.createBody({
      type: 'static',
      position: v(0, h(0, -30) + 0.9, -30),
      rotation: new Quat().setFromAxisAngle(v(1, 0, 0), rampAngle),
    });
    ramp.addCollider({ shape: new BoxShape(v(2.5, 0.2, 4)), friction: 0.8 });
    view.setAppearance(ramp, { color: 0xd8c9a8 });

    // ---- 箱子堆与路锥 ----
    for (let row = 0; row < 4; row++) {
      for (let i = 0; i < 4 - row; i++) {
        addBox(
          world,
          v(-10 + (i - (3 - row) / 2) * 1.02, h(-10, -15) + 0.5 + row, -15),
          v(0.5, 0.5, 0.5),
          {
            density: 40,
          },
        );
      }
    }
    for (let i = 0; i < 8; i++) {
      const x = 8;
      const z = -8 - i * 3;
      const cone = world.createBody({ position: v(x, h(x, z) + 0.4, z) });
      cone.addCollider({ shape: new CylinderShape(0.25, 0.4, 12), density: 30 });
      view.setAppearance(cone, { color: 0xff7a1a });
    }
    for (let i = 0; i < 3; i++) {
      const x = -4 + i * 2;
      createRagdoll(world, v(x, h(x, -45) + 1.3, -45));
    }

    // ---- 车 ----
    const start = v(0, h(0, 0) + 1.5, 0);
    const chassis = world.createBody({ position: start, angularDamping: 0.2 });
    chassis.addCollider({ shape: new BoxShape(v(0.9, 0.3, 2)), density: 150, friction: 0.5 });
    // 驾驶舱与低处配重（降低质心，不易侧翻）
    chassis.addCollider({
      shape: new BoxShape(v(0.75, 0.25, 0.9)),
      position: v(0, 0.55, 0.2),
      density: 60,
    });
    chassis.addCollider({
      shape: new BoxShape(v(0.8, 0.1, 1.6)),
      position: v(0, -0.3, 0),
      density: 500,
    });
    view.setAppearance(chassis, { color: 0xd63a3a, roughness: 0.35, metalness: 0.4 });
    const car = new RaycastVehicle(world, { chassis });
    const wheelSpots = [
      [-0.9, -1.3],
      [0.9, -1.3],
      [-0.9, 1.35],
      [0.9, 1.35],
    ] as const;
    for (const [x, z] of wheelSpots) {
      car.addWheel({
        position: v(x, -0.15, z),
        radius: 0.42,
        suspensionRestLength: 0.35,
        maxSuspensionTravel: 0.25,
        suspensionStiffness: 45,
        dampingCompression: 4.5,
        dampingRelaxation: 3,
        frictionSlip: z < 0 ? 2.6 : 2.4,
        rollInfluence: 0.02,
      });
    }

    // 车轮网格（加到 PhysicsView 的根节点，切换 demo 时一起移除）
    const tireGeo = new THREE.CylinderGeometry(0.42, 0.42, 0.3, 24).rotateZ(Math.PI / 2);
    const tireMat = new THREE.MeshStandardMaterial({ color: 0x222226, roughness: 0.9 });
    const hubGeo = new THREE.BoxGeometry(0.32, 0.5, 0.12);
    const hubMat = new THREE.MeshStandardMaterial({
      color: 0xc8c8cc,
      metalness: 0.7,
      roughness: 0.3,
    });
    const wheelMeshes = car.wheels.map(() => {
      const g = new THREE.Group();
      const tire = new THREE.Mesh(tireGeo, tireMat);
      tire.castShadow = true;
      const hub = new THREE.Mesh(hubGeo, hubMat);
      g.add(tire, hub);
      view.root.add(g);
      return g;
    });

    // ---- 输入 ----
    const pressed = new Set<string>();
    const onDown = (e: KeyboardEvent): void => {
      const k = e.key.toLowerCase();
      if (!KEYS.includes(k)) return;
      pressed.add(k);
      e.preventDefault();
    };
    const onUp = (e: KeyboardEvent): void => {
      pressed.delete(e.key.toLowerCase());
    };
    window.addEventListener('keydown', onDown);
    window.addEventListener('keyup', onUp);

    let steering = 0;
    const p = new Vec3();
    const q = new Quat();
    const follow = new Vec3();
    return {
      update(dt) {
        const up = pressed.has('w') || pressed.has('arrowup');
        const down = pressed.has('s') || pressed.has('arrowdown');
        const left = pressed.has('a') || pressed.has('arrowleft');
        const right = pressed.has('d') || pressed.has('arrowright');
        const speed = car.forwardSpeed;

        // 转向：随车速减小最大转角，平滑趋近
        const maxSteer = 0.55 / (1 + Math.abs(speed) / 15);
        const target = (left ? maxSteer : 0) - (right ? maxSteer : 0);
        steering += (target - steering) * Math.min(1, 8 * dt);
        car.setSteering(steering, 0);
        car.setSteering(steering, 1);

        // 油门 / 倒车 / 刹车（后轮驱动）
        let engine = 0;
        let brake = 0;
        if (up) engine = speed < 30 ? 4200 : 0;
        if (down) {
          if (speed > 1) brake = 6000;
          else engine = -2500;
        }
        if (!up && !down) brake = 300; // 松油门时的滚动阻力
        for (let i = 0; i < 4; i++) {
          car.applyEngineForce(i >= 2 ? engine : 0, i);
          car.setBrake(brake, i);
        }
        if (pressed.has(' ')) {
          car.setBrake(9000, 2);
          car.setBrake(9000, 3);
        }

        // 翻车或掉出地图时复位
        const upY = chassis.getWorldVector(v(0, 1, 0), new Vec3()).y;
        if (chassis.position.y < -30 || (upY < 0.1 && chassis.linearVelocity.length() < 0.5)) {
          chassis.setTransform(
            v(
              chassis.position.x,
              h(chassis.position.x, chassis.position.z) + 2,
              chassis.position.z,
            ),
            new Quat(),
          );
          chassis.setLinearVelocity(v(0, 0, 0));
          chassis.setAngularVelocity(v(0, 0, 0));
        }

        const skid = car.wheels.some((w) => w.isInContact && w.skidInfo < 0.9);
        setInfo(
          `${(speed * 3.6).toFixed(0)} km/h，着地车轮 ${car.wheelsInContact} / 4${skid ? '，打滑' : ''}`,
        );
      },
      render(alpha) {
        for (let i = 0; i < wheelMeshes.length; i++) {
          car.getWheelTransform(i, p, q, alpha);
          wheelMeshes[i]!.position.set(p.x, p.y, p.z);
          wheelMeshes[i]!.quaternion.set(q.x, q.y, q.z, q.w);
        }
      },
      follow(alpha) {
        chassis.interpolate(alpha, follow);
        return follow;
      },
      dispose() {
        window.removeEventListener('keydown', onDown);
        window.removeEventListener('keyup', onUp);
        tireGeo.dispose();
        hubGeo.dispose();
        tireMat.dispose();
        hubMat.dispose();
      },
    };
  },
};
