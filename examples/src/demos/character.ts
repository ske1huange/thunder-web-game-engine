import {
  BoxShape,
  CharacterController,
  CylinderShape,
  HingeJoint,
  Quat,
  SphereShape,
  Vec3,
} from '@thunder/physics';
import * as THREE from 'three';
import type { Demo } from './types';
import { addBox, v } from './helpers';

const KEYS = ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' ', 'shift'];

export const character: Demo = {
  id: 'character',
  name: '角色控制器',
  description:
    'WASD / 方向键移动，Shift 跑，空格跳。角色会自动迈上台阶、爬缓坡（陡坡上不去）、下坡贴地、站在移动 / 旋转平台上随之移动、推开箱子，压在跷跷板上会让它倾斜。',
  camera: { position: [0, 6, 10], target: [0, 1, 0] },
  lighting: 'day',
  keys: KEYS,
  setup({ world, ground, view, camera, setInfo }) {
    const wall = { body: { type: 'static' as const }, friction: 0.6 };
    const stone = 0xb7ad9b;

    // ---- 楼梯：10 级，每级高 0.18 m ----
    for (let i = 0; i < 10; i++) {
      const h = 0.18 * (i + 1);
      const b = addBox(world, v(-6, h / 2, -2 - i * 0.4), v(1.2, h / 2, 0.2), wall);
      view.setAppearance(b, { color: stone });
    }
    const landing = addBox(world, v(-6, 0.9, -7), v(1.2, 0.9, 1), wall);
    view.setAppearance(landing, { color: stone });

    // ---- 缓坡（20°）与陡坡（55°） ----
    for (const [x, deg, color] of [
      [-1.5, 20, 0x9cc28a],
      [2.5, 55, 0xd98f7a],
    ] as const) {
      const a = (deg * Math.PI) / 180;
      const len = 5;
      const ramp = world.createBody({
        type: 'static',
        position: v(x, (Math.sin(a) * len) / 2 - 0.1, -2 - (Math.cos(a) * len) / 2),
        rotation: new Quat().setFromAxisAngle(v(1, 0, 0), a),
      });
      ramp.addCollider({ shape: new BoxShape(v(1.2, 0.1, len / 2)), friction: 0.6 });
      view.setAppearance(ramp, { color });
    }

    // ---- 往返移动的平台与旋转平台（运动学刚体） ----
    const lift = world.createBody({ type: 'kinematic', position: v(7, 0.6, 2) });
    lift.addCollider({ shape: new BoxShape(v(1.2, 0.15, 1.2)) });
    view.setAppearance(lift, { color: 0x6aa6e8 });
    const disc = world.createBody({ type: 'kinematic', position: v(7, 0.3, -5) });
    disc.addCollider({ shape: new CylinderShape(2, 0.15, 24) });
    disc.setAngularVelocity(v(0, 0.6, 0));
    view.setAppearance(disc, { color: 0xe8c16a });

    // ---- 可推动的箱子与球 ----
    for (let i = 0; i < 6; i++) {
      addBox(world, v(-2 + (i % 3) * 1.1, 0.35 + Math.floor(i / 3) * 0.7, 4), v(0.35, 0.35, 0.35), {
        density: 8,
        friction: 0.5,
      });
    }
    for (let i = 0; i < 3; i++) {
      const ball = world.createBody({ position: v(1.5 + i * 0.9, 0.4, 5) });
      ball.addCollider({ shape: new SphereShape(0.4), density: 5, friction: 0.5 });
    }

    // ---- 跷跷板 ----
    const pivot = addBox(world, v(-3, 0.3, 8), v(0.2, 0.3, 0.8), wall);
    view.setAppearance(pivot, { color: stone });
    const plank = world.createBody({ position: v(-3, 0.68, 8) });
    plank.addCollider({ shape: new BoxShape(v(2.6, 0.08, 0.8)), density: 20, friction: 0.8 });
    world.addJoint(
      new HingeJoint({
        bodyA: ground,
        bodyB: plank,
        anchor: v(-3, 0.62, 8),
        axis: v(0, 0, 1),
        enableLimit: true,
        lowerAngle: -0.3,
        upperAngle: 0.3,
      }),
    );
    view.setAppearance(plank, { color: 0xc98b52 });

    // ---- 角色 ----
    const player = new CharacterController(world, {
      position: v(0, 1.5, 2),
      stepHeight: 0.3,
      maxSlopeAngle: (45 * Math.PI) / 180,
      mass: 80,
    });
    view.setAppearance(player.body!, { color: 0xff7a45, roughness: 0.4 });
    // 面朝方向的小“鼻子”，便于看出朝向
    const nose = world.createBody({ type: 'kinematic', position: v(0, 1.5, 2.3) });
    // 不参与任何碰撞与查询
    nose.addCollider({ shape: new SphereShape(0.08), filter: { categoryBits: 0, maskBits: 0 } });
    view.setAppearance(nose, { color: 0x222222, opacity: 1 });

    // ---- 输入 ----
    const pressed = new Set<string>();
    let jumpQueued = false;
    const onDown = (e: KeyboardEvent): void => {
      const k = e.key.toLowerCase();
      if (!KEYS.includes(k)) return;
      if (k === ' ' && !pressed.has(k)) jumpQueued = true;
      pressed.add(k);
      e.preventDefault();
    };
    const onUp = (e: KeyboardEvent): void => {
      pressed.delete(e.key.toLowerCase());
    };
    window.addEventListener('keydown', onDown);
    window.addEventListener('keyup', onUp);

    const velocity = new Vec3();
    const wish = new Vec3();
    const forward = new THREE.Vector3();
    const step = new Vec3();
    const facing = new Vec3(0, 0, -1);
    const followTarget = new Vec3();
    const tmpQ = new Quat();
    let liftTime = 0;

    return {
      update(dt) {
        // 平台：升降机沿 X 往返并上下移动
        liftTime += dt;
        const target = v(
          7 + Math.sin(liftTime * 0.5) * 3,
          0.6 + (1 - Math.cos(liftTime * 0.5)) * 0.8,
          2,
        );
        lift.moveKinematic(target, lift.rotation, dt);

        // 按相机朝向计算期望方向
        camera.getWorldDirection(forward);
        forward.y = 0;
        forward.normalize();
        const fx = forward.x;
        const fz = forward.z;
        let ix = 0;
        let iz = 0;
        if (pressed.has('w') || pressed.has('arrowup')) iz += 1;
        if (pressed.has('s') || pressed.has('arrowdown')) iz -= 1;
        if (pressed.has('d') || pressed.has('arrowright')) ix += 1;
        if (pressed.has('a') || pressed.has('arrowleft')) ix -= 1;
        // 右方向 = forward × up = (-fz, 0, fx)
        wish.set(fx * iz - fz * ix, 0, fz * iz + fx * ix);
        const len = wish.length();
        if (len > 0) wish.scale(1 / len);
        const speed = pressed.has('shift') ? 7 : 4;

        // 水平速度平滑趋近目标（空中控制力较弱）
        const accel = player.isGrounded ? 12 : 3;
        const k = Math.min(1, accel * dt);
        velocity.x += (wish.x * speed - velocity.x) * k;
        velocity.z += (wish.z * speed - velocity.z) * k;
        if (player.isGrounded) {
          velocity.y = jumpQueued ? 6 : 0;
        } else {
          velocity.y -= 9.81 * dt;
          if (player.hitCeiling && velocity.y > 0) velocity.y = 0;
        }
        jumpQueued = false;
        player.move(step.copy(velocity).scale(dt), dt);

        // 朝向与“鼻子”
        if (len > 0) facing.set(wish.x, 0, wish.z);
        tmpQ.setFromUnitVectors(v(0, 0, -1), facing);
        const nosePos = v(player.position.x, player.position.y + 0.5, player.position.z).addScaled(
          facing,
          0.3,
        );
        nose.moveKinematic(nosePos, tmpQ, dt);

        const g = player.groundBody;
        const groundText = !player.isGrounded
          ? '空中'
          : g === null
            ? '地面'
            : g.isKinematic()
              ? '移动平台'
              : g.isDynamic()
                ? '动态物体'
                : '静态地面';
        setInfo(
          `${groundText}，水平速度 ${Math.hypot(player.velocity.x, player.velocity.z).toFixed(1)} m/s，高度 ${(player.position.y - 0.9).toFixed(2)} m`,
        );
      },
      follow(alpha) {
        player.body!.interpolate(alpha, followTarget);
        return followTarget;
      },
      dispose() {
        window.removeEventListener('keydown', onDown);
        window.removeEventListener('keyup', onUp);
      },
    };
  },
};
