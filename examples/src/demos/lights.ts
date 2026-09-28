import { BoxShape, CylinderShape, Vec3 } from '@thunder/physics';
import type { Demo } from './types';
import { addBox, addSphere, rng, v } from './helpers';

const GLOW_COLORS = [0xff5a36, 0x36c5ff, 0xffd23f, 0x7cff6b, 0xff4fd8, 0x9f7bff];

export const lights: Demo = {
  id: 'lights',
  name: '光源与调色',
  description:
    '夜晚预设 + 电影感调色。发光球上绑定了点光源；中央灯塔是旋转的运动学刚体，聚光灯随它扫过场景并投射阴影。可在左上角切换光照与调色预设。',
  camera: { position: [13, 9, 15], target: [0, 1.5, 0] },
  lighting: 'night',
  grading: 'cinematic',
  setup({ world, view, lights, setInfo }) {
    const rand = rng(7);

    // 灯塔：运动学刚体，绕 Y 轴匀速旋转，聚光灯绑定在灯头上
    const tower = world.createBody({ type: 'kinematic', position: v(0, 0, 0) });
    tower.addCollider({ shape: new CylinderShape(0.5, 1.6, 20), position: v(0, 1.6, 0) });
    tower.addCollider({ shape: new BoxShape(new Vec3(0.45, 0.3, 0.45)), position: v(0, 3.5, 0) });
    tower.setAngularVelocity(v(0, 0.6, 0));
    view.setAppearance(tower, { color: 0xd9d4c7, roughness: 0.4 });
    const beam = lights.addSpotLight({
      color: 0xfff1c9,
      intensity: 400,
      angle: 0.32,
      penumbra: 0.4,
      castShadow: true,
      shadowMapSize: 1024,
    });
    // 光源放在灯头外侧，避免被灯头自身的阴影遮挡
    lights.attachToBody(beam, tower, {
      offset: { x: 0.55, y: 3.5, z: 0 },
      direction: { x: 1, y: -0.35, z: 0 },
    });

    // 柱子与散落的箱子，用来展示阴影
    for (let i = 0; i < 8; i++) {
      const a = (i / 8) * Math.PI * 2;
      addBox(world, v(Math.cos(a) * 7, 1.5, Math.sin(a) * 7), v(0.35, 1.5, 0.35), {
        body: { type: 'static' },
      });
    }
    for (let i = 0; i < 14; i++) {
      const a = rand() * Math.PI * 2;
      const r = 2.5 + rand() * 3.5;
      addBox(world, v(Math.cos(a) * r, 0.4, Math.sin(a) * r), v(0.4, 0.4, 0.4));
    }

    // 发光球：自发光材质 + 绑定的点光源
    const balls = GLOW_COLORS.map((color, i) => {
      const a = (i / GLOW_COLORS.length) * Math.PI * 2;
      const ball = addSphere(world, v(Math.cos(a) * 4, 1 + i * 0.4, Math.sin(a) * 4), 0.3, {
        restitution: 0.6,
        friction: 0.3,
        body: { linearVelocity: v(-Math.sin(a) * 4, 0, Math.cos(a) * 4) },
      });
      // 光源在球体内部：球体不投射阴影，否则会挡住自己的光
      view.setAppearance(ball, { color, emissive: color, emissiveIntensity: 3, castShadow: false });
      const lamp = lights.addPointLight({
        color,
        intensity: 12,
        distance: 9,
        // 点光源阴影需要渲染 6 次，只给一个开启
        castShadow: i === 0,
      });
      lights.attachToBody(lamp, ball);
      return ball;
    });

    setInfo('拖拽发光球或按空格发射小球。切换光照 / 调色预设可对比效果（黑白、复古、鲜艳……）。');
    let timer = 0;
    let next = 0;
    return (dt) => {
      // 每隔一段时间踢一下某个发光球，让光影持续变化
      timer += dt;
      if (timer > 1.5) {
        timer = 0;
        const ball = balls[next++ % balls.length]!;
        ball.applyImpulse(v((rand() - 0.5) * 0.8, 0.6 + rand() * 0.4, (rand() - 0.5) * 0.8));
      }
    };
  },
};
