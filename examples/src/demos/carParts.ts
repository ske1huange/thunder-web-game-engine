import { BoxShape, CylinderShape, Quat, Vec3 } from '@thunder/physics';
import { PhysicsCar, isHingedPart, isSlidingPart, isWiperPart } from '@thunder/render';
import type { Demo } from './types';
import { addBox, v } from './helpers';
import { buildSportsCar, sportsCarBinding } from './sportsCar';

const DRIVE_KEYS = ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' '];
/** 面板按钮：名称 → 快捷键 */
const CONTROLS: [string, string][] = [
  ['左车门', '1'],
  ['右车门', '2'],
  ['引擎盖', '3'],
  ['后备箱', '4'],
  ['天窗', '5'],
  ['尾翼', '6'],
  ['雨刮', '7'],
  ['大灯', '8'],
];

export const carParts: Demo = {
  id: 'car-parts',
  name: '车模部件',
  description:
    '车门、引擎盖、后备箱是铰链刚体（锁止 / 马达开合），开着车门行驶时车门会被甩动，甩回原位会重新锁上；天窗、尾翼是滑动关节（尾翼高速自动升起），雨刮是往复马达；车轮、卡钳、方向盘随悬挂与转向运动，车身姿态由悬挂决定。WASD 驾驶，空格手刹，1–8 或右下角按钮控制部件。',
  camera: { position: [5.5, 3.2, 6.5], target: [0, 0.8, 0] },
  lighting: 'sunset',
  grading: 'cinematic',
  keys: [...DRIVE_KEYS, ...CONTROLS.map(([, k]) => k)],
  setup({ world, view, lights, panel, setInfo }) {
    // ---- 场地：减速带、斜坡、路锥 ----
    for (let i = 0; i < 5; i++) {
      const bump = world.createBody({
        type: 'static',
        position: v(0, -0.02, -14 - i * 2.2),
        rotation: new Quat().setFromAxisAngle(v(0, 0, 1), Math.PI / 2),
      });
      bump.addCollider({ shape: new CylinderShape(0.1, 3, 16) });
      view.setAppearance(bump, { color: 0xe0b020 });
    }
    const ramp = world.createBody({
      type: 'static',
      position: v(8, 0.35, -18),
      rotation: new Quat().setFromAxisAngle(v(1, 0, 0), 0.16),
    });
    ramp.addCollider({ shape: new BoxShape(v(2, 0.2, 3)) });
    view.setAppearance(ramp, { color: 0xb8b0a0 });
    for (let i = 0; i < 6; i++) {
      addBox(world, v(-6, 0.3, -8 - i * 3), v(0.25, 0.3, 0.25), { density: 20 });
    }

    // ---- 车 ----
    const model = buildSportsCar();
    const car = new PhysicsCar(world, model, {
      ...sportsCarBinding,
      lights: { ...sportsCarBinding.lights, rig: lights },
    });
    view.root.add(car.root);
    for (const b of car.bodies) view.setAppearance(b, { visible: false });

    // 雨刮两根一起控制
    const toggle = (name: string): void => {
      if (name === '雨刮') {
        car.toggle('左雨刮');
        car.toggle('右雨刮');
      } else if (name === '大灯') {
        car.setHeadlights(!car.headlights);
      } else {
        car.toggle(name);
      }
      refresh();
    };
    const buttons = new Map<string, HTMLButtonElement>();
    for (const [name, key] of CONTROLS) {
      const b = document.createElement('button');
      b.type = 'button';
      b.innerHTML = `<kbd>${key}</kbd>${name}`;
      b.addEventListener('click', () => toggle(name));
      panel.append(b);
      buttons.set(name, b);
    }
    const isOn = (name: string): boolean => {
      if (name === '大灯') return car.headlights;
      const p = car.part(name === '雨刮' ? '左雨刮' : name);
      if (isWiperPart(p)) return p.isRunning;
      if (isHingedPart(p) || isSlidingPart(p)) return p.state === 'open' || p.state === 'opening';
      return false;
    };
    const refresh = (): void => {
      for (const [name, b] of buttons) b.classList.toggle('on', isOn(name));
    };

    // ---- 输入 ----
    const pressed = new Set<string>();
    const onDown = (e: KeyboardEvent): void => {
      const k = e.key.toLowerCase();
      const control = CONTROLS.find(([, key]) => key === k);
      if (control && !e.repeat) {
        toggle(control[0]);
        e.preventDefault();
        return;
      }
      if (!DRIVE_KEYS.includes(k)) return;
      pressed.add(k);
      e.preventDefault();
    };
    const onUp = (e: KeyboardEvent): void => {
      pressed.delete(e.key.toLowerCase());
    };
    window.addEventListener('keydown', onDown);
    window.addEventListener('keyup', onUp);

    const describe = (name: string): string => {
      const p = car.part(name);
      if (isHingedPart(p)) return `${Math.round((Math.abs(p.angle) * 180) / Math.PI)}°`;
      if (isSlidingPart(p)) return `${Math.round(p.openness * 100)}%`;
      return '';
    };
    const follow = new Vec3();
    let infoTimer = 0;
    return {
      update(dt) {
        const up = pressed.has('w') || pressed.has('arrowup');
        const down = pressed.has('s') || pressed.has('arrowdown');
        const left = pressed.has('a') || pressed.has('arrowleft');
        const right = pressed.has('d') || pressed.has('arrowright');
        car.setInput({
          throttle: (up ? 1 : 0) - (down ? 1 : 0),
          steer: (left ? 1 : 0) - (right ? 1 : 0),
          handbrake: pressed.has(' '),
        });
        infoTimer += dt;
        if (infoTimer > 0.2) {
          infoTimer = 0;
          refresh();
          const doors = ['左车门', '右车门', '引擎盖', '后备箱', '天窗', '尾翼']
            .map((n) => `${n} ${describe(n)}`)
            .join('，');
          setInfo(`${(car.speed * 3.6).toFixed(0)} km/h · ${doors}`);
        }
      },
      render(alpha) {
        car.sync(alpha);
      },
      follow(alpha) {
        car.chassis.interpolate(alpha, follow);
        follow.y += 0.4;
        return follow;
      },
      dispose() {
        window.removeEventListener('keydown', onDown);
        window.removeEventListener('keyup', onUp);
      },
    };
  },
};
