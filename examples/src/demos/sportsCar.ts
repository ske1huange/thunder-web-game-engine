import type { PhysicsCarOptions } from '@thunder/render';
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';

/**
 * 用 three.js 几何体程序化搭建的跑车模型（车头朝 -Z，车轮着地于 y = 0）。
 * 可动部件都是独立命名的节点，与从 glTF 导入的车模结构相同：
 *
 * Body（车身，含方向盘 SteeringWheel、车灯）、Door_L / Door_R、Hood、Trunk、Sunroof、Spoiler、
 * Wiper_L / Wiper_R、Wheel_FL / FR / RL / RR、Caliper_FL / FR / RL / RR
 */
export function buildSportsCar(paintColor: THREE.ColorRepresentation = 0x1f6feb): THREE.Group {
  const paint = new THREE.MeshPhysicalMaterial({
    color: paintColor,
    metalness: 0.55,
    roughness: 0.32,
    clearcoat: 1,
    clearcoatRoughness: 0.08,
  });
  const dark = new THREE.MeshStandardMaterial({ color: 0x1a1c20, roughness: 0.8 });
  const carbon = new THREE.MeshStandardMaterial({
    color: 0x24262b,
    roughness: 0.45,
    metalness: 0.3,
  });
  const glass = new THREE.MeshPhysicalMaterial({
    color: 0x0d1b2a,
    roughness: 0.05,
    metalness: 0.1,
    transparent: true,
    opacity: 0.55,
  });
  const interior = new THREE.MeshStandardMaterial({ color: 0x3a2f2a, roughness: 0.9 });
  const chrome = new THREE.MeshStandardMaterial({ color: 0xd0d4dc, metalness: 1, roughness: 0.25 });
  const engineMat = new THREE.MeshStandardMaterial({
    color: 0x4a505a,
    metalness: 0.7,
    roughness: 0.4,
  });
  const red = new THREE.MeshStandardMaterial({ color: 0xd62828, roughness: 0.4, metalness: 0.3 });
  const tire = new THREE.MeshStandardMaterial({ color: 0x151517, roughness: 0.95 });
  const headlight = new THREE.MeshStandardMaterial({
    color: 0xe8eef5,
    roughness: 0.15,
    metalness: 0.2,
  });
  const tail = new THREE.MeshStandardMaterial({ color: 0x5a0808, roughness: 0.3 });
  const reverseMat = new THREE.MeshStandardMaterial({ color: 0xcfd3d8, roughness: 0.3 });

  const mesh = (
    geometry: THREE.BufferGeometry,
    material: THREE.Material,
    parent: THREE.Object3D,
    x: number,
    y: number,
    z: number,
    name = '',
  ): THREE.Mesh => {
    const m = new THREE.Mesh(geometry, material);
    m.position.set(x, y, z);
    m.castShadow = true;
    m.receiveShadow = true;
    m.name = name;
    parent.add(m);
    return m;
  };
  const box = (w: number, h: number, d: number) => new THREE.BoxGeometry(w, h, d);
  const rounded = (w: number, h: number, d: number, r = 0.04) =>
    new RoundedBoxGeometry(w, h, d, 3, Math.min(r, w / 2, h / 2, d / 2));
  const group = (name: string, parent: THREE.Object3D, x = 0, y = 0, z = 0): THREE.Group => {
    const g = new THREE.Group();
    g.name = name;
    g.position.set(x, y, z);
    parent.add(g);
    return g;
  };

  const car = new THREE.Group();
  car.name = 'SportsCar';
  const body = group('Body', car);

  // ---- 车身：底盘、前舱、后舱、保险杠 ----
  mesh(box(1.8, 0.18, 4.3), dark, body, 0, 0.37, 0);
  mesh(rounded(1.86, 0.3, 1.56, 0.06), paint, body, 0, 0.59, -1.42);
  mesh(rounded(1.86, 0.3, 1.36, 0.06), paint, body, 0, 0.59, 1.52);
  // 前舱四周的挡板（引擎盖合上时轮廓连续）
  for (const x of [-0.89, 0.89]) {
    mesh(box(0.08, 0.1, 1.5), paint, body, x, 0.77, -1.42);
    mesh(box(0.08, 0.1, 0.7), paint, body, x, 0.77, 1.66);
  }
  mesh(box(1.86, 0.1, 0.08), paint, body, 0, 0.77, -2.17);
  // 引擎与后备箱底板
  mesh(rounded(0.9, 0.08, 0.7, 0.02), engineMat, body, 0, 0.76, -1.5);
  mesh(box(0.6, 0.05, 0.28), red, body, 0, 0.795, -1.5);
  mesh(box(1.6, 0.04, 0.6), dark, body, 0, 0.75, 1.66);
  // 车尾平台（尾翼立在上面）
  mesh(rounded(1.86, 0.12, 0.2, 0.03), paint, body, 0, 0.78, 2.1);
  mesh(rounded(1.84, 0.14, 0.14, 0.05), dark, body, 0, 0.42, -2.2);
  mesh(rounded(1.84, 0.14, 0.14, 0.05), dark, body, 0, 0.42, 2.2);
  mesh(box(1.1, 0.1, 0.04), dark, body, 0, 0.56, -2.21);

  // ---- 座舱 ----
  for (const x of [-0.4, 0.4]) {
    mesh(rounded(0.5, 0.12, 0.5, 0.04), interior, body, x, 0.53, 0.3);
    const back = mesh(rounded(0.5, 0.55, 0.12, 0.04), interior, body, x, 0.83, 0.6);
    back.rotation.x = 0.2;
  }
  mesh(rounded(1.62, 0.18, 0.34, 0.04), dark, body, 0, 0.84, -0.5);
  // 方向盘（绕自身法线转动）
  const steer = group('SteeringWheel', body, -0.4, 0.93, -0.26);
  steer.rotation.x = -0.35;
  mesh(new THREE.TorusGeometry(0.16, 0.022, 8, 28), dark, steer, 0, 0, 0);
  mesh(box(0.3, 0.035, 0.02), chrome, steer, 0, 0, 0);
  mesh(box(0.035, 0.16, 0.02), chrome, steer, 0, -0.08, 0);
  const column = mesh(
    new THREE.CylinderGeometry(0.025, 0.025, 0.25, 8),
    dark,
    body,
    -0.4,
    0.88,
    -0.38,
  );
  column.rotation.x = Math.PI / 2 - 0.35;

  // ---- 风挡、车顶、后窗、立柱 ----
  const windshield = mesh(box(1.6, 0.015, 0.62), glass, body, 0, 1.01, -0.42);
  windshield.rotation.x = -0.67;
  const rearWindow = mesh(box(1.5, 0.015, 0.62), glass, body, 0, 1.015, 1.03);
  rearWindow.rotation.x = 0.64;
  for (const x of [-0.79, 0.79]) {
    const a = mesh(box(0.06, 0.05, 0.64), paint, body, x, 1.01, -0.42);
    a.rotation.x = -0.67;
    const c = mesh(box(0.14, 0.05, 0.64), paint, body, x * 0.96, 1.015, 1.03);
    c.rotation.x = 0.64;
  }
  // 车顶框（中间是天窗开口）
  mesh(box(1.44, 0.04, 0.2), paint, body, 0, 1.22, -0.1);
  mesh(box(1.44, 0.04, 0.3), paint, body, 0, 1.22, 0.63);
  for (const x of [-0.61, 0.61]) mesh(box(0.22, 0.04, 0.48), paint, body, x, 1.22, 0.24);

  // ---- 车灯 ----
  for (const [side, x] of [
    ['L', -0.62],
    ['R', 0.62],
  ] as const) {
    mesh(rounded(0.38, 0.08, 0.06, 0.02), headlight, body, x, 0.66, -2.2, `Headlight_${side}`);
    mesh(rounded(0.42, 0.06, 0.05, 0.02), tail, body, x, 0.7, 2.2, `BrakeLight_${side}`);
  }
  mesh(box(0.18, 0.05, 0.04), reverseMat, body, 0, 0.56, 2.26, 'ReverseLight');

  // ---- 车门（含车窗、后视镜、门把手） ----
  for (const [side, x] of [
    ['L', -0.935],
    ['R', 0.935],
  ] as const) {
    const s = Math.sign(x);
    const door = group(`Door_${side}`, car, x, 0.67, 0.1);
    mesh(rounded(0.07, 0.46, 1.44, 0.03), paint, door, 0, 0, 0);
    mesh(box(0.02, 0.3, 1.1), glass, door, -s * 0.02, 0.38, 0.02);
    mesh(box(0.03, 0.025, 1.12), dark, door, -s * 0.02, 0.535, 0.02);
    mesh(box(0.03, 0.03, 0.14), chrome, door, s * 0.04, 0.1, 0.35);
    mesh(rounded(0.16, 0.1, 0.18, 0.03), paint, door, s * 0.1, 0.28, -0.6);
  }

  // ---- 引擎盖、后备箱、天窗、尾翼、雨刮 ----
  const hood = group('Hood', car, 0, 0.84, -1.41);
  mesh(rounded(1.76, 0.035, 1.5, 0.015), paint, hood, 0, 0, 0);
  const trunk = group('Trunk', car, 0, 0.84, 1.65);
  mesh(rounded(1.76, 0.035, 0.68, 0.015), paint, trunk, 0, 0, 0);
  const sunroof = group('Sunroof', car, 0, 1.25, 0.24);
  mesh(box(0.98, 0.02, 0.47), glass, sunroof, 0, 0, 0);
  const spoiler = group('Spoiler', car, 0, 0.98, 2.1);
  mesh(rounded(1.6, 0.04, 0.26, 0.015), carbon, spoiler, 0, 0, 0);
  for (const x of [-0.8, 0.8]) mesh(box(0.02, 0.12, 0.3), carbon, spoiler, x, -0.03, 0);
  for (const x of [-0.5, 0.5]) mesh(box(0.04, 0.13, 0.08), carbon, spoiler, x, -0.08, 0.02);
  for (const [name, x] of [
    ['Wiper_L', -0.4],
    ['Wiper_R', 0.3],
  ] as const) {
    const w = group(name, car, x, 0.875, -0.64);
    mesh(box(0.6, 0.015, 0.03), dark, w, 0, 0, 0);
  }

  // ---- 车轮与刹车卡钳 ----
  const tireGeo = new THREE.CylinderGeometry(0.34, 0.34, 0.24, 32).rotateZ(Math.PI / 2);
  const rimGeo = new THREE.CylinderGeometry(0.22, 0.22, 0.245, 24).rotateZ(Math.PI / 2);
  const discGeo = new THREE.CylinderGeometry(0.18, 0.18, 0.02, 24).rotateZ(Math.PI / 2);
  const spokeGeo = box(0.02, 0.045, 0.4);
  for (const [name, x, z] of [
    ['FL', -0.84, -1.4],
    ['FR', 0.84, -1.4],
    ['RL', -0.84, 1.35],
    ['RR', 0.84, 1.35],
  ] as const) {
    const s = Math.sign(x);
    const wheel = group(`Wheel_${name}`, car, x, 0.34, z);
    mesh(tireGeo, tire, wheel, 0, 0, 0);
    mesh(rimGeo, dark, wheel, 0, 0, 0);
    mesh(discGeo, chrome, wheel, -s * 0.02, 0, 0);
    for (let k = 0; k < 5; k++) {
      const spoke = mesh(spokeGeo, chrome, wheel, s * 0.125, 0, 0);
      spoke.rotation.x = (k * Math.PI * 2) / 5;
    }
    const caliper = group(`Caliper_${name}`, car, x, 0.34, z);
    mesh(rounded(0.07, 0.15, 0.11, 0.02), red, caliper, s * 0.03, 0.12, 0.1);
  }

  return car;
}

/** 与 buildSportsCar 的节点对应的绑定配置 */
export const sportsCarBinding: Omit<PhysicsCarOptions, 'lights'> & {
  lights: { head: string[]; brake: string[]; reverse: string[] };
} = {
  mass: 1250,
  wheels: [
    { node: 'Wheel_FL', caliper: 'Caliper_FL' },
    { node: 'Wheel_FR', caliper: 'Caliper_FR' },
    { node: 'Wheel_RL', caliper: 'Caliper_RL' },
    { node: 'Wheel_RR', caliper: 'Caliper_RR' },
  ],
  suspension: { restLength: 0.3, stiffness: 35, rollInfluence: 0.15 },
  hinges: [
    { node: 'Door_L', name: '左车门', edge: 'front', angle: 70 },
    { node: 'Door_R', name: '右车门', edge: 'front', angle: 70 },
    { node: 'Hood', name: '引擎盖', edge: 'rear', angle: 55 },
    { node: 'Trunk', name: '后备箱', edge: 'front', angle: 60 },
  ],
  sliders: [
    { node: 'Sunroof', name: '天窗', direction: 'back', travel: 0.42 },
    { node: 'Spoiler', name: '尾翼', direction: 'up', travel: 0.16, autoAbove: 16 },
  ],
  wipers: [
    { node: 'Wiper_L', name: '左雨刮', pivot: 'left', sweep: 85 },
    { node: 'Wiper_R', name: '右雨刮', pivot: 'left', sweep: 85 },
  ],
  steeringWheel: { node: 'SteeringWheel', ratio: 8 },
  lights: {
    head: ['Headlight_L', 'Headlight_R'],
    brake: ['BrakeLight_L', 'BrakeLight_R'],
    reverse: ['ReverseLight'],
  },
};
