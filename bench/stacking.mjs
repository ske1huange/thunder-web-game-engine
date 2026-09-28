// 物理引擎基准：活跃状态下 World.step 的耗时（关闭休眠，让所有物体持续参与求解）。
// 使用构建产物（packages/physics/dist），运行：pnpm bench
import {
  BoxShape,
  PlaneShape,
  Quat,
  SphereShape,
  Vec3,
  World,
} from '../packages/physics/dist/index.js';

function rng(seed) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

function container(world, half) {
  const ground = world.createBody({ type: 'static' });
  ground.addCollider({ shape: new PlaneShape() });
  const walls = world.createBody({ type: 'static' });
  for (const [x, z, hx, hz] of [
    [half, 0, 0.5, half],
    [-half, 0, 0.5, half],
    [0, half, half, 0.5],
    [0, -half, half, 0.5],
  ]) {
    walls.addCollider({ shape: new BoxShape(new Vec3(hx, 4, hz)), position: new Vec3(x, 4, z) });
  }
}

/** count 个物体规则排布后落入容器，预模拟 2 秒形成堆积 */
function pile(count, kind) {
  const world = new World({ enableSleep: false });
  const half = Math.ceil(Math.sqrt(count) * 0.35) + 1;
  container(world, half);
  const rand = rng(count);
  const shape = kind === 'box' ? new BoxShape(new Vec3(0.3, 0.3, 0.3)) : new SphereShape(0.3);
  const perRow = Math.floor((2 * half - 2) / 0.7);
  const perLayer = perRow * perRow;
  for (let i = 0; i < count; i++) {
    const layer = Math.floor(i / perLayer);
    const idx = i % perLayer;
    const b = world.createBody({
      position: new Vec3(
        -half + 1 + (idx % perRow) * 0.7,
        0.5 + layer * 0.7,
        -half + 1 + Math.floor(idx / perRow) * 0.7,
      ),
      rotation: new Quat().setFromEuler(rand() * 3, rand() * 3, rand() * 3),
    });
    b.addCollider({ shape });
  }
  for (let i = 0; i < 120; i++) world.step(1 / 60);
  return world;
}

function pyramid(rows) {
  const world = new World({ enableSleep: false });
  const ground = world.createBody({ type: 'static' });
  ground.addCollider({ shape: new PlaneShape() });
  for (let row = 0; row < rows; row++) {
    for (let i = 0; i < rows - row; i++) {
      const b = world.createBody({
        position: new Vec3((i - (rows - row - 1) / 2) * 1.02, 0.5 + row, 0),
      });
      b.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
    }
  }
  for (let i = 0; i < 60; i++) world.step(1 / 60);
  return world;
}

function measure(name, world, steps = 120) {
  const times = [];
  for (let i = 0; i < steps; i++) {
    const t0 = performance.now();
    world.step(1 / 60);
    times.push(performance.now() - t0);
  }
  times.sort((a, b) => a - b);
  const mean = times.reduce((a, b) => a + b, 0) / times.length;
  let points = 0;
  for (const c of world.contacts) if (c.touching) points += c.manifold.pointCount;
  return {
    场景: name,
    动态刚体: world.bodies.filter((b) => b.isDynamic()).length,
    接触点: points,
    '平均 ms/步': mean.toFixed(2),
    '中位 ms/步': times[Math.floor(times.length / 2)].toFixed(2),
    'p95 ms/步': times[Math.floor(times.length * 0.95)].toFixed(2),
  };
}

const rows = [
  measure('100 个箱子堆积', pile(100, 'box')),
  measure('金字塔 20 层', pyramid(20)),
  measure('500 个箱子堆积', pile(500, 'box')),
  measure('1000 个箱子堆积', pile(1000, 'box')),
  measure('1000 个球堆积', pile(1000, 'sphere')),
];
console.log(`Node ${process.version}，1/60 s，4 子步，关闭休眠`);
console.table(rows);
