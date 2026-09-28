import { MouseJoint, PlaneShape, type RigidBody, SphereShape, Vec3, World } from '@thunder/physics';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { type Demo, demos } from './demos';
import type { DemoHooks } from './demos/types';
import { PhysicsView } from './PhysicsView';
import { ThreeDebugRenderer } from './ThreeDebugRenderer';

const FIXED_DT = 1 / 60;

// ---------------------------------------------------------------------------
// three.js 场景
// ---------------------------------------------------------------------------
const stage = document.getElementById('stage')!;
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
stage.prepend(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1115);
scene.fog = new THREE.Fog(0x0f1115, 40, 120);

const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 500);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.mouseButtons = { LEFT: null, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.ROTATE };

scene.add(new THREE.HemisphereLight(0xdde6ff, 0x30303a, 0.9));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(12, 24, 10);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.left = -25;
sun.shadow.camera.right = 25;
sun.shadow.camera.top = 25;
sun.shadow.camera.bottom = -25;
sun.shadow.camera.far = 80;
sun.shadow.bias = -0.0005;
scene.add(sun);

const grid = new THREE.GridHelper(100, 100, 0x3a4154, 0x262c3a);
grid.position.y = 0.001;
scene.add(grid);

const debugRenderer = new ThreeDebugRenderer();
const overlayRenderer = new ThreeDebugRenderer();
scene.add(debugRenderer.object, overlayRenderer.object);

function resize(): void {
  const w = stage.clientWidth;
  const h = stage.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();

// ---------------------------------------------------------------------------
// 物理世界与 demo 管理
// ---------------------------------------------------------------------------
let world!: World;
let view!: PhysicsView;
let ground!: RigidBody;
let hooks: DemoHooks = {};
let current: Demo = demos[0]!;
let simTime = 0;
let accumulator = 0;
let paused = false;
let stepOnce = false;
let showDebug = false;
let stepTimeAvg = 0;

const infoEl = document.getElementById('info')!;

function loadDemo(demo: Demo): void {
  if (view) {
    scene.remove(view.root);
    view.clear();
  }
  current = demo;
  world = new World();
  view = new PhysicsView(world);
  scene.add(view.root);
  ground = world.createBody({ type: 'static' });
  if (demo.ground !== false) ground.addCollider({ shape: new PlaneShape(), friction: 0.6 });
  infoEl.textContent = '';
  simTime = 0;
  accumulator = 0;
  dragJoint = null;
  const result = demo.setup({
    world,
    ground,
    view,
    overlay: overlayRenderer,
    setInfo: (text) => {
      infoEl.textContent = text;
    },
  });
  hooks = typeof result === 'function' ? { update: result } : (result ?? {});
  const cam = demo.camera ?? { position: [12, 8, 14], target: [0, 2, 0] };
  camera.position.set(...cam.position);
  controls.target.set(...cam.target);
  controls.update();
  document.getElementById('demo-title')!.textContent = demo.name;
  document.getElementById('demo-desc')!.textContent = demo.description;
  for (const a of document.querySelectorAll<HTMLAnchorElement>('#demo-list a')) {
    a.classList.toggle('active', a.dataset.id === demo.id);
  }
  exposeForTests();
}

function stepWorld(): void {
  hooks.update?.(FIXED_DT, simTime);
  world.step(FIXED_DT);
  simTime += FIXED_DT;
  stepTimeAvg = stepTimeAvg * 0.95 + world.stats.stepTime * 0.05;
}

// ---------------------------------------------------------------------------
// 交互：拖拽（MouseJoint）与发射小球
// ---------------------------------------------------------------------------
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
const dragPlane = new THREE.Plane();
const dragPoint = new THREE.Vector3();
let dragJoint: MouseJoint | null = null;

function updatePointer(e: PointerEvent): void {
  const rect = renderer.domElement.getBoundingClientRect();
  pointer.set(
    ((e.clientX - rect.left) / rect.width) * 2 - 1,
    -((e.clientY - rect.top) / rect.height) * 2 + 1,
  );
  raycaster.setFromCamera(pointer, camera);
}

renderer.domElement.addEventListener('pointerdown', (e) => {
  if (e.button !== 0) return;
  updatePointer(e);
  const o = raycaster.ray.origin;
  const d = raycaster.ray.direction;
  const hit = world.raycast(new Vec3(o.x, o.y, o.z), new Vec3(d.x, d.y, d.z), 500);
  if (!hit || !hit.body.isDynamic()) return;
  dragJoint = world.addJoint(
    new MouseJoint({
      bodyA: ground,
      bodyB: hit.body,
      anchor: hit.point,
      maxForce: 1000 * hit.body.mass,
      hertz: 5,
      dampingRatio: 0.7,
    }),
  );
  const normal = camera.getWorldDirection(new THREE.Vector3()).negate();
  dragPlane.setFromNormalAndCoplanarPoint(
    normal,
    new THREE.Vector3(hit.point.x, hit.point.y, hit.point.z),
  );
  renderer.domElement.setPointerCapture(e.pointerId);
});

renderer.domElement.addEventListener('pointermove', (e) => {
  if (!dragJoint) return;
  updatePointer(e);
  if (raycaster.ray.intersectPlane(dragPlane, dragPoint)) {
    dragJoint.setTarget(new Vec3(dragPoint.x, dragPoint.y, dragPoint.z));
  }
});

function endDrag(): void {
  if (dragJoint && dragJoint.world) world.removeJoint(dragJoint);
  dragJoint = null;
}
renderer.domElement.addEventListener('pointerup', endDrag);
renderer.domElement.addEventListener('pointercancel', endDrag);
renderer.domElement.addEventListener('contextmenu', (e) => e.preventDefault());

function shootBall(): void {
  const dir = camera.getWorldDirection(new THREE.Vector3());
  const ball = world.createBody({
    position: new Vec3(camera.position.x, camera.position.y, camera.position.z),
    linearVelocity: new Vec3(dir.x * 40, dir.y * 40, dir.z * 40),
    isBullet: true,
  });
  ball.addCollider({ shape: new SphereShape(0.35), density: 20, friction: 0.4, restitution: 0.2 });
}

window.addEventListener('keydown', (e) => {
  if (e.target instanceof HTMLInputElement) return;
  switch (e.key.toLowerCase()) {
    case ' ':
      e.preventDefault();
      shootBall();
      break;
    case 'p':
      togglePause();
      break;
    case 'n':
      stepOnce = true;
      break;
    case 'r':
      loadDemo(current);
      break;
    case 'd':
      toggleDebug();
      break;
  }
});

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
const list = document.getElementById('demo-list')!;
for (const demo of demos) {
  const a = document.createElement('a');
  a.href = `#${demo.id}`;
  a.textContent = demo.name;
  a.dataset.id = demo.id;
  list.append(a);
}

function demoFromHash(): Demo {
  const id = location.hash.slice(1);
  return demos.find((d) => d.id === id) ?? demos[0]!;
}
window.addEventListener('hashchange', () => loadDemo(demoFromHash()));

const btnPause = document.getElementById('btn-pause')!;
const btnDebug = document.getElementById('btn-debug')!;
function togglePause(): void {
  paused = !paused;
  btnPause.textContent = paused ? '继续' : '暂停';
  btnPause.classList.toggle('on', paused);
}
function toggleDebug(): void {
  showDebug = !showDebug;
  btnDebug.classList.toggle('on', showDebug);
}
btnPause.addEventListener('click', togglePause);
btnDebug.addEventListener('click', toggleDebug);
document.getElementById('btn-step')!.addEventListener('click', () => {
  stepOnce = true;
});
document.getElementById('btn-reset')!.addEventListener('click', () => loadDemo(current));

const statsEl = document.getElementById('stats')!;
let fps = 60;
function updateStats(): void {
  const s = world.stats;
  const rows: [string, string][] = [
    ['刚体（唤醒）', `${s.bodies}（${s.awakeBodies}）`],
    ['接触（接触中）', `${s.contacts}（${s.touchingContacts}）`],
    ['关节', `${s.joints}`],
    ['每步耗时', `${stepTimeAvg.toFixed(2)} ms`],
    ['帧率', `${fps.toFixed(0)} FPS`],
  ];
  statsEl.innerHTML = rows.map(([k, v]) => `<tr><td>${k}</td><td>${v}</td></tr>`).join('');
}

// ---------------------------------------------------------------------------
// 主循环
// ---------------------------------------------------------------------------
const clock = new THREE.Clock();
let statsTimer = 0;

function frame(): void {
  requestAnimationFrame(frame);
  const dt = Math.min(clock.getDelta(), 0.1);
  fps = fps * 0.95 + (dt > 0 ? 1 / dt : 60) * 0.05;

  if (!paused) {
    accumulator += dt;
    let steps = 0;
    while (accumulator >= FIXED_DT && steps < 4) {
      stepWorld();
      accumulator -= FIXED_DT;
      steps++;
    }
    if (steps === 4) accumulator = 0;
  } else if (stepOnce) {
    stepWorld();
  }
  stepOnce = false;
  const alpha = paused ? 1 : accumulator / FIXED_DT;
  view.sync(alpha);

  overlayRenderer.begin();
  hooks.render?.();
  overlayRenderer.end();

  debugRenderer.begin();
  if (showDebug) world.debugDraw(debugRenderer, { contacts: true, joints: true });
  debugRenderer.end();

  controls.update();
  renderer.render(scene, camera);

  statsTimer += dt;
  if (statsTimer > 0.25) {
    statsTimer = 0;
    updateStats();
  }
}

/** 暴露给端到端测试 */
function exposeForTests(): void {
  (window as unknown as { __thunder: unknown }).__thunder = {
    get world() {
      return world;
    },
    demo: current.id,
    get simTime() {
      return simTime;
    },
  };
}

loadDemo(demoFromHash());
frame();
