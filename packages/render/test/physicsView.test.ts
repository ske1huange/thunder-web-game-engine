import { BoxShape, PlaneShape, SphereShape, Vec3, World } from '@thunder/physics';
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import { PhysicsView, ThreeDebugRenderer } from '../src';

function scene() {
  const world = new World();
  const ground = world.createBody({ type: 'static' });
  ground.addCollider({ shape: new PlaneShape() });
  const box = world.createBody({ position: new Vec3(0, 3, 0) });
  box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
  box.addCollider({ shape: new SphereShape(0.3), position: new Vec3(0, 0.8, 0) });
  const zone = world.createBody({ type: 'static', position: new Vec3(3, 1, 0) });
  zone.addCollider({ shape: new BoxShape(new Vec3(1, 1, 1)), isSensor: true });
  return { world, ground, box, zone };
}

const meshes = (o: THREE.Object3D) => o.children as THREE.Mesh[];

describe('PhysicsView', () => {
  it('为每个刚体按碰撞体生成网格并同步位姿', () => {
    const { world, box } = scene();
    const view = new PhysicsView(world);
    view.sync();
    expect(view.root.children.length).toBe(3);
    const obj = view.getObject(box)!;
    expect(meshes(obj).length).toBe(2);
    expect(meshes(obj)[1]!.position.y).toBeCloseTo(0.8, 12);
    for (let i = 0; i < 30; i++) world.step(1 / 60);
    view.sync(1);
    expect(obj.position.y).toBeCloseTo(box.position.y, 12);
    view.sync(0);
    expect(obj.position.y).toBeCloseTo(box.previousPosition.y, 12);
    expect(view.bodyOf(meshes(obj)[0]!)).toBe(box);
  });

  it('传感器半透明且不投射阴影，外观可自定义', () => {
    const { world, box, zone } = scene();
    const view = new PhysicsView(world);
    view.sync();
    const sensorMesh = meshes(view.getObject(zone)!)[0]!;
    const sensorMat = sensorMesh.material as THREE.MeshStandardMaterial;
    expect(sensorMat.transparent).toBe(true);
    expect(sensorMesh.castShadow).toBe(false);
    view.setAppearance(box, {
      color: 0x123456,
      emissive: 0xff0000,
      emissiveIntensity: 2,
      metalness: 0.9,
    });
    view.sync();
    const mat = meshes(view.getObject(box)!)[0]!.material as THREE.MeshStandardMaterial;
    expect(mat.color.getHex()).toBe(0x123456);
    expect(mat.emissive.getHex()).toBe(0xff0000);
    expect(mat.emissiveIntensity).toBe(2);
    expect(mat.metalness).toBe(0.9);
    // 绑定了投射阴影光源的刚体可以关闭自身阴影
    view.setAppearance(box, { castShadow: false });
    view.sync();
    expect(meshes(view.getObject(box)!).every((m) => !m.castShadow)).toBe(true);
    view.clearAppearance(box);
    view.sync();
    expect(meshes(view.getObject(box)!).every((m) => m.castShadow)).toBe(true);
    const restored = meshes(view.getObject(box)!)[0]!.material as THREE.MeshStandardMaterial;
    expect(restored.color.getHex()).not.toBe(0x123456);
  });

  it('休眠刚体变暗，销毁的刚体被移除', () => {
    const { world, box } = scene();
    const view = new PhysicsView(world, { sleepingBrightness: 0.5, palette: [0xffffff] });
    view.sync();
    box.sleep();
    view.sync();
    const mat = meshes(view.getObject(box)!)[0]!.material as THREE.MeshStandardMaterial;
    expect(mat.color.r).toBeCloseTo(0.5, 6);
    world.destroyBody(box);
    view.sync();
    expect(view.getObject(box)).toBeUndefined();
    expect(view.root.children.length).toBe(2);
    view.clear();
    expect(view.root.children.length).toBe(0);
  });
});

describe('ThreeDebugRenderer', () => {
  it('收集调试线段并按帧清空', () => {
    const { world } = scene();
    world.step(1 / 60);
    const lines = new ThreeDebugRenderer();
    lines.begin();
    world.debugDraw(lines, { contacts: true });
    lines.end();
    expect(lines.lineCount).toBeGreaterThan(20);
    expect(lines.object.geometry.drawRange.count).toBe(lines.lineCount * 2);
    lines.begin();
    lines.end();
    expect(lines.lineCount).toBe(0);
    lines.dispose();
  });
});
