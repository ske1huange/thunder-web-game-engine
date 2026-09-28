import { BoxShape, PlaneShape, Quat, SphereShape, Vec3, World } from '@thunder/physics';
import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import {
  LightRig,
  directionFromAngles,
  lightingPresetNames,
  lightingPresets,
  snapShadowCenter,
} from '../src';

describe('方向与阴影纹素对齐', () => {
  it('方位角/仰角转换为单位方向', () => {
    const d = directionFromAngles(0, 90, new THREE.Vector3());
    expect(d.y).toBeCloseTo(Math.sin(THREE.MathUtils.degToRad(89)), 9);
    const east = directionFromAngles(90, 0, new THREE.Vector3());
    expect(east.x).toBeCloseTo(1, 9);
    expect(east.length()).toBeCloseTo(1, 12);
  });

  it('对齐只发生在光源视平面内，且坐标为纹素的整数倍', () => {
    const dir = directionFromAngles(35, 50, new THREE.Vector3());
    const halfExtent = 20;
    const mapSize = 1024;
    const texel = (2 * halfExtent) / mapSize;
    const right = new THREE.Vector3().crossVectors(new THREE.Vector3(0, 1, 0), dir).normalize();
    const up = new THREE.Vector3().crossVectors(dir, right);
    for (const f of [new THREE.Vector3(1.234, 0, -7.77), new THREE.Vector3(-30.5, 2, 12.01)]) {
      const c = snapShadowCenter(f, dir, halfExtent, mapSize, new THREE.Vector3());
      const delta = c.clone().sub(f);
      expect(Math.abs(delta.dot(dir))).toBeLessThan(1e-9);
      expect(delta.length()).toBeLessThanOrEqual(texel);
      const x = c.dot(right) / texel;
      const y = c.dot(up) / texel;
      expect(Math.abs(x - Math.round(x))).toBeLessThan(1e-6);
      expect(Math.abs(y - Math.round(y))).toBeLessThan(1e-6);
    }
  });

  it('沿光线方向移动跟随点不改变对齐后的平面坐标', () => {
    const dir = directionFromAngles(-60, 30, new THREE.Vector3());
    const f = new THREE.Vector3(3.3, 0, 4.4);
    const a = snapShadowCenter(f, dir, 25, 2048, new THREE.Vector3());
    const b = snapShadowCenter(
      f.clone().addScaledVector(dir, 5),
      dir,
      25,
      2048,
      new THREE.Vector3(),
    );
    expect(b.clone().sub(a).cross(dir).length()).toBeLessThan(1e-9);
  });
});

describe('LightRig', () => {
  it('应用每个光照预设，并设置场景背景、雾与环境强度', () => {
    const rig = new LightRig();
    const scene = new THREE.Scene();
    for (const name of lightingPresetNames) {
      rig.applyPreset(name, scene);
      const p = lightingPresets[name];
      expect(rig.preset).toBe(name);
      expect(rig.sun.intensity).toBe(p.sun.intensity);
      expect(rig.sun.color.getHex()).toBe(new THREE.Color(p.sun.color).getHex());
      expect(rig.hemisphere.intensity).toBe(p.hemisphere.intensity);
      expect((scene.background as THREE.Color).getHex()).toBe(
        new THREE.Color(p.background).getHex(),
      );
      expect(scene.environmentIntensity).toBe(p.environmentIntensity);
      if (p.fog) expect((scene.fog as THREE.Fog).far).toBe(p.fog.far);
      // 仰角越高，方向的 y 分量越大
      expect(rig.sunDirection.y).toBeCloseTo(
        Math.sin(THREE.MathUtils.degToRad(p.sun.elevation)),
        9,
      );
    }
  });

  it('主光与阴影相机跟随注视点', () => {
    const rig = new LightRig({ halfExtent: 10, distance: 40 });
    rig.update(new THREE.Vector3(100, 0, -50));
    const target = rig.sun.target.position;
    expect(target.distanceTo(new THREE.Vector3(100, 0, -50))).toBeLessThan(0.05);
    const toLight = rig.sun.position.clone().sub(target);
    expect(toLight.length()).toBeCloseTo(40, 9);
    expect(toLight.normalize().dot(rig.sunDirection)).toBeCloseTo(1, 9);
    expect(rig.sun.shadow.camera.right).toBe(10);
    expect(rig.sun.shadow.camera.far).toBe(80);
  });

  it('阴影质量', () => {
    const rig = new LightRig();
    rig.setShadowQuality('high');
    expect(rig.sun.shadow.mapSize.x).toBe(4096);
    expect(rig.sun.castShadow).toBe(true);
    rig.setShadowQuality('off');
    expect(rig.sun.castShadow).toBe(false);
    rig.applyPreset('day');
    expect(rig.sun.castShadow).toBe(false);
    rig.setShadowQuality('low');
    expect(rig.sun.castShadow).toBe(true);
    expect(rig.sun.shadow.mapSize.x).toBe(1024);
  });

  it('点光源 / 聚光灯的添加与移除', () => {
    const rig = new LightRig();
    const p = rig.addPointLight({ color: 0xff0000, intensity: 5, position: { x: 1, y: 2, z: 3 } });
    const s = rig.addSpotLight({
      position: { x: 0, y: 5, z: 0 },
      target: { x: 0, y: 0, z: 0 },
      castShadow: true,
    });
    expect(rig.lights.length).toBe(2);
    expect(p.position.toArray()).toEqual([1, 2, 3]);
    expect(s.castShadow).toBe(true);
    expect(rig.group.children).toContain(s.target);
    rig.removeLight(p);
    expect(rig.lights).toEqual([s]);
    rig.clearLights();
    expect(rig.lights.length).toBe(0);
    expect(rig.group.children).not.toContain(s);
  });

  it('光源绑定到刚体：随刚体移动与旋转，刚体销毁后自动解除', () => {
    const world = new World({ gravity: new Vec3(0, 0, 0) });
    const body = world.createBody({
      position: new Vec3(0, 2, 0),
      rotation: new Quat().setFromAxisAngle(new Vec3(0, 0, 1), Math.PI / 2),
      linearVelocity: new Vec3(3, 0, 0),
    });
    body.addCollider({ shape: new SphereShape(0.3) });
    const rig = new LightRig();
    const lamp = rig.addPointLight();
    const spot = rig.addSpotLight();
    rig.attachToBody(lamp, body, { offset: { x: 1, y: 0, z: 0 } });
    rig.attachToBody(spot, body, { direction: { x: 1, y: 0, z: 0 } });
    for (let i = 0; i < 60; i++) world.step(1 / 60);
    rig.update(undefined, 1);
    // 局部 +X 偏移经过绕 Z 旋转 90° 后指向世界 +Y
    expect(lamp.position.x).toBeCloseTo(body.position.x, 6);
    expect(lamp.position.y).toBeCloseTo(body.position.y + 1, 6);
    const dir = spot.target.position.clone().sub(spot.position);
    expect(dir.y).toBeCloseTo(1, 6);
    // 插值：alpha = 0 时使用上一步的位置
    rig.update(undefined, 0);
    expect(lamp.position.x).toBeCloseTo(body.previousPosition.x, 6);
    world.destroyBody(body);
    const before = lamp.position.clone();
    rig.update(undefined, 1);
    expect(lamp.position.equals(before)).toBe(true);
  });

  it('与物理世界配合：地面上的刚体被正常包含在阴影范围内', () => {
    const world = new World();
    const ground = world.createBody({ type: 'static' });
    ground.addCollider({ shape: new PlaneShape() });
    const box = world.createBody({ position: new Vec3(5, 3, 5) });
    box.addCollider({ shape: new BoxShape(new Vec3(0.5, 0.5, 0.5)) });
    const rig = new LightRig({ halfExtent: 8 });
    rig.update(new THREE.Vector3(box.position.x, 0, box.position.z));
    rig.sun.updateMatrixWorld();
    rig.sun.target.updateMatrixWorld();
    rig.sun.shadow.updateMatrices(rig.sun);
    // 把刚体位置投影到阴影相机裁剪空间，应位于 [-1, 1] 内
    const p = new THREE.Vector3(box.position.x, box.position.y, box.position.z);
    p.applyMatrix4(rig.sun.shadow.matrix);
    expect(p.x).toBeGreaterThan(0);
    expect(p.x).toBeLessThan(1);
    expect(p.y).toBeGreaterThan(0);
    expect(p.y).toBeLessThan(1);
  });
});
