import type { DebugDrawer, Vec3 } from '@thunder/physics';
import * as THREE from 'three';

/**
 * DebugDrawer 的 three.js 实现：把引擎输出的线段收集到一个 LineSegments 中。
 */
export class ThreeDebugRenderer implements DebugDrawer {
  readonly object: THREE.LineSegments;
  private positions = new Float32Array(0);
  private colors = new Float32Array(0);
  private count = 0;
  private readonly geometry = new THREE.BufferGeometry();
  private readonly color = new THREE.Color();

  constructor() {
    const material = new THREE.LineBasicMaterial({
      vertexColors: true,
      depthTest: false,
      transparent: true,
      opacity: 0.9,
    });
    this.object = new THREE.LineSegments(this.geometry, material);
    this.object.renderOrder = 999;
    this.object.frustumCulled = false;
  }

  begin(): void {
    this.count = 0;
  }

  private ensure(vertices: number): void {
    if (this.positions.length >= vertices * 3) return;
    const size = Math.max(vertices * 3, this.positions.length * 2, 3000);
    const p = new Float32Array(size);
    const c = new Float32Array(size);
    p.set(this.positions);
    c.set(this.colors);
    this.positions = p;
    this.colors = c;
    this.geometry.setAttribute('position', new THREE.BufferAttribute(this.positions, 3));
    this.geometry.setAttribute('color', new THREE.BufferAttribute(this.colors, 3));
  }

  drawLine(from: Readonly<Vec3>, to: Readonly<Vec3>, color: number): void {
    this.ensure(this.count + 2);
    this.color.setHex(color);
    let i = this.count * 3;
    this.positions[i] = from.x;
    this.positions[i + 1] = from.y;
    this.positions[i + 2] = from.z;
    this.colors[i] = this.color.r;
    this.colors[i + 1] = this.color.g;
    this.colors[i + 2] = this.color.b;
    i += 3;
    this.positions[i] = to.x;
    this.positions[i + 1] = to.y;
    this.positions[i + 2] = to.z;
    this.colors[i] = this.color.r;
    this.colors[i + 1] = this.color.g;
    this.colors[i + 2] = this.color.b;
    this.count += 2;
  }

  drawPoint(point: Readonly<Vec3>, size: number, color: number): void {
    // 用一个小十字表示点
    const s = size * 0.01;
    const a = { x: point.x - s, y: point.y, z: point.z } as Vec3;
    const b = { x: point.x + s, y: point.y, z: point.z } as Vec3;
    this.drawLine(a, b, color);
    a.x = point.x;
    b.x = point.x;
    a.y = point.y - s;
    b.y = point.y + s;
    this.drawLine(a, b, color);
    a.y = point.y;
    b.y = point.y;
    a.z = point.z - s;
    b.z = point.z + s;
    this.drawLine(a, b, color);
  }

  end(): void {
    const pos = this.geometry.getAttribute('position') as THREE.BufferAttribute | undefined;
    const col = this.geometry.getAttribute('color') as THREE.BufferAttribute | undefined;
    if (pos) pos.needsUpdate = true;
    if (col) col.needsUpdate = true;
    this.geometry.setDrawRange(0, this.count);
  }
}
