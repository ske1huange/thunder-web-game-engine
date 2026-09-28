import {
  type BoxShape,
  type CapsuleShape,
  type Collider,
  type CylinderShape,
  type HeightfieldShape,
  type RigidBody,
  Quat,
  ShapeType,
  type SphereShape,
  type TriMeshShape,
  Vec3,
  type World,
} from '@thunder/physics';
import * as THREE from 'three';

/** 刚体外观（材质参数），未指定的项使用默认值 */
export interface BodyAppearance {
  /** 基础颜色 */
  color: THREE.ColorRepresentation;
  /** 自发光颜色（配合点光源可做“发光球”） */
  emissive: THREE.ColorRepresentation;
  emissiveIntensity: number;
  roughness: number;
  metalness: number;
  /** 不透明度，小于 1 时开启透明 */
  opacity: number;
  /**
   * 是否投射阴影（默认 true）。绑定了投射阴影光源的刚体应设为 false，
   * 否则光源位于网格内部，会被自身完全遮挡。
   */
  castShadow: boolean;
  /**
   * 是否显示（默认 true）。关卡模型由游戏自己渲染、只用来生成碰撞体时设为 false，避免画两遍。
   */
  visible: boolean;
}

export interface PhysicsViewOptions {
  /** 动态刚体的调色板，按刚体编号循环取色 */
  palette?: readonly THREE.ColorRepresentation[];
  staticColor?: THREE.ColorRepresentation;
  kinematicColor?: THREE.ColorRepresentation;
  sensorColor?: THREE.ColorRepresentation;
  /** 休眠刚体的亮度系数（1 表示不区分），默认 0.6 */
  sleepingBrightness?: number;
  /** 网格是否投射 / 接收阴影，默认 true */
  shadows?: boolean;
  /** 平面形状渲染成的地面尺寸，默认 200 */
  planeSize?: number;
}

const DEFAULT_PALETTE = [
  0xf2a541, 0x4f9dde, 0xe4572e, 0x76b041, 0xa06cd5, 0x2ec4b6, 0xff8fab, 0xffd23f,
];

/**
 * 把物理世界同步到 three.js 场景：为每个刚体按碰撞形状创建网格，
 * 每帧按插值系数写入位置与朝向，刚体被销毁时自动移除网格。
 */
export class PhysicsView {
  readonly root = new THREE.Group();
  private readonly objects = new Map<RigidBody, THREE.Object3D>();
  private readonly materials = new Map<string, THREE.MeshStandardMaterial>();
  private readonly appearances = new Map<RigidBody, Partial<BodyAppearance>>();
  private readonly options: Required<PhysicsViewOptions>;
  private readonly tmpP = new Vec3();
  private readonly tmpQ = new Quat();
  private readonly tmpColor = new THREE.Color();

  constructor(
    private readonly world: World,
    options: PhysicsViewOptions = {},
  ) {
    this.root.name = 'PhysicsView';
    this.options = {
      palette: options.palette ?? DEFAULT_PALETTE,
      staticColor: options.staticColor ?? 0x6b7280,
      kinematicColor: options.kinematicColor ?? 0x3b82f6,
      sensorColor: options.sensorColor ?? 0x4dcc66,
      sleepingBrightness: options.sleepingBrightness ?? 0.6,
      shadows: options.shadows ?? true,
      planeSize: options.planeSize ?? 200,
    };
  }

  /** 设置某个刚体的外观（与已有设置合并） */
  setAppearance(body: RigidBody, appearance: Partial<BodyAppearance>): void {
    this.appearances.set(body, { ...this.appearances.get(body), ...appearance });
  }

  /** 恢复默认外观 */
  clearAppearance(body: RigidBody): void {
    this.appearances.delete(body);
  }

  /** 刚体对应的 three.js 对象（尚未同步过时为 undefined） */
  getObject(body: RigidBody): THREE.Object3D | undefined {
    return this.objects.get(body);
  }

  /** 由网格（或其子对象）反查刚体 */
  bodyOf(object: THREE.Object3D | null): RigidBody | null {
    let o: THREE.Object3D | null = object;
    while (o) {
      if (o.userData.body) return o.userData.body as RigidBody;
      o = o.parent;
    }
    return null;
  }

  /** 同步场景：新增/删除刚体的网格，写入插值位姿与材质 */
  sync(alpha = 1): void {
    const alive = new Set<RigidBody>();
    for (const body of this.world.bodies) {
      alive.add(body);
      let obj = this.objects.get(body);
      if (!obj) {
        obj = this.createObject(body);
        this.objects.set(body, obj);
        this.root.add(obj);
      }
      const appearance = this.appearances.get(body);
      obj.visible = appearance?.visible ?? true;
      if (!obj.visible) continue;
      body.interpolate(alpha, this.tmpP, this.tmpQ);
      obj.position.set(this.tmpP.x, this.tmpP.y, this.tmpP.z);
      obj.quaternion.set(this.tmpQ.x, this.tmpQ.y, this.tmpQ.z, this.tmpQ.w);
      const sleeping = body.isDynamic() && !body.isAwake;
      const castShadow = appearance?.castShadow ?? true;
      for (const child of obj.children) {
        const mesh = child as THREE.Mesh;
        const collider = mesh.userData.collider as Collider;
        mesh.material = this.material(body, collider, sleeping);
        mesh.castShadow = castShadow && (mesh.userData.castsShadow as boolean);
      }
    }
    for (const [body, obj] of this.objects) {
      if (!alive.has(body)) {
        this.disposeObject(obj);
        this.objects.delete(body);
        this.appearances.delete(body);
      }
    }
  }

  /** 移除全部网格并释放几何体与材质 */
  clear(): void {
    for (const obj of this.objects.values()) this.disposeObject(obj);
    this.objects.clear();
    this.appearances.clear();
    for (const m of this.materials.values()) m.dispose();
    this.materials.clear();
  }

  private baseColor(body: RigidBody, collider: Collider): THREE.ColorRepresentation {
    const custom = this.appearances.get(body)?.color;
    if (custom !== undefined) return custom;
    if (collider.isSensor) return this.options.sensorColor;
    if (body.isStatic()) return this.options.staticColor;
    if (body.isKinematic()) return this.options.kinematicColor;
    const palette = this.options.palette;
    return palette[body.id % palette.length]!;
  }

  /** 按外观参数缓存材质，外观相同的刚体共享材质 */
  private material(body: RigidBody, collider: Collider, sleeping: boolean): THREE.Material {
    const a = this.appearances.get(body);
    const color = this.tmpColor.set(this.baseColor(body, collider));
    if (sleeping) color.multiplyScalar(this.options.sleepingBrightness);
    const opacity = collider.isSensor ? (a?.opacity ?? 0.25) : (a?.opacity ?? 1);
    const emissive = a?.emissive ?? 0x000000;
    const emissiveIntensity = a?.emissiveIntensity ?? 1;
    const roughness = a?.roughness ?? 0.65;
    const metalness = a?.metalness ?? 0.05;
    // 双面三角网格两面都要可见
    const shape = collider.shape;
    const doubleSided = shape.type === ShapeType.TriMesh && (shape as TriMeshShape).doubleSided;
    const key = `${color.getHexString()}|${String(emissive)}|${emissiveIntensity}|${roughness}|${metalness}|${opacity}|${doubleSided}`;
    let m = this.materials.get(key);
    if (!m) {
      const transparent = opacity < 1;
      m = new THREE.MeshStandardMaterial({
        color: color.clone(),
        emissive: new THREE.Color(emissive),
        emissiveIntensity,
        roughness,
        metalness,
        transparent,
        opacity,
        depthWrite: !transparent,
        side: doubleSided ? THREE.DoubleSide : THREE.FrontSide,
      });
      this.materials.set(key, m);
    }
    return m;
  }

  private createGeometry(collider: Collider): THREE.BufferGeometry | null {
    const shape = collider.shape;
    switch (shape.type) {
      case ShapeType.Sphere:
        return new THREE.SphereGeometry((shape as SphereShape).radius, 24, 16);
      case ShapeType.Box: {
        const h = (shape as BoxShape).halfExtents;
        return new THREE.BoxGeometry(h.x * 2, h.y * 2, h.z * 2);
      }
      case ShapeType.Capsule: {
        const c = shape as CapsuleShape;
        return new THREE.CapsuleGeometry(c.radius, c.halfHeight * 2, 6, 16);
      }
      case ShapeType.Cylinder: {
        const c = shape as CylinderShape;
        return new THREE.CylinderGeometry(
          c.cylinderRadius,
          c.cylinderRadius,
          c.halfHeight * 2,
          c.segments,
        );
      }
      case ShapeType.ConvexHull: {
        const hull = shape.hull!;
        const positions: number[] = [];
        for (const face of hull.faces) {
          const v0 = hull.vertices[face.indices[0]!]!;
          for (let i = 1; i + 1 < face.indices.length; i++) {
            const v1 = hull.vertices[face.indices[i]!]!;
            const v2 = hull.vertices[face.indices[i + 1]!]!;
            positions.push(v0.x, v0.y, v0.z, v1.x, v1.y, v1.z, v2.x, v2.y, v2.z);
          }
        }
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        g.computeVertexNormals();
        return g;
      }
      case ShapeType.Plane: {
        const size = this.options.planeSize;
        const g = new THREE.PlaneGeometry(size, size);
        g.rotateX(-Math.PI / 2);
        return g;
      }
      case ShapeType.TriMesh: {
        // 关卡网格一般有硬边：展开为非索引几何体得到平直着色
        const mesh = shape as TriMeshShape;
        const g = new THREE.BufferGeometry();
        g.setAttribute('position', new THREE.BufferAttribute(Float32Array.from(mesh.positions), 3));
        g.setIndex(new THREE.BufferAttribute(Uint32Array.from(mesh.indices), 1));
        const flat = g.toNonIndexed();
        g.dispose();
        flat.computeVertexNormals();
        return flat;
      }
      case ShapeType.Heightfield:
        return heightfieldGeometry(shape as HeightfieldShape);
      default:
        return null;
    }
  }

  private createObject(body: RigidBody): THREE.Object3D {
    const group = new THREE.Group();
    for (const collider of body.colliders) {
      const geometry = this.createGeometry(collider);
      if (!geometry) continue;
      const mesh = new THREE.Mesh(geometry, this.material(body, collider, false));
      const lt = collider.localTransform;
      mesh.position.set(lt.position.x, lt.position.y, lt.position.z);
      mesh.quaternion.set(lt.rotation.x, lt.rotation.y, lt.rotation.z, lt.rotation.w);
      const solid = !collider.isSensor;
      mesh.userData.castsShadow =
        this.options.shadows && solid && collider.shape.type !== ShapeType.Plane;
      mesh.castShadow = mesh.userData.castsShadow as boolean;
      mesh.receiveShadow = this.options.shadows && solid;
      mesh.userData.collider = collider;
      group.add(mesh);
    }
    group.userData.body = body;
    return group;
  }

  private disposeObject(obj: THREE.Object3D): void {
    this.root.remove(obj);
    obj.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (mesh.geometry) mesh.geometry.dispose();
    });
  }
}

/** 高度场网格：与碰撞三角形相同的对角线划分，平滑法线 */
function heightfieldGeometry(field: HeightfieldShape): THREE.BufferGeometry {
  const { rows, cols } = field;
  const positions = new Float32Array(rows * cols * 3);
  const p = new Vec3();
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      field.getVertex(r, c, p);
      positions.set([p.x, p.y, p.z], (r * cols + c) * 3);
    }
  }
  const indices = new Uint32Array((rows - 1) * (cols - 1) * 6);
  let k = 0;
  for (let r = 0; r < rows - 1; r++) {
    for (let c = 0; c < cols - 1; c++) {
      const i = r * cols + c;
      // 与 HeightfieldShape.getTriangle 一致：(c,r)(c,r+1)(c+1,r+1) 与 (c,r)(c+1,r+1)(c+1,r)
      indices.set([i, i + cols, i + cols + 1, i, i + cols + 1, i + 1], k);
      k += 6;
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  g.setIndex(new THREE.BufferAttribute(indices, 1));
  g.computeVertexNormals();
  return g;
}
