import {
  type BoxShape,
  type CapsuleShape,
  type Collider,
  type CylinderShape,
  type RigidBody,
  ShapeType,
  type SphereShape,
  Quat,
  Vec3,
  type World,
} from '@thunder/physics';
import * as THREE from 'three';

const PALETTE = [0xf2a541, 0x4f9dde, 0xe4572e, 0x76b041, 0xa06cd5, 0x2ec4b6, 0xff8fab, 0xffd23f];
const STATIC_COLOR = 0x6b7280;
const KINEMATIC_COLOR = 0x3b82f6;

/**
 * 把物理世界同步到 three.js 场景：为每个刚体创建一组网格，
 * 每帧按插值系数写入位置与朝向。
 */
export class PhysicsView {
  readonly root = new THREE.Group();
  private readonly objects = new Map<RigidBody, THREE.Object3D>();
  private readonly materials = new Map<string, THREE.MeshStandardMaterial>();
  private readonly tmpP = new Vec3();
  private readonly tmpQ = new Quat();
  /** 自定义颜色（例如传感器 demo 中高亮） */
  readonly colorOverride = new Map<RigidBody, number>();

  constructor(private readonly world: World) {}

  private material(color: number, sleeping: boolean, transparent = false): THREE.MeshStandardMaterial {
    const key = `${color}-${sleeping}-${transparent}`;
    let m = this.materials.get(key);
    if (!m) {
      const c = new THREE.Color(color);
      if (sleeping) c.multiplyScalar(0.6);
      m = new THREE.MeshStandardMaterial({
        color: c,
        roughness: 0.65,
        metalness: 0.05,
        transparent,
        opacity: transparent ? 0.25 : 1,
        depthWrite: !transparent,
      });
      this.materials.set(key, m);
    }
    return m;
  }

  private baseColor(body: RigidBody): number {
    const override = this.colorOverride.get(body);
    if (override !== undefined) return override;
    if (body.isStatic()) return STATIC_COLOR;
    if (body.isKinematic()) return KINEMATIC_COLOR;
    return PALETTE[body.id % PALETTE.length]!;
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
        return new THREE.CylinderGeometry(c.cylinderRadius, c.cylinderRadius, c.halfHeight * 2, c.segments);
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
        const g = new THREE.PlaneGeometry(200, 200);
        g.rotateX(-Math.PI / 2);
        return g;
      }
      default:
        return null;
    }
  }

  private createObject(body: RigidBody): THREE.Object3D {
    const group = new THREE.Group();
    for (const collider of body.colliders) {
      const geometry = this.createGeometry(collider);
      if (!geometry) continue;
      const mesh = new THREE.Mesh(
        geometry,
        this.material(this.baseColor(body), false, collider.isSensor),
      );
      const lt = collider.localTransform;
      mesh.position.set(lt.position.x, lt.position.y, lt.position.z);
      mesh.quaternion.set(lt.rotation.x, lt.rotation.y, lt.rotation.z, lt.rotation.w);
      mesh.castShadow = !collider.isSensor && collider.shape.type !== ShapeType.Plane;
      mesh.receiveShadow = !collider.isSensor;
      mesh.userData.collider = collider;
      group.add(mesh);
    }
    group.userData.body = body;
    return group;
  }

  /** 同步场景：新增/删除刚体的网格，写入插值位姿 */
  sync(alpha: number): void {
    const alive = new Set<RigidBody>();
    for (const body of this.world.bodies) {
      alive.add(body);
      let obj = this.objects.get(body);
      if (!obj) {
        obj = this.createObject(body);
        this.objects.set(body, obj);
        this.root.add(obj);
      }
      body.interpolate(alpha, this.tmpP, this.tmpQ);
      obj.position.set(this.tmpP.x, this.tmpP.y, this.tmpP.z);
      obj.quaternion.set(this.tmpQ.x, this.tmpQ.y, this.tmpQ.z, this.tmpQ.w);
      const sleeping = body.isDynamic() && !body.isAwake;
      const color = this.baseColor(body);
      for (const child of obj.children) {
        const mesh = child as THREE.Mesh;
        const collider = mesh.userData.collider as Collider;
        mesh.material = this.material(color, sleeping, collider.isSensor);
      }
    }
    for (const [body, obj] of this.objects) {
      if (!alive.has(body)) {
        this.root.remove(obj);
        obj.traverse((o) => {
          if ((o as THREE.Mesh).geometry) (o as THREE.Mesh).geometry.dispose();
        });
        this.objects.delete(body);
      }
    }
  }

  /** 由网格反查刚体 */
  bodyOf(object: THREE.Object3D | null): RigidBody | null {
    let o: THREE.Object3D | null = object;
    while (o) {
      if (o.userData.body) return o.userData.body as RigidBody;
      o = o.parent;
    }
    return null;
  }

  clear(): void {
    for (const obj of this.objects.values()) {
      this.root.remove(obj);
      obj.traverse((o) => {
        if ((o as THREE.Mesh).geometry) (o as THREE.Mesh).geometry.dispose();
      });
    }
    this.objects.clear();
    this.colorOverride.clear();
  }
}
