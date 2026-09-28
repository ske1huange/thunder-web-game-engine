import {
  BoxShape,
  CapsuleShape,
  type ColliderOptions,
  Quat,
  type RigidBody,
  type RigidBodyOptions,
  SphereShape,
  Vec3,
  type World,
} from '@thunder/physics';

type Extra = Omit<ColliderOptions, 'shape'> & { body?: Omit<RigidBodyOptions, 'position'> };

export function v(x: number, y: number, z: number): Vec3 {
  return new Vec3(x, y, z);
}

export function q(x: number, y: number, z: number): Quat {
  return new Quat().setFromEuler(x, y, z);
}

export function addBox(world: World, position: Vec3, half: Vec3, extra: Extra = {}): RigidBody {
  const { body, ...collider } = extra;
  const b = world.createBody({ ...body, position });
  b.addCollider({ shape: new BoxShape(half), ...collider });
  return b;
}

export function addSphere(world: World, position: Vec3, radius: number, extra: Extra = {}): RigidBody {
  const { body, ...collider } = extra;
  const b = world.createBody({ ...body, position });
  b.addCollider({ shape: new SphereShape(radius), ...collider });
  return b;
}

export function addCapsule(
  world: World,
  position: Vec3,
  radius: number,
  halfHeight: number,
  extra: Extra = {},
): RigidBody {
  const { body, ...collider } = extra;
  const b = world.createBody({ ...body, position });
  b.addCollider({ shape: new CapsuleShape(radius, halfHeight), ...collider });
  return b;
}

/** 可复现的伪随机数 */
export function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}
