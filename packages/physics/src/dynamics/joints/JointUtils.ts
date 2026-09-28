import { Mat3 } from '../../math/Mat3';
import { Quat } from '../../math/Quat';
import { Vec3 } from '../../math/Vec3';
import type { RigidBody } from '../RigidBody';

/**
 * 关节求解的公共工具。所有函数都基于子步内的“当前”状态：
 * 当前旋转 = deltaRotation * rotation0，当前质心 = center0 + deltaPosition。
 */

const tq = new Quat();
const tv = new Vec3();
const skew = new Mat3();
const tm = new Mat3();

/** 当前旋转 */
export function currentRotation(body: RigidBody, out: Quat): Quat {
  return out.multiplyQuats(body.deltaRotation, body.rotation0);
}

/** 当前质心 */
export function currentCenter(body: RigidBody, out: Vec3): Vec3 {
  return out.addVectors(body.center0, body.deltaPosition);
}

/** 局部锚点（相对刚体原点）→ 当前世界朝向下相对质心的向量 */
export function currentArm(body: RigidBody, localAnchor: Readonly<Vec3>, out: Vec3): Vec3 {
  currentRotation(body, tq);
  out.subVectors(localAnchor, body.localCenter);
  return tq.rotate(out, out);
}

/** 局部方向 → 当前世界方向 */
export function currentDirection(body: RigidBody, localDir: Readonly<Vec3>, out: Vec3): Vec3 {
  currentRotation(body, tq);
  return tq.rotate(localDir, out);
}

/** 锚点处相对速度 vB + wB×rB - vA - wA×rA */
export function pointVelocity(
  bodyA: RigidBody,
  bodyB: RigidBody,
  rA: Readonly<Vec3>,
  rB: Readonly<Vec3>,
  out: Vec3,
): Vec3 {
  out.crossVectors(bodyB.angularVelocity, rB).add(bodyB.linearVelocity);
  tv.crossVectors(bodyA.angularVelocity, rA).add(bodyA.linearVelocity);
  return out.sub(tv);
}

/** 点约束的 3x3 有效质量矩阵 K = (mA+mB)I - [rA]× IA⁻¹ [rA]× - [rB]× IB⁻¹ [rB]× */
export function pointMassMatrix(
  bodyA: RigidBody,
  bodyB: RigidBody,
  rA: Readonly<Vec3>,
  rB: Readonly<Vec3>,
  out: Mat3,
): Mat3 {
  const m = bodyA.invMass + bodyB.invMass;
  out.setDiagonal(m, m, m);
  if (bodyA.invMass > 0) {
    skew.setSkew(rA);
    tm.multiplyMatrices(skew, bodyA.invInertiaWorld).multiplyMatrices(tm, skew);
    out.add(tm.scale(-1));
  }
  if (bodyB.invMass > 0) {
    skew.setSkew(rB);
    tm.multiplyMatrices(skew, bodyB.invInertiaWorld).multiplyMatrices(tm, skew);
    out.add(tm.scale(-1));
  }
  return out;
}

/** 施加作用于锚点的线冲量 P（B 正、A 负） */
export function applyPointImpulse(
  bodyA: RigidBody,
  bodyB: RigidBody,
  rA: Readonly<Vec3>,
  rB: Readonly<Vec3>,
  P: Readonly<Vec3>,
): void {
  if (bodyA.invMass > 0) {
    bodyA.linearVelocity.addScaled(P, -bodyA.invMass);
    tv.crossVectors(rA, P);
    bodyA.invInertiaWorld.transformVector(tv, tv);
    bodyA.angularVelocity.sub(tv);
  }
  if (bodyB.invMass > 0) {
    bodyB.linearVelocity.addScaled(P, bodyB.invMass);
    tv.crossVectors(rB, P);
    bodyB.invInertiaWorld.transformVector(tv, tv);
    bodyB.angularVelocity.add(tv);
  }
}

/** 施加角冲量 L（B 正、A 负） */
export function applyAngularImpulse(bodyA: RigidBody, bodyB: RigidBody, L: Readonly<Vec3>): void {
  if (bodyA.invMass > 0) {
    bodyA.invInertiaWorld.transformVector(L, tv);
    bodyA.angularVelocity.sub(tv);
  }
  if (bodyB.invMass > 0) {
    bodyB.invInertiaWorld.transformVector(L, tv);
    bodyB.angularVelocity.add(tv);
  }
}

/** 角约束的有效质量矩阵 K = IA⁻¹ + IB⁻¹ */
export function angularMassMatrix(bodyA: RigidBody, bodyB: RigidBody, out: Mat3): Mat3 {
  return out.copy(bodyA.invInertiaWorld).add(bodyB.invInertiaWorld);
}

/**
 * 一维约束行：Cdot = n·(vB - vA) + jA·wA + jB·wB。
 * n 可为 null（纯角约束）。
 */
export function rowVelocity(
  bodyA: RigidBody,
  bodyB: RigidBody,
  n: Readonly<Vec3> | null,
  jA: Readonly<Vec3>,
  jB: Readonly<Vec3>,
): number {
  let c = jA.dot(bodyA.angularVelocity) + jB.dot(bodyB.angularVelocity);
  if (n) c += n.dot(bodyB.linearVelocity) - n.dot(bodyA.linearVelocity);
  return c;
}

/** 一维约束行的有效质量（返回 1/K，K 为 0 时返回 0） */
export function rowMass(
  bodyA: RigidBody,
  bodyB: RigidBody,
  n: Readonly<Vec3> | null,
  jA: Readonly<Vec3>,
  jB: Readonly<Vec3>,
): number {
  let k = bodyA.invInertiaWorld.quadraticForm(jA) + bodyB.invInertiaWorld.quadraticForm(jB);
  if (n) k += (bodyA.invMass + bodyB.invMass) * n.lengthSq();
  return k > 0 ? 1 / k : 0;
}

/** 施加一维约束行的冲量 λ */
export function rowApply(
  bodyA: RigidBody,
  bodyB: RigidBody,
  n: Readonly<Vec3> | null,
  jA: Readonly<Vec3>,
  jB: Readonly<Vec3>,
  lambda: number,
): void {
  if (bodyA.invMass > 0) {
    if (n) bodyA.linearVelocity.addScaled(n, -bodyA.invMass * lambda);
    bodyA.invInertiaWorld.transformVector(jA, tv);
    bodyA.angularVelocity.addScaled(tv, lambda);
  }
  if (bodyB.invMass > 0) {
    if (n) bodyB.linearVelocity.addScaled(n, bodyB.invMass * lambda);
    bodyB.invInertiaWorld.transformVector(jB, tv);
    bodyB.angularVelocity.addScaled(tv, lambda);
  }
}

/**
 * 相对旋转误差（世界系旋转向量）：qB 相对于目标 qA * qRel 的偏差。
 */
export function rotationError(
  qA: Readonly<Quat>,
  qB: Readonly<Quat>,
  qRel: Readonly<Quat>,
  out: Vec3,
): Vec3 {
  // e = qB * (qA * qRel)⁻¹
  tq.multiplyQuats(qA, qRel).conjugate();
  tq.premultiply(qB);
  return tq.toRotationVector(out);
}

const TWO_PI = Math.PI * 2;
const twist = new Quat();

/**
 * 摆动-扭转分解：把误差旋转 e（世界系）分解为 e = swing · twist，twist 绕 axis。
 * 返回扭转角（[-π, π]），摆动部分的旋转向量（与 axis 垂直）写入 swingOut。
 */
export function swingTwist(e: Readonly<Quat>, axis: Readonly<Vec3>, swingOut: Vec3): number {
  const d = e.x * axis.x + e.y * axis.y + e.z * axis.z;
  let angle = 2 * Math.atan2(d, e.w);
  if (angle > Math.PI) angle -= TWO_PI;
  else if (angle < -Math.PI) angle += TWO_PI;
  twist.set(axis.x * d, axis.y * d, axis.z * d, e.w);
  if (twist.lengthSq() < 1e-18) twist.identity();
  else twist.normalize();
  // swing = e * twist⁻¹
  twist.conjugate();
  twist.premultiply(e);
  twist.toRotationVector(swingOut);
  return angle;
}
