import { computeBasis, Vec3 } from '../../math/Vec3';
import type { Contact } from '../Contact';
import type { RigidBody } from '../RigidBody';
import type { SolverContext } from './SolverContext';

/**
 * 接触约束求解（Box2D v3 “Soft Step” 的三维版本）。
 *
 * - 锚点在准备阶段固定（相对质心，世界朝向），子步内用刚体的位移/相对旋转
 *   计算当前分离距离：s = dot(dpB - dpA + qB·rB - qA·rA, n) + (s0 - dot(rB - rA, n))
 * - s > 0 时为推测接触：允许以 s/h 的速度接近；s ≤ 0 时用软约束把穿透推出
 * - relax 阶段不加偏置，去掉推出带来的多余速度
 * - 弹性在所有子步结束后单独处理
 */

const rA = new Vec3();
const rB = new Vec3();
const tmp = new Vec3();
const tmp2 = new Vec3();
const dv = new Vec3();
const P = new Vec3();
const prA = new Vec3();
const prB = new Vec3();

function effectiveMass(bodyA: RigidBody, bodyB: RigidBody, ra: Vec3, rb: Vec3, axis: Vec3): number {
  // k = mA + mB + (rA×n)·IA⁻¹(rA×n) + (rB×n)·IB⁻¹(rB×n)
  let k = bodyA.invMass + bodyB.invMass;
  tmp.crossVectors(ra, axis);
  k += bodyA.invInertiaWorld.quadraticForm(tmp);
  tmp.crossVectors(rb, axis);
  k += bodyB.invInertiaWorld.quadraticForm(tmp);
  return k > 0 ? 1 / k : 0;
}

/** 相对速度 dv = (vB + wB×rB) - (vA + wA×rA) */
function relativeVelocity(bodyA: RigidBody, bodyB: RigidBody, ra: Vec3, rb: Vec3, out: Vec3): Vec3 {
  out.crossVectors(bodyB.angularVelocity, rb).add(bodyB.linearVelocity);
  tmp.crossVectors(bodyA.angularVelocity, ra).add(bodyA.linearVelocity);
  return out.sub(tmp);
}

/** 施加冲量 P（作用于 B，反作用于 A） */
function applyImpulse(bodyA: RigidBody, bodyB: RigidBody, ra: Vec3, rb: Vec3, impulse: Vec3): void {
  if (bodyA.invMass > 0) {
    bodyA.linearVelocity.addScaled(impulse, -bodyA.invMass);
    tmp.crossVectors(ra, impulse);
    bodyA.invInertiaWorld.transformVector(tmp, tmp);
    bodyA.angularVelocity.sub(tmp);
  }
  if (bodyB.invMass > 0) {
    bodyB.linearVelocity.addScaled(impulse, bodyB.invMass);
    tmp.crossVectors(rb, impulse);
    bodyB.invInertiaWorld.transformVector(tmp, tmp);
    bodyB.angularVelocity.add(tmp);
  }
}

export function prepareContacts(contacts: readonly Contact[], ctx: SolverContext): void {
  for (const c of contacts) {
    const bodyA = c.bodyA;
    const bodyB = c.bodyB;
    const m = c.manifold;
    const n = m.normal;
    computeBasis(n, m.tangent1, m.tangent2);
    c.softness.biasRate = 0;
    const soft =
      bodyA.invMass === 0 || bodyB.invMass === 0 ? ctx.staticSoftness : ctx.contactSoftness;
    c.softness.biasRate = soft.biasRate;
    c.softness.massScale = soft.massScale;
    c.softness.impulseScale = soft.impulseScale;

    for (let i = 0; i < m.pointCount; i++) {
      const p = m.points[i]!;
      p.anchorA.subVectors(p.point, bodyA.center);
      p.anchorB.subVectors(p.point, bodyB.center);
      p.baseSeparation = p.separation - tmp2.subVectors(p.anchorB, p.anchorA).dot(n);
      p.normalMass = effectiveMass(bodyA, bodyB, p.anchorA, p.anchorB, n);
      p.tangentMass1 = effectiveMass(bodyA, bodyB, p.anchorA, p.anchorB, m.tangent1);
      p.tangentMass2 = effectiveMass(bodyA, bodyB, p.anchorA, p.anchorB, m.tangent2);
      p.relativeVelocity = relativeVelocity(bodyA, bodyB, p.anchorA, p.anchorB, dv).dot(n);
      p.maxNormalImpulse = 0;
      if (!ctx.enableWarmStarting) {
        p.normalImpulse = 0;
        p.tangentImpulse1 = 0;
        p.tangentImpulse2 = 0;
      }
    }
  }
}

export function warmStartContacts(contacts: readonly Contact[]): void {
  for (const c of contacts) {
    const bodyA = c.bodyA;
    const bodyB = c.bodyB;
    const m = c.manifold;
    for (let i = 0; i < m.pointCount; i++) {
      const p = m.points[i]!;
      P.copy(m.normal).scale(p.normalImpulse);
      P.addScaled(m.tangent1, p.tangentImpulse1);
      P.addScaled(m.tangent2, p.tangentImpulse2);
      applyImpulse(bodyA, bodyB, p.anchorA, p.anchorB, P);
    }
  }
}

export function solveContacts(
  contacts: readonly Contact[],
  ctx: SolverContext,
  useBias: boolean,
): void {
  const invH = ctx.invH;
  const maxBiasVelocity = ctx.maxBiasVelocity;
  for (const c of contacts) {
    const bodyA = c.bodyA;
    const bodyB = c.bodyB;
    const m = c.manifold;
    const n = m.normal;
    const soft = c.softness;

    // ---- 法向（非穿透） ----
    for (let i = 0; i < m.pointCount; i++) {
      const p = m.points[i]!;
      rA.copy(p.anchorA);
      rB.copy(p.anchorB);
      // 当前分离距离
      bodyA.deltaRotation.rotate(rA, prA);
      bodyB.deltaRotation.rotate(rB, prB);
      tmp2.subVectors(bodyB.deltaPosition, bodyA.deltaPosition).add(prB).sub(prA);
      const s = tmp2.dot(n) + p.baseSeparation;

      let velocityBias = 0;
      let massScale = 1;
      let impulseScale = 0;
      if (s > 0) {
        // 推测接触：允许在本子步内恰好闭合间隙
        velocityBias = s * invH;
      } else if (useBias) {
        velocityBias = Math.max(soft.biasRate * s, -maxBiasVelocity);
        massScale = soft.massScale;
        impulseScale = soft.impulseScale;
      }

      const vn = relativeVelocity(bodyA, bodyB, rA, rB, dv).dot(n);
      let impulse = -p.normalMass * massScale * (vn + velocityBias) - impulseScale * p.normalImpulse;
      const newImpulse = Math.max(p.normalImpulse + impulse, 0);
      impulse = newImpulse - p.normalImpulse;
      p.normalImpulse = newImpulse;
      if (impulse > p.maxNormalImpulse) p.maxNormalImpulse = impulse;
      P.copy(n).scale(impulse);
      applyImpulse(bodyA, bodyB, rA, rB, P);
    }

    // ---- 摩擦（二维库仑锥，按圆盘截断） ----
    const friction = c.friction;
    if (friction > 0) {
      for (let i = 0; i < m.pointCount; i++) {
        const p = m.points[i]!;
        rA.copy(p.anchorA);
        rB.copy(p.anchorB);
        relativeVelocity(bodyA, bodyB, rA, rB, dv);
        const vt1 = dv.dot(m.tangent1);
        const vt2 = dv.dot(m.tangent2);
        let new1 = p.tangentImpulse1 - p.tangentMass1 * vt1;
        let new2 = p.tangentImpulse2 - p.tangentMass2 * vt2;
        const maxFriction = friction * p.normalImpulse;
        const lenSq = new1 * new1 + new2 * new2;
        if (lenSq > maxFriction * maxFriction) {
          const scale = lenSq > 0 ? maxFriction / Math.sqrt(lenSq) : 0;
          new1 *= scale;
          new2 *= scale;
        }
        const d1 = new1 - p.tangentImpulse1;
        const d2 = new2 - p.tangentImpulse2;
        p.tangentImpulse1 = new1;
        p.tangentImpulse2 = new2;
        P.copy(m.tangent1).scale(d1).addScaled(m.tangent2, d2);
        applyImpulse(bodyA, bodyB, rA, rB, P);
      }
    }
  }
}

export function applyRestitution(contacts: readonly Contact[], ctx: SolverContext): void {
  const threshold = ctx.restitutionThreshold;
  for (const c of contacts) {
    const e = c.restitution;
    if (e === 0) continue;
    const bodyA = c.bodyA;
    const bodyB = c.bodyB;
    const m = c.manifold;
    const n = m.normal;
    for (let i = 0; i < m.pointCount; i++) {
      const p = m.points[i]!;
      // 只对足够快的接近速度且确实产生过冲量的点处理
      if (p.relativeVelocity > -threshold || p.maxNormalImpulse === 0) continue;
      rA.copy(p.anchorA);
      rB.copy(p.anchorB);
      const vn = relativeVelocity(bodyA, bodyB, rA, rB, dv).dot(n);
      let impulse = -p.normalMass * (vn + e * p.relativeVelocity);
      const newImpulse = Math.max(p.normalImpulse + impulse, 0);
      impulse = newImpulse - p.normalImpulse;
      p.normalImpulse = newImpulse;
      if (impulse > p.maxNormalImpulse) p.maxNormalImpulse = impulse;
      P.copy(n).scale(impulse);
      applyImpulse(bodyA, bodyB, rA, rB, P);
    }
  }
}
