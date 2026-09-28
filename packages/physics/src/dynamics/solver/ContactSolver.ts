import type { Mat3 } from '../../math/Mat3';
import { computeBasis } from '../../math/Vec3';
import type { Contact } from '../Contact';
import type { SolverContext } from './SolverContext';

/**
 * 接触约束求解（Box2D v3 “Soft Step” 的三维版本）。
 *
 * - 锚点在准备阶段固定（相对质心、世界朝向），因此一步内雅可比为常量，可预先算好
 *   rA×n、IA⁻¹(rA×n) 等，求解时只剩标量运算
 * - 子步内用刚体的位移/相对旋转计算当前分离距离：
 *   s = dot(dpB - dpA + qB·rB - qA·rA, n) + (s0 - dot(rB - rA, n))
 * - s > 0 为推测接触：允许以 s/h 的速度接近；s ≤ 0 时用软约束推出
 * - relax 阶段不加偏置，去掉推出带来的多余速度；弹性在所有子步之后单独处理
 */

// jacobian 数组中各分量的偏移
const N_RA = 0; // rA × n
const N_RB = 3; // rB × n
const N_IA = 6; // IA⁻¹ (rA × n)
const N_IB = 9; // IB⁻¹ (rB × n)
const T1 = 12;
const T2 = 24;

function writeAxis(
  j: Float64Array,
  offset: number,
  rax: number,
  ray: number,
  raz: number,
  rbx: number,
  rby: number,
  rbz: number,
  nx: number,
  ny: number,
  nz: number,
  IA: Mat3,
  IB: Mat3,
  mA: number,
  mB: number,
): number {
  const cax = ray * nz - raz * ny;
  const cay = raz * nx - rax * nz;
  const caz = rax * ny - ray * nx;
  const cbx = rby * nz - rbz * ny;
  const cby = rbz * nx - rbx * nz;
  const cbz = rbx * ny - rby * nx;
  const iax = IA.m00 * cax + IA.m01 * cay + IA.m02 * caz;
  const iay = IA.m10 * cax + IA.m11 * cay + IA.m12 * caz;
  const iaz = IA.m20 * cax + IA.m21 * cay + IA.m22 * caz;
  const ibx = IB.m00 * cbx + IB.m01 * cby + IB.m02 * cbz;
  const iby = IB.m10 * cbx + IB.m11 * cby + IB.m12 * cbz;
  const ibz = IB.m20 * cbx + IB.m21 * cby + IB.m22 * cbz;
  j[offset + N_RA] = cax;
  j[offset + N_RA + 1] = cay;
  j[offset + N_RA + 2] = caz;
  j[offset + N_RB] = cbx;
  j[offset + N_RB + 1] = cby;
  j[offset + N_RB + 2] = cbz;
  j[offset + N_IA] = iax;
  j[offset + N_IA + 1] = iay;
  j[offset + N_IA + 2] = iaz;
  j[offset + N_IB] = ibx;
  j[offset + N_IB + 1] = iby;
  j[offset + N_IB + 2] = ibz;
  const k = mA + mB + cax * iax + cay * iay + caz * iaz + cbx * ibx + cby * iby + cbz * ibz;
  return k > 0 ? 1 / k : 0;
}

export function prepareContacts(contacts: readonly Contact[], ctx: SolverContext): void {
  for (const c of contacts) {
    const bodyA = c.bodyA;
    const bodyB = c.bodyB;
    const m = c.manifold;
    const n = m.normal;
    computeBasis(n, m.tangent1, m.tangent2);
    const soft =
      bodyA.invMass === 0 || bodyB.invMass === 0 ? ctx.staticSoftness : ctx.contactSoftness;
    c.softness.biasRate = soft.biasRate;
    c.softness.massScale = soft.massScale;
    c.softness.impulseScale = soft.impulseScale;
    const mA = bodyA.invMass;
    const mB = bodyB.invMass;
    const IA = bodyA.invInertiaWorld;
    const IB = bodyB.invInertiaWorld;
    const t1 = m.tangent1;
    const t2 = m.tangent2;
    const vA = bodyA.linearVelocity;
    const wA = bodyA.angularVelocity;
    const vB = bodyB.linearVelocity;
    const wB = bodyB.angularVelocity;

    for (let i = 0; i < m.pointCount; i++) {
      const p = m.points[i]!;
      const rA = p.anchorA.subVectors(p.point, bodyA.center);
      const rB = p.anchorB.subVectors(p.point, bodyB.center);
      p.baseSeparation =
        p.separation - ((rB.x - rA.x) * n.x + (rB.y - rA.y) * n.y + (rB.z - rA.z) * n.z);
      const j = p.jacobian;
      p.normalMass = writeAxis(j, 0, rA.x, rA.y, rA.z, rB.x, rB.y, rB.z, n.x, n.y, n.z, IA, IB, mA, mB);
      p.tangentMass1 = writeAxis(j, T1, rA.x, rA.y, rA.z, rB.x, rB.y, rB.z, t1.x, t1.y, t1.z, IA, IB, mA, mB);
      p.tangentMass2 = writeAxis(j, T2, rA.x, rA.y, rA.z, rB.x, rB.y, rB.z, t2.x, t2.y, t2.z, IA, IB, mA, mB);
      // 求解前的相对法向速度（用于弹性）
      p.relativeVelocity =
        n.x * (vB.x - vA.x) +
        n.y * (vB.y - vA.y) +
        n.z * (vB.z - vA.z) +
        j[N_RB]! * wB.x +
        j[N_RB + 1]! * wB.y +
        j[N_RB + 2]! * wB.z -
        j[N_RA]! * wA.x -
        j[N_RA + 1]! * wA.y -
        j[N_RA + 2]! * wA.z;
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
    const mA = bodyA.invMass;
    const mB = bodyB.invMass;
    const vA = bodyA.linearVelocity;
    const wA = bodyA.angularVelocity;
    const vB = bodyB.linearVelocity;
    const wB = bodyB.angularVelocity;
    const n = m.normal;
    const t1 = m.tangent1;
    const t2 = m.tangent2;
    for (let i = 0; i < m.pointCount; i++) {
      const p = m.points[i]!;
      const j = p.jacobian;
      const ln = p.normalImpulse;
      const l1 = p.tangentImpulse1;
      const l2 = p.tangentImpulse2;
      const px = n.x * ln + t1.x * l1 + t2.x * l2;
      const py = n.y * ln + t1.y * l1 + t2.y * l2;
      const pz = n.z * ln + t1.z * l1 + t2.z * l2;
      vA.x -= mA * px;
      vA.y -= mA * py;
      vA.z -= mA * pz;
      vB.x += mB * px;
      vB.y += mB * py;
      vB.z += mB * pz;
      wA.x -= j[N_IA]! * ln + j[T1 + N_IA]! * l1 + j[T2 + N_IA]! * l2;
      wA.y -= j[N_IA + 1]! * ln + j[T1 + N_IA + 1]! * l1 + j[T2 + N_IA + 1]! * l2;
      wA.z -= j[N_IA + 2]! * ln + j[T1 + N_IA + 2]! * l1 + j[T2 + N_IA + 2]! * l2;
      wB.x += j[N_IB]! * ln + j[T1 + N_IB]! * l1 + j[T2 + N_IB]! * l2;
      wB.y += j[N_IB + 1]! * ln + j[T1 + N_IB + 1]! * l1 + j[T2 + N_IB + 1]! * l2;
      wB.z += j[N_IB + 2]! * ln + j[T1 + N_IB + 2]! * l1 + j[T2 + N_IB + 2]! * l2;
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
    const count = m.pointCount;
    const mA = bodyA.invMass;
    const mB = bodyB.invMass;
    const soft = c.softness;

    // 把刚体状态读入局部变量
    const lvA = bodyA.linearVelocity;
    const avA = bodyA.angularVelocity;
    const lvB = bodyB.linearVelocity;
    const avB = bodyB.angularVelocity;
    let vAx = lvA.x,
      vAy = lvA.y,
      vAz = lvA.z;
    let wAx = avA.x,
      wAy = avA.y,
      wAz = avA.z;
    let vBx = lvB.x,
      vBy = lvB.y,
      vBz = lvB.z;
    let wBx = avB.x,
      wBy = avB.y,
      wBz = avB.z;

    const dpA = bodyA.deltaPosition;
    const dpB = bodyB.deltaPosition;
    const qA = bodyA.deltaRotation;
    const qB = bodyB.deltaRotation;
    const dpx = dpB.x - dpA.x,
      dpy = dpB.y - dpA.y,
      dpz = dpB.z - dpA.z;
    const n = m.normal;
    const nx = n.x,
      ny = n.y,
      nz = n.z;

    // ---- 法向（非穿透） ----
    // 求解与 relax 两个阶段以相反顺序遍历接触点，抵消 Gauss-Seidel 顺序带来的不对称
    const reverse = !useBias;
    for (let k = 0; k < count; k++) {
      const p = m.points[reverse ? count - 1 - k : k]!;
      const j = p.jacobian;
      const rA = p.anchorA;
      const rB = p.anchorB;
      // 当前锚点 qA·rA、qB·rB（内联四元数旋转）
      let tx = 2 * (qA.y * rA.z - qA.z * rA.y);
      let ty = 2 * (qA.z * rA.x - qA.x * rA.z);
      let tz = 2 * (qA.x * rA.y - qA.y * rA.x);
      const prAx = rA.x + qA.w * tx + (qA.y * tz - qA.z * ty);
      const prAy = rA.y + qA.w * ty + (qA.z * tx - qA.x * tz);
      const prAz = rA.z + qA.w * tz + (qA.x * ty - qA.y * tx);
      tx = 2 * (qB.y * rB.z - qB.z * rB.y);
      ty = 2 * (qB.z * rB.x - qB.x * rB.z);
      tz = 2 * (qB.x * rB.y - qB.y * rB.x);
      const prBx = rB.x + qB.w * tx + (qB.y * tz - qB.z * ty);
      const prBy = rB.y + qB.w * ty + (qB.z * tx - qB.x * tz);
      const prBz = rB.z + qB.w * tz + (qB.x * ty - qB.y * tx);
      const s =
        (dpx + prBx - prAx) * nx + (dpy + prBy - prAy) * ny + (dpz + prBz - prAz) * nz + p.baseSeparation;

      let velocityBias = 0;
      let massScale = 1;
      let impulseScale = 0;
      if (s > 0) {
        velocityBias = s * invH; // 推测接触
      } else if (useBias) {
        velocityBias = Math.max(soft.biasRate * s, -maxBiasVelocity);
        massScale = soft.massScale;
        impulseScale = soft.impulseScale;
      }

      const vn =
        (vBx - vAx) * nx +
        (vBy - vAy) * ny +
        (vBz - vAz) * nz +
        j[N_RB]! * wBx +
        j[N_RB + 1]! * wBy +
        j[N_RB + 2]! * wBz -
        j[N_RA]! * wAx -
        j[N_RA + 1]! * wAy -
        j[N_RA + 2]! * wAz;
      let impulse = -p.normalMass * massScale * (vn + velocityBias) - impulseScale * p.normalImpulse;
      const newImpulse = Math.max(p.normalImpulse + impulse, 0);
      impulse = newImpulse - p.normalImpulse;
      p.normalImpulse = newImpulse;
      if (impulse > p.maxNormalImpulse) p.maxNormalImpulse = impulse;

      const px = nx * impulse,
        py = ny * impulse,
        pz = nz * impulse;
      vAx -= mA * px;
      vAy -= mA * py;
      vAz -= mA * pz;
      vBx += mB * px;
      vBy += mB * py;
      vBz += mB * pz;
      wAx -= j[N_IA]! * impulse;
      wAy -= j[N_IA + 1]! * impulse;
      wAz -= j[N_IA + 2]! * impulse;
      wBx += j[N_IB]! * impulse;
      wBy += j[N_IB + 1]! * impulse;
      wBz += j[N_IB + 2]! * impulse;
    }

    // ---- 摩擦（按圆盘截断的库仑锥） ----
    const friction = c.friction;
    if (friction > 0) {
      const t1 = m.tangent1;
      const t2 = m.tangent2;
      for (let k = 0; k < count; k++) {
        const p = m.points[reverse ? count - 1 - k : k]!;
        const j = p.jacobian;
        const dvx = vBx - vAx,
          dvy = vBy - vAy,
          dvz = vBz - vAz;
        const vt1 =
          dvx * t1.x +
          dvy * t1.y +
          dvz * t1.z +
          j[T1 + N_RB]! * wBx +
          j[T1 + N_RB + 1]! * wBy +
          j[T1 + N_RB + 2]! * wBz -
          j[T1 + N_RA]! * wAx -
          j[T1 + N_RA + 1]! * wAy -
          j[T1 + N_RA + 2]! * wAz;
        const vt2 =
          dvx * t2.x +
          dvy * t2.y +
          dvz * t2.z +
          j[T2 + N_RB]! * wBx +
          j[T2 + N_RB + 1]! * wBy +
          j[T2 + N_RB + 2]! * wBz -
          j[T2 + N_RA]! * wAx -
          j[T2 + N_RA + 1]! * wAy -
          j[T2 + N_RA + 2]! * wAz;
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
        const px = t1.x * d1 + t2.x * d2;
        const py = t1.y * d1 + t2.y * d2;
        const pz = t1.z * d1 + t2.z * d2;
        vAx -= mA * px;
        vAy -= mA * py;
        vAz -= mA * pz;
        vBx += mB * px;
        vBy += mB * py;
        vBz += mB * pz;
        wAx -= j[T1 + N_IA]! * d1 + j[T2 + N_IA]! * d2;
        wAy -= j[T1 + N_IA + 1]! * d1 + j[T2 + N_IA + 1]! * d2;
        wAz -= j[T1 + N_IA + 2]! * d1 + j[T2 + N_IA + 2]! * d2;
        wBx += j[T1 + N_IB]! * d1 + j[T2 + N_IB]! * d2;
        wBy += j[T1 + N_IB + 1]! * d1 + j[T2 + N_IB + 1]! * d2;
        wBz += j[T1 + N_IB + 2]! * d1 + j[T2 + N_IB + 2]! * d2;
      }
    }

    // 写回（静态/运动学刚体的逆质量为 0，速度不会被改变）
    if (mA > 0) {
      lvA.x = vAx;
      lvA.y = vAy;
      lvA.z = vAz;
      avA.x = wAx;
      avA.y = wAy;
      avA.z = wAz;
    }
    if (mB > 0) {
      lvB.x = vBx;
      lvB.y = vBy;
      lvB.z = vBz;
      avB.x = wBx;
      avB.y = wBy;
      avB.z = wBz;
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
    const mA = bodyA.invMass;
    const mB = bodyB.invMass;
    const vA = bodyA.linearVelocity;
    const wA = bodyA.angularVelocity;
    const vB = bodyB.linearVelocity;
    const wB = bodyB.angularVelocity;
    const m = c.manifold;
    const n = m.normal;
    for (let i = 0; i < m.pointCount; i++) {
      const p = m.points[i]!;
      // 只对足够快的接近速度且确实产生过冲量的点处理
      if (p.relativeVelocity > -threshold || p.maxNormalImpulse === 0) continue;
      const j = p.jacobian;
      const vn =
        n.x * (vB.x - vA.x) +
        n.y * (vB.y - vA.y) +
        n.z * (vB.z - vA.z) +
        j[N_RB]! * wB.x +
        j[N_RB + 1]! * wB.y +
        j[N_RB + 2]! * wB.z -
        j[N_RA]! * wA.x -
        j[N_RA + 1]! * wA.y -
        j[N_RA + 2]! * wA.z;
      let impulse = -p.normalMass * (vn + e * p.relativeVelocity);
      const newImpulse = Math.max(p.normalImpulse + impulse, 0);
      impulse = newImpulse - p.normalImpulse;
      p.normalImpulse = newImpulse;
      if (impulse > p.maxNormalImpulse) p.maxNormalImpulse = impulse;
      if (mA > 0) {
        vA.addScaled(n, -mA * impulse);
        wA.x -= j[N_IA]! * impulse;
        wA.y -= j[N_IA + 1]! * impulse;
        wA.z -= j[N_IA + 2]! * impulse;
      }
      if (mB > 0) {
        vB.addScaled(n, mB * impulse);
        wB.x += j[N_IB]! * impulse;
        wB.y += j[N_IB + 1]! * impulse;
        wB.z += j[N_IB + 2]! * impulse;
      }
    }
  }
}
