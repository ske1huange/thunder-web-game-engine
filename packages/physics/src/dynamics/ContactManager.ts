import type { BroadPhase } from '../collision/broadphase/BroadPhase';
import type { CollideConfig } from '../collision/narrowphase/Collide';
import { type Collider, shouldFiltersCollide } from './Collider';
import { Contact } from './Contact';

/** 接触状态变化的回调 */
export interface ContactListener {
  onBegin(contact: Contact): void;
  onEnd(contact: Contact): void;
}

const KEY_SCALE = 2 ** 26;
const scratchConfig: CollideConfig = { speculativeDistance: 0, linearSlop: 0 };

/** 管理所有接触：创建、更新（窄相）与销毁 */
export class ContactManager {
  readonly contacts: Contact[] = [];
  private readonly map = new Map<number, Contact>();

  constructor(
    private readonly broadPhase: BroadPhase<Collider>,
    private readonly listener: ContactListener,
  ) {}

  static pairKey(a: Collider, b: Collider): number {
    return a.id < b.id ? a.id * KEY_SCALE + b.id : b.id * KEY_SCALE + a.id;
  }

  get count(): number {
    return this.contacts.length;
  }

  find(a: Collider, b: Collider): Contact | undefined {
    return this.map.get(ContactManager.pairKey(a, b));
  }

  /** 两个碰撞体是否应当产生接触 */
  static shouldCollide(a: Collider, b: Collider): boolean {
    const bodyA = a.body;
    const bodyB = b.body;
    if (bodyA === bodyB) return false;
    if (a.isSensor && b.isSensor) return false;
    if (a.isSensor || b.isSensor) {
      // 传感器：至少一方可移动
      if (bodyA.isStatic() && bodyB.isStatic()) return false;
    } else if (!bodyA.isDynamic() && !bodyB.isDynamic()) {
      return false;
    }
    if (!shouldFiltersCollide(a.filter, b.filter)) return false;
    if (!a.isSensor && !b.isSensor) {
      for (const j of bodyA.joints) {
        if (!j.collideConnected && (j.bodyA === bodyB || j.bodyB === bodyB)) return false;
      }
    }
    return true;
  }

  /** 宽相发现的新配对 */
  addPair(a: Collider, b: Collider): void {
    const key = ContactManager.pairKey(a, b);
    if (this.map.has(key)) return;
    if (!ContactManager.shouldCollide(a, b)) return;
    // 按编号排序，保证 A/B 顺序确定
    const [ca, cb] = a.id < b.id ? [a, b] : [b, a];
    const contact = new Contact(ca, cb, key);
    contact.index = this.contacts.length;
    this.contacts.push(contact);
    this.map.set(key, contact);
    ca.body.contacts.push(contact);
    cb.body.contacts.push(contact);
  }

  /** 销毁接触；若处于接触状态会触发结束回调 */
  destroy(contact: Contact, notify = true): void {
    if (contact.index < 0) return;
    if (notify && contact.touching) this.listener.onEnd(contact);
    contact.touching = false;
    const last = this.contacts.pop()!;
    if (last !== contact) {
      this.contacts[contact.index] = last;
      last.index = contact.index;
    }
    contact.index = -1;
    this.map.delete(contact.key);
    removeFrom(contact.bodyA.contacts, contact);
    removeFrom(contact.bodyB.contacts, contact);
  }

  /** 销毁与某个碰撞体相关的所有接触 */
  destroyForCollider(collider: Collider, notify = true): void {
    const list = collider.body.contacts;
    for (let i = list.length - 1; i >= 0; i--) {
      const c = list[i]!;
      if (c.colliderA === collider || c.colliderB === collider) this.destroy(c, notify);
      if (i > list.length) i = list.length;
    }
  }

  /**
   * 窄相：更新所有接触。扩展包围盒不再重叠的接触被销毁；
   * 双方都休眠（或静止）的接触跳过更新。
   */
  update(config: CollideConfig, dt: number, maxSpeculative: number): void {
    const contacts = this.contacts;
    const base = config.speculativeDistance;
    const local = scratchConfig;
    local.linearSlop = config.linearSlop;
    let i = 0;
    while (i < contacts.length) {
      const c = contacts[i]!;
      const a = c.colliderA;
      const b = c.colliderB;
      if (!this.broadPhase.testOverlap(a.proxy!, b.proxy!)) {
        this.destroy(c);
        continue; // 末尾元素已换到 i
      }
      const awakeA = a.body.isAwake;
      const awakeB = b.body.isAwake;
      if (awakeA || awakeB) {
        // 推测距离随相对速度增大（以宽相边距为上限），避免高速物体在一步内穿入
        const bA = a.body;
        const bB = b.body;
        const bound =
          (bA.linearVelocity.length() +
            bA.maxExtent * bA.angularVelocity.length() +
            bB.linearVelocity.length() +
            bB.maxExtent * bB.angularVelocity.length()) *
          dt;
        local.speculativeDistance = base + Math.min(bound, maxSpeculative);
        const was = c.touching;
        const now = c.update(local);
        if (now && !was) this.listener.onBegin(c);
        else if (!now && was) this.listener.onEnd(c);
        // 与休眠刚体接触时将其唤醒
        if (now && !c.isSensor && awakeA !== awakeB) {
          if (!awakeA && a.body.isDynamic()) a.body.wakeUp();
          if (!awakeB && b.body.isDynamic()) b.body.wakeUp();
        }
      }
      i++;
    }
  }
}

function removeFrom<T>(arr: T[], item: T): void {
  const i = arr.indexOf(item);
  if (i >= 0) {
    arr[i] = arr[arr.length - 1]!;
    arr.pop();
  }
}
