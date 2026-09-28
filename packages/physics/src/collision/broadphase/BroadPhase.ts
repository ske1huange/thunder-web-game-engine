import type { AABB } from '../../math/AABB';
import type { Vec3 } from '../../math/Vec3';
import { DynamicAabbTree } from './DynamicAabbTree';

export const ProxyType = {
  Static: 0,
  Dynamic: 1,
} as const;
export type ProxyType = (typeof ProxyType)[keyof typeof ProxyType];

/** 代理句柄：树类型 + 树内节点编号 */
export interface ProxyHandle {
  type: ProxyType;
  id: number;
}

/**
 * 宽相：静态物体与动态/运动学物体分别放在两棵 AABB 树里（参考 Box2D v3），
 * 这样静态-静态之间永远不会产生配对。只有扩展包围盒发生变化的代理才会重新查询配对。
 */
export class BroadPhase<T> {
  readonly staticTree: DynamicAabbTree<T>;
  readonly dynamicTree: DynamicAabbTree<T>;
  private moveBuffer: ProxyHandle[] = [];
  private readonly moveSet = new Set<number>();

  constructor(margin = 0.1) {
    this.staticTree = new DynamicAabbTree<T>(margin);
    this.dynamicTree = new DynamicAabbTree<T>(margin);
  }

  private tree(type: ProxyType): DynamicAabbTree<T> {
    return type === ProxyType.Static ? this.staticTree : this.dynamicTree;
  }

  createProxy(aabb: Readonly<AABB>, type: ProxyType, userData: T): ProxyHandle {
    const id = this.tree(type).createProxy(aabb, userData);
    const handle = { type, id };
    this.bufferMove(handle);
    return handle;
  }

  destroyProxy(handle: ProxyHandle): void {
    this.unbufferMove(handle);
    this.tree(handle.type).destroyProxy(handle.id);
  }

  /** 移动代理；扩展包围盒被更新时返回 true */
  moveProxy(handle: ProxyHandle, aabb: Readonly<AABB>, displacement?: Readonly<Vec3>): boolean {
    const moved = this.tree(handle.type).moveProxy(handle.id, aabb, displacement);
    if (moved) this.bufferMove(handle);
    return moved;
  }

  getFatAABB(handle: ProxyHandle): Readonly<AABB> {
    return this.tree(handle.type).getFatAABB(handle.id);
  }

  testOverlap(a: ProxyHandle, b: ProxyHandle): boolean {
    return this.getFatAABB(a).overlaps(this.getFatAABB(b));
  }

  /** 标记需要重新查询配对的代理（例如过滤条件改变后） */
  touchProxy(handle: ProxyHandle): void {
    this.bufferMove(handle);
  }

  /**
   * 为所有移动过的代理查找新的潜在配对。
   * 同一对可能被报告两次（双方都移动时），由调用方去重。
   */
  updatePairs(onPair: (a: T, b: T) => void): void {
    const buffer = this.moveBuffer;
    for (const handle of buffer) {
      const tree = this.tree(handle.type);
      const fat = tree.getFatAABB(handle.id);
      const self = tree.getUserData(handle.id);
      if (handle.type === ProxyType.Dynamic) {
        this.dynamicTree.query(fat, (id, other) => {
          if (id === handle.id) return true;
          // 双方都移动时只由编号较小的一方报告
          if (id < handle.id && this.moveSet.has(key(ProxyType.Dynamic, id))) return true;
          onPair(self, other);
          return true;
        });
        this.staticTree.query(fat, (_id, other) => {
          onPair(self, other);
          return true;
        });
      } else {
        this.dynamicTree.query(fat, (id, other) => {
          if (this.moveSet.has(key(ProxyType.Dynamic, id))) return true;
          onPair(self, other);
          return true;
        });
      }
    }
    this.moveBuffer = [];
    this.moveSet.clear();
  }

  query(aabb: Readonly<AABB>, callback: (userData: T) => boolean | void): void {
    let stop = false;
    this.staticTree.query(aabb, (_id, data) => {
      if (callback(data) === false) {
        stop = true;
        return false;
      }
      return true;
    });
    if (stop) return;
    this.dynamicTree.query(aabb, (_id, data) => callback(data));
  }

  /** 射线遍历两棵树，回调语义同 DynamicAabbTree.raycast */
  raycast(
    origin: Readonly<Vec3>,
    dir: Readonly<Vec3>,
    maxT: number,
    callback: (userData: T, maxT: number) => number,
  ): void {
    let tMax = maxT;
    let stop = false;
    const wrap = (_id: number, data: T, t: number): number => {
      const v = callback(data, t);
      if (v === 0) stop = true;
      else if (v > 0 && v < tMax) tMax = v;
      return v;
    };
    this.staticTree.raycast(origin, dir, tMax, wrap);
    if (stop) return;
    this.dynamicTree.raycast(origin, dir, tMax, wrap);
  }

  private bufferMove(handle: ProxyHandle): void {
    const k = key(handle.type, handle.id);
    if (this.moveSet.has(k)) return;
    this.moveSet.add(k);
    this.moveBuffer.push(handle);
  }

  private unbufferMove(handle: ProxyHandle): void {
    const k = key(handle.type, handle.id);
    if (!this.moveSet.delete(k)) return;
    const i = this.moveBuffer.findIndex((h) => h.type === handle.type && h.id === handle.id);
    if (i >= 0) this.moveBuffer.splice(i, 1);
  }
}

function key(type: ProxyType, id: number): number {
  return id * 2 + type;
}
