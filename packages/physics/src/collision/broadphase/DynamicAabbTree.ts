import { AABB } from '../../math/AABB';
import type { Vec3 } from '../../math/Vec3';

const NULL_NODE = -1;

class TreeNode<T> {
  readonly aabb = new AABB();
  parent = NULL_NODE;
  child1 = NULL_NODE;
  child2 = NULL_NODE;
  /** 叶子高度为 0，空闲节点为 -1 */
  height = -1;
  userData: T | null = null;

  isLeaf(): boolean {
    return this.child1 === NULL_NODE;
  }
}

/**
 * 动态 AABB 树（增量 BVH）。
 *
 * 移植自 Box2D 的 b2DynamicTree：叶子存放“扩展（fat）AABB”，物体小幅移动时无需更新树；
 * 插入时按表面积启发式（SAH）选兄弟节点，并用 AVL 式旋转保持平衡。
 */
export class DynamicAabbTree<T> {
  private readonly nodes: TreeNode<T>[] = [];
  private freeList = NULL_NODE;
  private root = NULL_NODE;
  private proxyCount = 0;
  private readonly stack: number[] = [];

  /** 扩展 AABB 的边距 */
  readonly margin: number;

  constructor(margin = 0.1) {
    this.margin = margin;
  }

  get count(): number {
    return this.proxyCount;
  }

  /** 创建代理，aabb 为紧致包围盒，内部会扩展 margin */
  createProxy(aabb: Readonly<AABB>, userData: T): number {
    const id = this.allocateNode();
    const node = this.nodes[id]!;
    node.aabb.copy(aabb).expandByScalar(this.margin);
    node.userData = userData;
    node.height = 0;
    this.insertLeaf(id);
    this.proxyCount++;
    return id;
  }

  destroyProxy(id: number): void {
    this.removeLeaf(id);
    this.freeNode(id);
    this.proxyCount--;
  }

  /**
   * 移动代理。若新的紧致包围盒仍在原扩展包围盒内则不做任何事并返回 false。
   * @param displacement 可选的位移预测，扩展包围盒会沿该方向额外延伸
   */
  moveProxy(id: number, aabb: Readonly<AABB>, displacement?: Readonly<Vec3>): boolean {
    const node = this.nodes[id]!;
    if (node.aabb.contains(aabb)) return false;
    this.removeLeaf(id);
    const fat = node.aabb.copy(aabb).expandByScalar(this.margin);
    if (displacement) {
      const k = 2;
      if (displacement.x < 0) fat.min.x += k * displacement.x;
      else fat.max.x += k * displacement.x;
      if (displacement.y < 0) fat.min.y += k * displacement.y;
      else fat.max.y += k * displacement.y;
      if (displacement.z < 0) fat.min.z += k * displacement.z;
      else fat.max.z += k * displacement.z;
    }
    this.insertLeaf(id);
    return true;
  }

  getFatAABB(id: number): Readonly<AABB> {
    return this.nodes[id]!.aabb;
  }

  getUserData(id: number): T {
    return this.nodes[id]!.userData as T;
  }

  /** 查询与 aabb 相交的所有叶子；回调返回 false 时提前结束 */
  query(aabb: Readonly<AABB>, callback: (id: number, userData: T) => boolean | void): void {
    if (this.root === NULL_NODE) return;
    const stack = this.stack;
    const base = stack.length;
    stack.push(this.root);
    while (stack.length > base) {
      const id = stack.pop()!;
      const node = this.nodes[id]!;
      if (!node.aabb.overlaps(aabb)) continue;
      if (node.isLeaf()) {
        if (callback(id, node.userData as T) === false) {
          stack.length = base;
          return;
        }
      } else {
        stack.push(node.child1, node.child2);
      }
    }
  }

  /**
   * 射线遍历。回调返回值：&lt;0 忽略该叶子；0 终止；&gt;0 作为新的 maxT 裁剪射线。
   */
  raycast(
    origin: Readonly<Vec3>,
    dir: Readonly<Vec3>,
    maxT: number,
    callback: (id: number, userData: T, maxT: number) => number,
  ): void {
    if (this.root === NULL_NODE) return;
    let tMax = maxT;
    const stack = this.stack;
    const base = stack.length;
    stack.push(this.root);
    while (stack.length > base) {
      const id = stack.pop()!;
      const node = this.nodes[id]!;
      if (node.aabb.rayIntersect(origin, dir, tMax) < 0) continue;
      if (node.isLeaf()) {
        const value = callback(id, node.userData as T, tMax);
        if (value === 0) {
          stack.length = base;
          return;
        }
        if (value > 0 && value < tMax) tMax = value;
      } else {
        stack.push(node.child1, node.child2);
      }
    }
  }

  /** 树高（空树为 0） */
  getHeight(): number {
    return this.root === NULL_NODE ? 0 : this.nodes[this.root]!.height;
  }

  /** 校验树结构（测试用），失败抛出异常 */
  validate(): void {
    if (this.root === NULL_NODE) return;
    if (this.nodes[this.root]!.parent !== NULL_NODE) throw new Error('root.parent 应为空');
    let leaves = 0;
    const walk = (id: number): number => {
      const node = this.nodes[id]!;
      if (node.isLeaf()) {
        if (node.height !== 0) throw new Error('叶子高度应为 0');
        leaves++;
        return 0;
      }
      const c1 = this.nodes[node.child1]!;
      const c2 = this.nodes[node.child2]!;
      if (c1.parent !== id || c2.parent !== id) throw new Error('父指针错误');
      if (!node.aabb.contains(c1.aabb) || !node.aabb.contains(c2.aabb)) {
        throw new Error('父节点包围盒未包含子节点');
      }
      const h = 1 + Math.max(walk(node.child1), walk(node.child2));
      if (h !== node.height) throw new Error('高度错误');
      if (Math.abs(c1.height - c2.height) > 1) throw new Error('树不平衡');
      return h;
    };
    walk(this.root);
    if (leaves !== this.proxyCount) throw new Error('叶子数量不一致');
  }

  private allocateNode(): number {
    if (this.freeList !== NULL_NODE) {
      const id = this.freeList;
      const node = this.nodes[id]!;
      this.freeList = node.parent;
      node.parent = NULL_NODE;
      node.child1 = NULL_NODE;
      node.child2 = NULL_NODE;
      node.height = 0;
      node.userData = null;
      return id;
    }
    this.nodes.push(new TreeNode<T>());
    const node = this.nodes[this.nodes.length - 1]!;
    node.height = 0;
    return this.nodes.length - 1;
  }

  private freeNode(id: number): void {
    const node = this.nodes[id]!;
    node.parent = this.freeList;
    node.child1 = NULL_NODE;
    node.child2 = NULL_NODE;
    node.height = -1;
    node.userData = null;
    this.freeList = id;
  }

  private insertLeaf(leaf: number): void {
    const nodes = this.nodes;
    if (this.root === NULL_NODE) {
      this.root = leaf;
      nodes[leaf]!.parent = NULL_NODE;
      return;
    }

    // 1. 按 SAH 选择最佳兄弟节点
    const leafAABB = nodes[leaf]!.aabb;
    const combined = tmpAABB;
    let index = this.root;
    while (!nodes[index]!.isLeaf()) {
      const node = nodes[index]!;
      const child1 = node.child1;
      const child2 = node.child2;
      const area = node.aabb.surfaceArea();
      combined.union(node.aabb, leafAABB);
      const combinedArea = combined.surfaceArea();
      // 在此处为新叶子创建父节点的代价
      const cost = 2 * combinedArea;
      // 继续下降时祖先包围盒增大的代价
      const inheritance = 2 * (combinedArea - area);
      const cost1 = this.descendCost(child1, leafAABB) + inheritance;
      const cost2 = this.descendCost(child2, leafAABB) + inheritance;
      if (cost < cost1 && cost < cost2) break;
      index = cost1 < cost2 ? child1 : child2;
    }
    const sibling = index;

    // 2. 创建新父节点
    const oldParent = nodes[sibling]!.parent;
    const newParent = this.allocateNode();
    const np = nodes[newParent]!;
    np.parent = oldParent;
    np.userData = null;
    np.aabb.union(leafAABB, nodes[sibling]!.aabb);
    np.height = nodes[sibling]!.height + 1;
    if (oldParent !== NULL_NODE) {
      const op = nodes[oldParent]!;
      if (op.child1 === sibling) op.child1 = newParent;
      else op.child2 = newParent;
    } else {
      this.root = newParent;
    }
    np.child1 = sibling;
    np.child2 = leaf;
    nodes[sibling]!.parent = newParent;
    nodes[leaf]!.parent = newParent;

    // 3. 向上回溯，修正高度与包围盒并做平衡
    this.refit(nodes[leaf]!.parent);
  }

  private descendCost(child: number, leafAABB: Readonly<AABB>): number {
    const node = this.nodes[child]!;
    tmpAABB2.union(leafAABB, node.aabb);
    if (node.isLeaf()) return tmpAABB2.surfaceArea();
    return tmpAABB2.surfaceArea() - node.aabb.surfaceArea();
  }

  private removeLeaf(leaf: number): void {
    const nodes = this.nodes;
    if (leaf === this.root) {
      this.root = NULL_NODE;
      return;
    }
    const parent = nodes[leaf]!.parent;
    const p = nodes[parent]!;
    const grandParent = p.parent;
    const sibling = p.child1 === leaf ? p.child2 : p.child1;
    if (grandParent !== NULL_NODE) {
      const gp = nodes[grandParent]!;
      if (gp.child1 === parent) gp.child1 = sibling;
      else gp.child2 = sibling;
      nodes[sibling]!.parent = grandParent;
      this.freeNode(parent);
      this.refit(grandParent);
    } else {
      this.root = sibling;
      nodes[sibling]!.parent = NULL_NODE;
      this.freeNode(parent);
    }
  }

  private refit(start: number): void {
    const nodes = this.nodes;
    let index = start;
    while (index !== NULL_NODE) {
      index = this.balance(index);
      const node = nodes[index]!;
      const c1 = nodes[node.child1]!;
      const c2 = nodes[node.child2]!;
      node.height = 1 + Math.max(c1.height, c2.height);
      node.aabb.union(c1.aabb, c2.aabb);
      index = node.parent;
    }
  }

  /** AVL 旋转：若 iA 不平衡则把较高的子树提上来，返回新的子树根 */
  private balance(iA: number): number {
    const nodes = this.nodes;
    const A = nodes[iA]!;
    if (A.isLeaf() || A.height < 2) return iA;
    const iB = A.child1;
    const iC = A.child2;
    const B = nodes[iB]!;
    const C = nodes[iC]!;
    const bal = C.height - B.height;

    if (bal > 1) {
      // 提升 C
      const iF = C.child1;
      const iG = C.child2;
      const F = nodes[iF]!;
      const G = nodes[iG]!;
      C.child1 = iA;
      C.parent = A.parent;
      A.parent = iC;
      this.replaceChild(C.parent, iA, iC);
      if (F.height > G.height) {
        C.child2 = iF;
        A.child2 = iG;
        G.parent = iA;
        A.aabb.union(B.aabb, G.aabb);
        C.aabb.union(A.aabb, F.aabb);
        A.height = 1 + Math.max(B.height, G.height);
        C.height = 1 + Math.max(A.height, F.height);
      } else {
        C.child2 = iG;
        A.child2 = iF;
        F.parent = iA;
        A.aabb.union(B.aabb, F.aabb);
        C.aabb.union(A.aabb, G.aabb);
        A.height = 1 + Math.max(B.height, F.height);
        C.height = 1 + Math.max(A.height, G.height);
      }
      return iC;
    }

    if (bal < -1) {
      // 提升 B
      const iD = B.child1;
      const iE = B.child2;
      const D = nodes[iD]!;
      const E = nodes[iE]!;
      B.child1 = iA;
      B.parent = A.parent;
      A.parent = iB;
      this.replaceChild(B.parent, iA, iB);
      if (D.height > E.height) {
        B.child2 = iD;
        A.child1 = iE;
        E.parent = iA;
        A.aabb.union(C.aabb, E.aabb);
        B.aabb.union(A.aabb, D.aabb);
        A.height = 1 + Math.max(C.height, E.height);
        B.height = 1 + Math.max(A.height, D.height);
      } else {
        B.child2 = iE;
        A.child1 = iD;
        D.parent = iA;
        A.aabb.union(C.aabb, D.aabb);
        B.aabb.union(A.aabb, E.aabb);
        A.height = 1 + Math.max(C.height, D.height);
        B.height = 1 + Math.max(A.height, E.height);
      }
      return iB;
    }
    return iA;
  }

  private replaceChild(parent: number, oldChild: number, newChild: number): void {
    if (parent === NULL_NODE) {
      this.root = newChild;
      return;
    }
    const p = this.nodes[parent]!;
    if (p.child1 === oldChild) p.child1 = newChild;
    else p.child2 = newChild;
  }
}

const tmpAABB = new AABB();
const tmpAABB2 = new AABB();
