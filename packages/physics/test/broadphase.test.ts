import { describe, expect, it } from 'vitest';
import { AABB, BroadPhase, DynamicAabbTree, ProxyType, Vec3 } from '../src';

function rng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

function randomBox(rand: () => number, extent = 50): AABB {
  const c = new Vec3((rand() - 0.5) * extent, (rand() - 0.5) * extent, (rand() - 0.5) * extent);
  const h = new Vec3(0.2 + rand() * 2, 0.2 + rand() * 2, 0.2 + rand() * 2);
  return new AABB(c.clone().sub(h), c.clone().add(h));
}

describe('DynamicAabbTree', () => {
  it('随机插入/移动/删除后结构合法，查询结果与暴力法一致', () => {
    const rand = rng(7);
    const tree = new DynamicAabbTree<number>(0.1);
    const boxes = new Map<number, AABB>();
    const ids: number[] = [];
    for (let i = 0; i < 500; i++) {
      const b = randomBox(rand);
      const id = tree.createProxy(b, i);
      boxes.set(id, b);
      ids.push(id);
    }
    tree.validate();
    // 随机移动
    for (let round = 0; round < 5; round++) {
      for (const id of ids) {
        if (rand() < 0.3) {
          const b = boxes.get(id)!;
          const d = new Vec3((rand() - 0.5) * 3, (rand() - 0.5) * 3, (rand() - 0.5) * 3);
          b.min.add(d);
          b.max.add(d);
          tree.moveProxy(id, b, d);
        }
      }
      tree.validate();
    }
    // 删除一部分
    for (let i = ids.length - 1; i >= 0; i -= 3) {
      tree.destroyProxy(ids[i]!);
      boxes.delete(ids[i]!);
    }
    tree.validate();
    expect(tree.count).toBe(boxes.size);
    // 高度应接近 log2(n)
    expect(tree.getHeight()).toBeLessThan(20);

    for (let q = 0; q < 100; q++) {
      const query = randomBox(rand);
      const found = new Set<number>();
      tree.query(query, (id) => {
        found.add(id);
      });
      // 树中存的是扩展包围盒：紧致包围盒相交的必须全部找到
      for (const [id, b] of boxes) {
        if (b.overlaps(query)) expect(found.has(id)).toBe(true);
        if (found.has(id)) expect(tree.getFatAABB(id).overlaps(query)).toBe(true);
      }
    }
  });

  it('射线遍历能找到最近的叶子', () => {
    const tree = new DynamicAabbTree<string>(0);
    tree.createProxy(new AABB(new Vec3(4, -1, -1), new Vec3(6, 1, 1)), 'near');
    tree.createProxy(new AABB(new Vec3(9, -1, -1), new Vec3(11, 1, 1)), 'far');
    tree.createProxy(new AABB(new Vec3(4, 5, -1), new Vec3(6, 7, 1)), 'off');
    const origin = new Vec3(0, 0, 0);
    const dir = new Vec3(1, 0, 0);
    const hits: string[] = [];
    tree.raycast(origin, dir, 100, (id, data) => {
      hits.push(data);
      return tree.getFatAABB(id).rayIntersect(origin, dir, 100);
    });
    expect(hits).toContain('near');
    expect(hits).not.toContain('off');
  });

  it('小幅移动不会触发更新', () => {
    const tree = new DynamicAabbTree<number>(0.5);
    const b = new AABB(new Vec3(0, 0, 0), new Vec3(1, 1, 1));
    const id = tree.createProxy(b, 0);
    b.min.x += 0.2;
    b.max.x += 0.2;
    expect(tree.moveProxy(id, b)).toBe(false);
    b.min.x += 1;
    b.max.x += 1;
    expect(tree.moveProxy(id, b)).toBe(true);
  });
});

describe('BroadPhase', () => {
  it('配对结果与暴力法一致，且静态之间不配对', () => {
    const rand = rng(99);
    const bp = new BroadPhase<number>(0.1);
    const handles = [];
    const types: ProxyType[] = [];
    for (let i = 0; i < 300; i++) {
      const type = i % 4 === 0 ? ProxyType.Static : ProxyType.Dynamic;
      handles.push(bp.createProxy(randomBox(rand, 30), type, i));
      types.push(type);
    }
    const pairs = new Set<string>();
    const keyOf = (a: number, b: number) => (a < b ? `${a}-${b}` : `${b}-${a}`);
    bp.updatePairs((a, b) => {
      pairs.add(keyOf(a, b));
    });
    for (let i = 0; i < handles.length; i++) {
      for (let j = i + 1; j < handles.length; j++) {
        const overlap = bp.testOverlap(handles[i]!, handles[j]!);
        const bothStatic = types[i] === ProxyType.Static && types[j] === ProxyType.Static;
        expect(pairs.has(keyOf(i, j))).toBe(overlap && !bothStatic);
      }
    }
    // 没有移动时不再产生配对
    let count = 0;
    bp.updatePairs(() => count++);
    expect(count).toBe(0);
  });
});
