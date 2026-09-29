// 包体积检查：用 esbuild 压缩打包构建产物（three 等依赖外置），统计 gzip 后的大小并与预算比较。
// 运行：pnpm size（需先 pnpm build）
import { build } from 'esbuild';
import { gzipSync } from 'node:zlib';

/** 预算（gzip 后的字节数） */
const cases = [
  {
    name: '@thunder/physics（全部导出）',
    code: "export * from './packages/physics/dist/index.js';",
    budget: 56 * 1024,
  },
  {
    name: '@thunder/physics（最小用法：World + Box + Plane）',
    code: "export { World, BoxShape, PlaneShape, Vec3 } from './packages/physics/dist/index.js';",
    budget: 36 * 1024,
  },
  {
    name: '@thunder/render（不含 three）',
    code: "export * from './packages/render/dist/index.js';",
    budget: 16 * 1024,
  },
  {
    name: '@thunder/render（最小用法：RenderPipeline + PhysicsView）',
    code: "export { RenderPipeline, PhysicsView } from './packages/render/dist/index.js';",
    budget: 9 * 1024,
  },
];

let failed = false;
const rows = [];
for (const c of cases) {
  const result = await build({
    stdin: { contents: c.code, resolveDir: process.cwd(), loader: 'js' },
    bundle: true,
    minify: true,
    format: 'esm',
    target: 'es2022',
    write: false,
    external: ['three', 'three/*', '@thunder/physics'],
    logLevel: 'silent',
  });
  const bytes = result.outputFiles[0].contents;
  const gzip = gzipSync(bytes, { level: 9 }).length;
  const ok = gzip <= c.budget;
  if (!ok) failed = true;
  rows.push({
    场景: c.name,
    '压缩后 KB': (bytes.length / 1024).toFixed(1),
    'gzip KB': (gzip / 1024).toFixed(1),
    '预算 KB': (c.budget / 1024).toFixed(0),
    结果: ok ? '✓' : '超出预算',
  });
}
console.table(rows);
if (failed) process.exit(1);
