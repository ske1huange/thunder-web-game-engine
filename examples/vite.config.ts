import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vite';

export default defineConfig({
  base: './',
  resolve: {
    alias: {
      // 开发时直接引用引擎源码，改动即时生效，无需先构建 packages/physics
      '@thunder/physics': fileURLToPath(
        new URL('../packages/physics/src/index.ts', import.meta.url),
      ),
      '@thunder/render': fileURLToPath(new URL('../packages/render/src/index.ts', import.meta.url)),
    },
    // 引擎包与演示站共用同一份 three.js
    dedupe: ['three'],
  },
  // 物理 Worker 以 ES 模块打包
  worker: { format: 'es' },
  build: {
    target: 'es2022',
    // three.js 本身约 600KB，放宽告警阈值
    chunkSizeWarningLimit: 1000,
  },
});
