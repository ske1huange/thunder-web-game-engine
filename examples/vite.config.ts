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
    },
  },
  build: {
    target: 'es2022',
  },
});
