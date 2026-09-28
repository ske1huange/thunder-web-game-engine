import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vitest/config';

const src = (pkg: string) =>
  fileURLToPath(new URL(`./packages/${pkg}/src/index.ts`, import.meta.url));

export default defineConfig({
  resolve: {
    // 测试直接使用各包源码，无需先构建
    alias: {
      '@thunder/physics': src('physics'),
      '@thunder/render': src('render'),
    },
  },
  test: {
    include: ['packages/*/test/**/*.test.ts'],
  },
});
