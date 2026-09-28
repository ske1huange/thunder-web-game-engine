import { defineConfig } from 'tsup';

export default defineConfig([
  {
    entry: { index: 'src/index.ts' },
    format: ['esm', 'cjs'],
    // tsup 的 d.ts 构建会注入 baseUrl，在 TypeScript 6 中需显式忽略弃用提示
    dts: { compilerOptions: { ignoreDeprecations: '6.0' } },
    sourcemap: true,
    clean: true,
    target: 'es2022',
  },
  {
    entry: { 'thunder-physics': 'src/index.ts' },
    format: ['iife'],
    globalName: 'ThunderPhysics',
    minify: true,
    sourcemap: true,
    target: 'es2022',
  },
]);
