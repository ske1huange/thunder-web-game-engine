import { defineConfig } from 'tsup';

export default defineConfig([
  {
    entry: { index: 'src/index.ts' },
    format: ['esm', 'cjs'],
    dts: true,
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
