import { defineConfig } from 'tsup';

export default defineConfig({
  entry: { index: 'src/index.ts' },
  format: ['esm', 'cjs'],
  // 构建时 @thunder/physics 解析到其 dist（依赖拓扑保证先构建），不把物理引擎打进来
  tsconfig: 'tsconfig.build.json',
  // tsup 的 d.ts 构建会注入 baseUrl，在 TypeScript 6 中需显式忽略弃用提示
  dts: { compilerOptions: { ignoreDeprecations: '6.0' } },
  sourcemap: true,
  clean: true,
  target: 'es2022',
  external: ['three', /^three\//, '@thunder/physics'],
});
