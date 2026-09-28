import { defineConfig } from 'vitest/config';

export default defineConfig({
  test: {
    include: ['packages/*/test/**/*.test.ts'],
    benchmark: {
      include: ['bench/**/*.bench.ts'],
    },
  },
});
