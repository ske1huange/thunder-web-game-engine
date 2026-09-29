// 发布前检查：publint（exports / types / files 配置）+ npm pack 预演（发布内容）。
// 运行：pnpm check:packages（需先 pnpm build）
import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { publint } from 'publint';
import { formatMessage } from 'publint/utils';

const packages = ['packages/physics', 'packages/render'];
const required = [
  'package.json',
  'README.md',
  'LICENSE',
  'CHANGELOG.md',
  'dist/index.js',
  'dist/index.cjs',
  'dist/index.d.ts',
];
let failed = false;

for (const dir of packages) {
  const pkg = JSON.parse(readFileSync(`${dir}/package.json`, 'utf8'));
  console.log(`\n${pkg.name}@${pkg.version}`);

  const { messages } = await publint({ pkgDir: dir, level: 'warning', strict: true });
  for (const m of messages) {
    console.log(`  publint ${m.type}: ${formatMessage(m, pkg)}`);
    failed = true;
  }

  const [packed] = JSON.parse(
    execFileSync('npm', ['pack', '--dry-run', '--json', '--ignore-scripts'], {
      cwd: dir,
      encoding: 'utf8',
    }),
  );
  const files = packed.files.map((f) => f.path);
  for (const f of required) {
    if (!files.includes(f)) {
      console.log(`  缺少文件：${f}`);
      failed = true;
    }
  }
  const unexpected = files.filter((f) => f.startsWith('src/') || f.startsWith('test/'));
  if (unexpected.length) {
    console.log(`  不应发布的文件：${unexpected.join(', ')}`);
    failed = true;
  }
  console.log(
    `  ${files.length} 个文件，打包 ${(packed.size / 1024).toFixed(1)} KB，解压 ${(packed.unpackedSize / 1024).toFixed(1)} KB`,
  );
}

if (failed) {
  console.error('\n发布检查未通过');
  process.exit(1);
}
console.log('\n发布检查通过');
