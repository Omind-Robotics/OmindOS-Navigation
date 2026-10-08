const fs = require('node:fs');
const esbuild = require('../../web/robot-workbench/node_modules/esbuild');
const result = esbuild.transformSync(fs.readFileSync(process.argv[2], 'utf8'), {
  minifyWhitespace: true, minifySyntax: true, minifyIdentifiers: false,
  legalComments: 'inline', sourcemap: false, target: 'es2020'
});
fs.writeFileSync(process.argv[3], result.code);
