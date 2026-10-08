const path = require('node:path');
const fs = require('node:fs');
require('esbuild').buildSync({
  absWorkingDir: __dirname,
  entryPoints: ['robot-scene.source.js'],
  nodePaths: (process.env.NODE_PATH || '').split(path.delimiter).filter(Boolean),
  bundle: true,
  format: 'esm',
  minify: true,
  target: 'es2020',
  outfile: 'robot-scene.js',
  legalComments: 'inline',
});
// GLSL template strings in the upstream bundle contain trailing indentation.
// Removing it leaves shader meaning unchanged and keeps repository diff checks clean.
const output = path.join(__dirname, 'robot-scene.js');
fs.writeFileSync(output, fs.readFileSync(output, 'utf8').replace(/[\t ]+$/gm, ''));
