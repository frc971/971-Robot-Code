const fs = require('fs');
const path = require('path');
const {spawnSync} = require('child_process');

const output_dir = path.join(
  process.env.BAZEL_BINDIR,
  process.env.BAZEL_PACKAGE
);
console.log(output_dir);
console.log(process.argv[2]);
console.log(process.cwd());
const ngsw_config = process.argv[2];
console.log(`Trying to run ${ngsw_config} ${process.argv.slice(4).join(' ')}`);
const result = spawnSync(ngsw_config, process.argv.slice(4), {
  stdout: 'inherit',
  stderr: 'inherit',
});

if (result.status || result.error || result.signal) {
  console.log("Failed to run 'ngsw_config'");
  console.log(`status: ${result.status}`);
  console.log(`error: ${result.error}`);
  console.log(`signal: ${result.signal}`);
  console.log(`stdout: ${result.stdout}`);
  console.log(`stderr: ${result.stderr}`);
  process.exit(1);
}

const currentDirectory = process.cwd();

// Read the contents of the current directory
console.log(`Contents of the current directory: ${currentDirectory}`);
fs.readdirSync(currentDirectory).forEach((file) => {
  console.log(file);
});

fs.copyFileSync(path.join(process.argv[4], 'ngsw.json'), process.argv[3]);
