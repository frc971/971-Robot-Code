// This script acts as a wrapper for the `foxglove-extension` binary. We need a
// wrapper here because `foxglove-extension` wants to invoke `npm` directly.
// Since we don't support the real npm binary, we force `foxglove-extension` to
// use our fake npm binary (`foxglove_extension_wrapper_npm.js`) by
// manipulating the PATH.

const { spawnSync } = require('child_process');
const path = require('path');
const process = require('process');
const fs = require('fs');
const { tmpdir } = require('os');

// Add a directory to the PATH environment variable.
function addToPath(directory) {
    const currentPath = process.env.PATH || '';
    const newPath = `${directory}${path.delimiter}${currentPath}`;
    process.env.PATH = newPath;
}

const fakeNpm = path.join(__dirname, 'foxglove_extension_wrapper_npm.sh');

const tempBinDir = fs.mkdtempSync(path.join(tmpdir(), "foxglove_extension_wrapper-tmp-"));
fs.symlinkSync(fakeNpm, path.join(tempBinDir, 'npm'));

addToPath(tempBinDir);

// Create a relative path for a specific root-relative directory.
function getRelativePath(filePath) {
    // Count the number of directories and construct the relative path.
    const numDirectories = filePath.split('/').length;
    return '../'.repeat(numDirectories);
}

// We need to know the path to the `foxglove-extension` binary from the
// sub-directory where we're generating code into.
const relativePath = getRelativePath(process.env.BAZEL_PACKAGE);
const foxgloveExtensionPath = path.join(relativePath, `tools/foxglove/foxglove_extension.sh`)

// Extract arguments intended for the `foxglove-extension` binary.
const args = process.argv.slice(2);

// Execute the `foxglove-extension` binary.
try {
    const result = spawnSync(foxgloveExtensionPath, args, { stdio: 'inherit', cwd: process.env.BAZEL_PACKAGE });
    if (result.error) {
        console.error('Error executing foxglove_extension:', result.error);
        process.exit(1);
    }
    if (result.status !== 0) {
        console.error(`foxglove_extension exited with status ${result.status}`);
        process.exit(result.status);
    }
} catch (error) {
    console.error('Error executing foxglove_extension:', error);
    process.exit(1);
}
