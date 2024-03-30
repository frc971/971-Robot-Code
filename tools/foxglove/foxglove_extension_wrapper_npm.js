// This script acts as an "npm" binary for the foxglove-extension binary. We
// don't actually care to do any npm things here. For some reason
// foxglove-extension defers to npm to execute the various build stages. So
// all this script does is execute those various build stages. The stages are
// defined in the package.json file.

const fs = require('fs');
const { execSync } = require('child_process');
const path = require('path');

// Read the package.json file.
function readPackageJson() {
    try {
        const packageJson = fs.readFileSync('package.json', 'utf8');
        return JSON.parse(packageJson);
    } catch (error) {
        console.error('Error reading package.json:', error);
        process.exit(1);
    }
}

// Execute the named script specified in package.json.
function executeScript(scriptName) {
    const packageJson = readPackageJson();
    const scripts = packageJson.scripts || {};

    if (!scripts[scriptName]) {
        console.error(`Script '${scriptName}' not found in package.json`);
        process.exit(1);
    }

    // We cannot execute the `foxglove-extension` binary as-is (at least not
    // without setting up a custom PATH). So we instead point at the
    // Bazel-generated wrapper script for that binary.
    const scriptParts = scripts[scriptName].split(' ');
    const bin = scriptParts[0];
    if (bin !== 'foxglove-extension') {
        console.error(`Cannot support commands other than 'foxglove-extension'. Got: ${bin}`);
        process.exit(1);
    }
    scriptParts[0] = path.join(__dirname, 'foxglove_extension.sh');

    // Execute the `foxglove-extension` command specified in the script.
    try {
        console.log(`Executing script '${scriptName}'...`);
        execSync(scriptParts.join(' '), { stdio: 'inherit' });
    } catch (error) {
        console.error(`Error executing script '${scriptName}':`, error);
        process.exit(1);
    }
}

function main() {
    // Validate the input arguments.
    if (process.argv.length !== 4) {
        console.error('Usage: node foxglove_extension_wrapper_npm.js <scriptName>');
        process.exit(1);
    }
    if (process.argv[2] !== "run") {
        console.error(`Cannot support commands other than 'run'. Got: ${process.argv[2]}`);
        process.exit(1);
    }

    // Run the script specified in the package.json file.
    const scriptName = process.argv[3];
    executeScript(scriptName);
}

main();
