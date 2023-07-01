const child_process = require('child_process');
const process = require('process');

const cypress = require('cypress');

// Set up the xvfb binary.
// TODO(philipp): Figure out how to point Xvfb at the sandboxed usr/bin
// directory. Currently impossible as it's hardcoded to use /usr/bin.
process.env['PATH'] = [
  `${process.env.RUNFILES_DIR}/xvfb_amd64/wrapped_bin`,
  `${process.env.RUNFILES_DIR}/xvfb_amd64/usr/bin`,
  process.env.PATH,
].join(':');

// Start the web server, database, and fake TBA server.
// We use file descriptor 3 ('pipe') for the test server to let us know when
// everything has started up.
console.log('Starting server.');
let servers = child_process.spawn(
  'testing/scouting_test_servers',
  ['--port=8000', '--notify_fd=3'],
  {
    stdio: ['inherit', 'inherit', 'inherit', 'pipe'],
  }
);

// Wait for the server to finish starting up.
const serverStartup = new Promise((resolve, reject) => {
  let cumulativeData = '';
  servers.stdio[3].on('data', async (data) => {
    console.log('Got data: ' + data);
    cumulativeData += data;
    if (cumulativeData.includes('READY')) {
      console.log('Everything is ready!');
      resolve();
    }
  });

  servers.on('error', (err) => {
    console.log(`Failed to start scouting_test_servers: ${err}`);
    reject();
  });

  servers.on('close', (code, signal) => {
    console.log(`scouting_test_servers closed: ${code} (${signal})`);
    reject();
  });

  servers.on('exit', (code, signal) => {
    console.log(`scouting_test_servers exited: ${code} (${signal})`);
    reject();
  });
});

// Wait for the server to shut down.
const serverShutdown = new Promise((resolve) => {
  servers.on('exit', () => {
    resolve();
  });
});

// Wait for the server to be ready, run the tests, then shut down the server.
(async () => {
  // Parse command line options.
  let runOptions = await cypress.cli.parseRunArguments(process.argv.slice(2));

  await serverStartup;
  const result = await cypress.run(
    Object.assign(runOptions, {
      config: {
        baseUrl: 'http://localhost:8000',
        screenshotsFolder:
          process.env.TEST_UNDECLARED_OUTPUTS_DIR + '/screenshots',
        video: false,
        videosFolder: process.env.TEST_UNDECLARED_OUTPUTS_DIR + '/videos',
      },
    })
  );
  await servers.kill();
  await serverShutdown;

  exitCode = 0;
  if (result.status == 'failed') {
    exitCode = 1;
    console.log('-'.repeat(50));
    console.log('Test FAILED: ' + result.message);
    console.log('-'.repeat(50));
  } else if (result.totalFailed > 0) {
    // When the "before" hook fails, we don't get a "failed" mesage for some
    // reason. In that case, we just have to exit with an error.
    exitCode = 1;
  }
  process.exit(exitCode);
})();
