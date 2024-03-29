/*
 * This test here validates that the re-authorization logic works at a very high level. Ideally we
 * would validate this with an actual Apache server, but that's a little tedious to set up. Instead,
 * we create a custom Go server that behaves as if the user is unauthorized. This works for our
 * immediate purposes.
 *
 * This test cannot be written in the usual //scouting:scouting_test because the Cypress test
 * framework does not support dealing with multiple tabs.
 */

const child_process = require('child_process');
const process = require('process');

const {Builder, By, Key, until, Condition} = require('selenium-webdriver');
const chrome = require('selenium-webdriver/chrome');

// These are the sandboxed binaries we want to point webdriver at.
const CHROME_PATH = '../../../../../chrome_linux/chrome';
const CHROMEDRIVER_PATH =
  '../../../../../chromedriver_linux/chromedriver-linux64/chromedriver';

// Checks if the specified element contains the specified text.
const elementContainsText = (element, text) => {
  return async (driver) => {
    const actualText = await element.getText();
    console.log(`Current message: ${actualText}`);
    return actualText.includes(text);
  };
};

// Asserts that the text includes the specified "include" text.
const expectTextToInclude = (text, include) => {
  if (!text.includes(include)) {
    throw new Error(`Could not find "${include}" in "${text}"`);
  }
};

// Run the actual test. We want to validate that the re-authorization helper hits the /authorize
// endpoint properly.
async function runTest(driver) {
  // Validate that the page opened properly.
  title = await driver.getTitle();
  expectTextToInclude(title, 'RPC Authorize Test');

  // When the page first loads, it should tell us it's waiting for a button click.
  const messageElement = await driver.wait(
    until.elementLocated(By.id('message')),
    10000
  );
  const message = await messageElement.getText();
  expectTextToInclude(message, 'Waiting for button click');

  // Click the "without re-auth" button and make sure we get a permission denied error.
  let withoutReAuthButton = await driver.findElement(By.id('without_re_auth'));
  withoutReAuthButton.click();
  await driver.wait(
    elementContainsText(messageElement, 'Failed to perform submission: 401!'),
    10000
  );

  // Click the "with re-auth" button and validate that we get a successful re-authorization.
  let withReAuthButton = await driver.findElement(By.id('with_re_auth'));
  withReAuthButton.click();
  await driver.wait(
    elementContainsText(messageElement, 'Successful submission 1!'),
    10000
  );

  // Now click the "without re-auth" button again and make sure it's successful.
  withoutReAuthButton.click();
  await driver.wait(
    elementContainsText(messageElement, 'Successful submission 2!'),
    10000
  );
}

(async () => {
  // Start the dummy server in the background.
  console.log('Starting server.');
  let server = child_process.spawn(
    './authorize_/authorize',
    ['-directory', './static_files'],
    {stdio: ['inherit', 'inherit', 'inherit']}
  );

  // Start up Chrome in the background.
  let chromeOptions = new chrome.Options();
  chromeOptions.setChromeBinaryPath(CHROME_PATH);
  chromeOptions.addArguments('--headless');
  chromeOptions.addArguments('--no-sandbox');
  chromeOptions.addArguments('--disable-dev-shm-usage');
  chromeOptions.addArguments('--remote-debugging-pipe');

  let service = new chrome.ServiceBuilder(CHROMEDRIVER_PATH).build();
  const driver = chrome.Driver.createSession(chromeOptions, service);

  // Load the page and wait for it to finish loading.
  driver.get('http://localhost:8000/');
  await driver.wait(function () {
    return driver
      .executeScript('return document.readyState')
      .then(function (readyState) {
        return readyState === 'complete';
      });
  });

  // Run the actual test we care about.
  let exitCode = 0;
  try {
    await runTest(driver);
  } catch (error) {
    console.log(`Failed test: ${error}`);
    exitCode = 1;
  }

  // Shut everything down.
  driver.quit();
  console.log('Killing server.');
  await server.kill();
  process.exit(exitCode);
})();
