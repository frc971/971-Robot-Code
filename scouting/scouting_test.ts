import {browser, by, element, protractor} from 'protractor';

const EC = protractor.ExpectedConditions;

// Loads the page (or reloads it) and deals with the "Are you sure you want to
// leave this page" popup.
async function loadPage() {
  await disableAlerts();
  await browser.navigate().refresh();
  expect((await browser.getTitle())).toEqual('FRC971 Scouting Application');
  await disableAlerts();
}

// Disables alert popups. They are extremely tedious to deal with in
// Protractor since they're not angular elements. We achieve this by checking
// an invisible checkbox that's off-screen.
async function disableAlerts() {
  await browser.executeAsyncScript(function (callback) {
    let block_alerts = document.getElementById('block_alerts') as HTMLInputElement;
    block_alerts.checked = true;
    callback();
  });
}
// Returns the contents of the header that displays the "Auto", "TeleOp", and
// "Climb" labels etc.
function getHeadingText() {
  return element(by.css('.header')).getText();
}

// Returns the currently displayed progress message on the screen. This only
// exists on screens where the web page interacts with the web server.
function getProgressMessage() {
  return element(by.css('.progress_message')).getText();
}

// Returns the currently displayed error message on the screen. This only
// exists on screens where the web page interacts with the web server.
function getErrorMessage() {
  return element(by.css('.error_message')).getText();
}

// Asserts that the field on the "Submit and Review" screen has a specific
// value.
function expectReviewFieldToBe(fieldName: string, expectedValue: string) {
  return expectNthReviewFieldToBe(fieldName, 0, expectedValue);
}

// Asserts that the n'th instance of a field on the "Submit and Review"
// screen has a specific value.
async function expectNthReviewFieldToBe(fieldName: string, n: number, expectedValue: string) {
  expect(await element.all(by.cssContainingText('li', `${fieldName}:`)).get(n).getText())
      .toEqual(`${fieldName}: ${expectedValue}`);
}

// Sets a text field to the specified value.
function setTextboxByIdTo(id: string, value: string) {
  // Just sending "value" to the input fields is insufficient. We need to
  // overwrite the text that is there. If we didn't hit CTRL-A to select all
  // the text, we'd be appending to whatever is there already.
  return element(by.id(id)).sendKeys(
        protractor.Key.CONTROL, 'a', protractor.Key.NULL,
        value);
}

describe('The scouting web page', () => {
  beforeAll(async () => {
    await browser.get(browser.baseUrl);
    expect((await browser.getTitle())).toEqual('FRC971 Scouting Application');
    await disableAlerts();

    // Import the match list before running any tests. Ideally this should be
    // run in beforeEach(), but it's not worth doing that at this time. Our
    // tests are basic enough not to require this.
    await element(by.cssContainingText('.nav-link', 'Import Match List')).click();
    expect(await getHeadingText()).toEqual('Import Match List');
    await setTextboxByIdTo('year', '2016');
    await setTextboxByIdTo('event_code', 'nytr');
    await element(by.buttonText('Import')).click();

    await browser.wait(EC.textToBePresentInElement(
        element(by.css('.progress_message')), 'Successfully imported match list.'));
  });

  it('should: error on unknown match.', async () => {
    await loadPage();

    // Pick a match that doesn't exist in the 2016nytr match list.
    await setTextboxByIdTo('match_number', '3');
    await setTextboxByIdTo('team_number', '971');

    // Click Next until we get to the submit screen.
    for (let i = 0; i < 5; i++) {
      await element(by.buttonText('Next')).click();
    }
    expect(await getHeadingText()).toEqual('Review and Submit');

    // Attempt to submit and validate the error.
    await element(by.buttonText('Submit')).click();
    expect(await getErrorMessage()).toContain(
        'Failed to find team 971 in match 3 in the schedule.');
  });


  it('should: review and submit correct data.', async () => {
    await loadPage();

    // Submit scouting data for a random team that attended 2016nytr.
    expect(await getHeadingText()).toEqual('Team Selection');
    await setTextboxByIdTo('match_number', '2');
    await setTextboxByIdTo('team_number', '5254');
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Auto');
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('TeleOp');
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Climb');
    await element(by.id('high')).click();
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Other');
    await element(by.id('no_show')).click();
    await element(by.id('mechanically_broke')).click();
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Review and Submit');
    expect(await getErrorMessage()).toEqual('');

    // Validate Team Selection.
    await expectReviewFieldToBe('Match number', '2');
    await expectReviewFieldToBe('Team number', '5254');

    // Validate Auto.
    await expectNthReviewFieldToBe('Upper Shots Made', 0, '0');
    await expectNthReviewFieldToBe('Lower Shots Made', 0, '0');
    await expectNthReviewFieldToBe('Missed Shots', 0, '0');

    // Validate TeleOp.
    await expectNthReviewFieldToBe('Upper Shots Made', 1, '0');
    await expectNthReviewFieldToBe('Lower Shots Made', 1, '0');
    await expectNthReviewFieldToBe('Missed Shots', 1, '0');

    // Validate Climb.
    await expectReviewFieldToBe('Level', 'High');

    // Validate Other.
    await expectReviewFieldToBe('Defense Played On Rating', '0');
    await expectReviewFieldToBe('Defense Played Rating', '0');
    await expectReviewFieldToBe('No show', 'true');
    await expectReviewFieldToBe('Never moved', 'false');
    await expectReviewFieldToBe('Battery died', 'false');
    await expectReviewFieldToBe('Broke (mechanically)', 'true');

    await element(by.buttonText('Submit')).click();
    await browser.wait(EC.textToBePresentInElement(
        element(by.css('.header')), 'Success'));

    // TODO(phil): Make sure the data made its way to the database correctly.
  });

  it('should: load all images successfully.', async () => {
    await loadPage();

    // Get to the Auto display with the field pictures.
    expect(await getHeadingText()).toEqual('Team Selection');
    await element(by.buttonText('Next')).click();
    expect(await getHeadingText()).toEqual('Auto');

    // We expect 2 fully loaded images.
    browser.executeAsyncScript(function (callback) {
      let images = document.getElementsByTagName('img');
      let numLoaded = 0;
      for (let i = 0; i < images.length; i += 1) {
        if (images[i].naturalWidth > 0) {
          numLoaded += 1;
        }
      }
      callback(numLoaded);
    }).then(function (numLoaded) {
      expect(numLoaded).toBe(2);
    });
  });
});
