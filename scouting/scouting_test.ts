import {browser, by, element, protractor} from 'protractor';

// Loads the page (or reloads it) and deals with the "Are you sure you want to
// leave this page" popup.
async function loadPage() {
  await browser.get(browser.baseUrl).catch(function () {
    return browser.switchTo().alert().then(function (alert) {
      alert.accept();
      return browser.get(browser.baseUrl);
    });
  });
}

// Returns the contents of the header that displays the "Auto", "TeleOp", and
// "Climb" labels etc.
function getHeadingText() {
  return element(by.css('.header')).getText();
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

describe('The scouting web page', () => {
  it('should: review and submit correct data.', async () => {
    await loadPage();

    expect(await getHeadingText()).toEqual('Team Selection');
    // Just sending "971" to the input fields is insufficient. We need to
    // overwrite the text that is there. If we didn't hit CTRL-A to select all
    // the text, we'd be appending to whatever is there already.
    await element(by.id('team_number')).sendKeys(
        protractor.Key.CONTROL, 'a', protractor.Key.NULL,
        '971');
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Auto');
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('TeleOp');
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Climb');
    await element(by.id('high')).click();
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Defense');
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Review and Submit');
    expect(await getErrorMessage()).toEqual('');

    // Validate Team Selection.
    await expectReviewFieldToBe('Match number', '1');
    await expectReviewFieldToBe('Team number', '971');

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

    // Validate Defense.
    await expectReviewFieldToBe('Defense Played On Rating', '0');
    await expectReviewFieldToBe('Defense Played Rating', '0');

    // TODO(phil): Submit data and make sure it made its way to the database
    // correctly. Right now the /requests/submit/data_scouting endpoint is not
    // implemented.
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
