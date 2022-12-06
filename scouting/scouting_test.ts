import {browser, by, element, protractor} from 'protractor';

const EC = protractor.ExpectedConditions;

// Loads the page (or reloads it) and deals with the "Are you sure you want to
// leave this page" popup.
async function loadPage() {
  await disableAlerts();
  await browser.navigate().refresh();
  expect(await browser.getTitle()).toEqual('FRC971 Scouting Application');
  await disableAlerts();
}

// Disables alert popups. They are extremely tedious to deal with in
// Protractor since they're not angular elements. We achieve this by checking
// an invisible checkbox that's off-screen.
async function disableAlerts() {
  await browser.executeAsyncScript(function (callback) {
    let block_alerts = document.getElementById(
      'block_alerts'
    ) as HTMLInputElement;
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

// Returns the currently displayed error message on the screen. This only
// exists on screens where the web page interacts with the web server.
function getValueOfInputById(id: string) {
  return element(by.id(id)).getAttribute('value');
}

// Asserts that the field on the "Submit and Review" screen has a specific
// value.
function expectReviewFieldToBe(fieldName: string, expectedValue: string) {
  return expectNthReviewFieldToBe(fieldName, 0, expectedValue);
}

// Asserts that the n'th instance of a field on the "Submit and Review"
// screen has a specific value.
async function expectNthReviewFieldToBe(
  fieldName: string,
  n: number,
  expectedValue: string
) {
  expect(
    await element
      .all(by.cssContainingText('li', `${fieldName}:`))
      .get(n)
      .getText()
  ).toEqual(`${fieldName}: ${expectedValue}`);
}

// Sets a text field to the specified value.
function setTextboxByIdTo(id: string, value: string) {
  // Just sending "value" to the input fields is insufficient. We need to
  // overwrite the text that is there. If we didn't hit CTRL-A to select all
  // the text, we'd be appending to whatever is there already.
  return element(by.id(id)).sendKeys(
    protractor.Key.CONTROL,
    'a',
    protractor.Key.NULL,
    value
  );
}

// Moves the nth slider left or right. A positive "adjustBy" value moves the
// slider to the right. A negative value moves the slider to the left.
//
//   negative/left <--- 0 ---> positive/right
async function adjustNthSliderBy(n: number, adjustBy: number) {
  const slider = element.all(by.css('input[type=range]')).get(n);
  const key =
    adjustBy > 0 ? protractor.Key.ARROW_RIGHT : protractor.Key.ARROW_LEFT;
  for (let i = 0; i < Math.abs(adjustBy); i++) {
    await slider.sendKeys(key);
  }
}

function getNthMatchLabel(n: number) {
  return element.all(by.css('.badge')).get(n).getText();
}

describe('The scouting web page', () => {
  beforeAll(async () => {
    await browser.get(browser.baseUrl);
    expect(await browser.getTitle()).toEqual('FRC971 Scouting Application');
    await disableAlerts();

    // Import the match list before running any tests. Ideally this should be
    // run in beforeEach(), but it's not worth doing that at this time. Our
    // tests are basic enough not to require this.
    await element(
      by.cssContainingText('.nav-link', 'Import Match List')
    ).click();
    expect(await getHeadingText()).toEqual('Import Match List');
    await setTextboxByIdTo('year', '2016');
    await setTextboxByIdTo('event_code', 'nytr');
    await element(by.buttonText('Import')).click();

    await browser.wait(
      EC.textToBePresentInElement(
        element(by.css('.progress_message')),
        'Successfully imported match list.'
      )
    );
  });

  it('should: show matches in chronological order.', async () => {
    await loadPage();

    expect(await getNthMatchLabel(0)).toEqual('Quals Match 1');
    expect(await getNthMatchLabel(1)).toEqual('Quals Match 2');
    expect(await getNthMatchLabel(2)).toEqual('Quals Match 3');
    expect(await getNthMatchLabel(9)).toEqual('Quals Match 10');
    expect(await getNthMatchLabel(72)).toEqual('Quarter Final 1 Match 1');
    expect(await getNthMatchLabel(73)).toEqual('Quarter Final 2 Match 1');
    expect(await getNthMatchLabel(74)).toEqual('Quarter Final 3 Match 1');
    expect(await getNthMatchLabel(75)).toEqual('Quarter Final 4 Match 1');
    expect(await getNthMatchLabel(76)).toEqual('Quarter Final 1 Match 2');
    expect(await getNthMatchLabel(82)).toEqual('Semi Final 1 Match 1');
    expect(await getNthMatchLabel(83)).toEqual('Semi Final 2 Match 1');
    expect(await getNthMatchLabel(84)).toEqual('Semi Final 1 Match 2');
    expect(await getNthMatchLabel(85)).toEqual('Semi Final 2 Match 2');
    expect(await getNthMatchLabel(89)).toEqual('Final 1 Match 3');
  });

  it('should: prefill the match information.', async () => {
    await loadPage();

    expect(await getHeadingText()).toEqual('Matches');

    // On the 87th row of matches (index 86) click on the second team
    // (index 1) which resolves to team 5254 in semi final 2 match 3.
    await element
      .all(by.css('button.match-item'))
      .get(86 * 6 + 1)
      .click();

    expect(await getHeadingText()).toEqual('Team Selection');
    expect(await getValueOfInputById('match_number')).toEqual('3');
    expect(await getValueOfInputById('team_number')).toEqual('5254');
    expect(await getValueOfInputById('set_number')).toEqual('2');
    expect(await getValueOfInputById('comp_level')).toEqual('3: sf');
  });

  it('should: error on unknown match.', async () => {
    await loadPage();

    await element(by.cssContainingText('.nav-link', 'Data Entry')).click();

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
      'Failed to find team 971 in match 3 in the schedule.'
    );
  });

  // Make sure that each page on the Entry tab has both "Next" and "Back"
  // buttons. The only screens exempted from this are the first page and the
  // last page.
  it('should: have forwards and backwards buttons.', async () => {
    await loadPage();

    await element(by.cssContainingText('.nav-link', 'Data Entry')).click();

    const expectedOrder = [
      'Team Selection',
      'Auto',
      'TeleOp',
      'Climb',
      'Other',
      'Review and Submit',
    ];

    // Go forward through the screens.
    for (let i = 0; i < expectedOrder.length; i++) {
      expect(await getHeadingText()).toEqual(expectedOrder[i]);
      if (i != expectedOrder.length - 1) {
        await element(by.buttonText('Next')).click();
      }
    }

    // Go backwards through the screens.
    for (let i = 0; i < expectedOrder.length; i++) {
      expect(await getHeadingText()).toEqual(
        expectedOrder[expectedOrder.length - i - 1]
      );
      if (i != expectedOrder.length - 1) {
        await element(by.buttonText('Back')).click();
      }
    }
  });

  it('should: review and submit correct data.', async () => {
    await loadPage();

    await element(by.cssContainingText('.nav-link', 'Data Entry')).click();

    // Submit scouting data for a random team that attended 2016nytr.
    expect(await getHeadingText()).toEqual('Team Selection');
    await setTextboxByIdTo('match_number', '2');
    await setTextboxByIdTo('team_number', '5254');
    await setTextboxByIdTo('set_number', '42');
    await element(by.cssContainingText('option', 'Semi Finals')).click();
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Auto');
    await element(by.id('quadrant3')).click();
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('TeleOp');
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Climb');
    await element(by.id('high')).click();
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Other');
    await adjustNthSliderBy(0, 3);
    await adjustNthSliderBy(1, 1);
    await element(by.id('no_show')).click();
    await element(by.id('mechanically_broke')).click();
    await setTextboxByIdTo('comment', 'A very useful comment here.');
    await element(by.buttonText('Next')).click();

    expect(await getHeadingText()).toEqual('Review and Submit');
    expect(await getErrorMessage()).toEqual('');

    // Validate Team Selection.
    await expectReviewFieldToBe('Match number', '2');
    await expectReviewFieldToBe('Team number', '5254');
    await expectReviewFieldToBe('SetNumber', '42');
    await expectReviewFieldToBe('Comp Level', 'Semi Finals');

    // Validate Auto.
    await expectNthReviewFieldToBe('Upper Shots Made', 0, '0');
    await expectNthReviewFieldToBe('Lower Shots Made', 0, '0');
    await expectNthReviewFieldToBe('Missed Shots', 0, '0');
    await expectReviewFieldToBe('Quadrant', '3');

    // Validate TeleOp.
    await expectNthReviewFieldToBe('Upper Shots Made', 1, '0');
    await expectNthReviewFieldToBe('Lower Shots Made', 1, '0');
    await expectNthReviewFieldToBe('Missed Shots', 1, '0');

    // Validate Climb.
    await expectReviewFieldToBe('Climb Level', 'High');

    // Validate Other.
    await expectReviewFieldToBe('Defense Played On Rating', '3');
    await expectReviewFieldToBe('Defense Played Rating', '1');
    await expectReviewFieldToBe('No show', 'true');
    await expectReviewFieldToBe('Never moved', 'false');
    await expectReviewFieldToBe('Battery died', 'false');
    await expectReviewFieldToBe('Broke (mechanically)', 'true');
    await expectReviewFieldToBe('Comments', 'A very useful comment here.');

    await element(by.buttonText('Submit')).click();
    await browser.wait(
      EC.textToBePresentInElement(element(by.css('.header')), 'Success')
    );

    // TODO(phil): Make sure the data made its way to the database correctly.
  });

  it('should: load all images successfully.', async () => {
    await loadPage();

    await element(by.cssContainingText('.nav-link', 'Data Entry')).click();

    // Get to the Auto display with the field pictures.
    expect(await getHeadingText()).toEqual('Team Selection');
    await element(by.buttonText('Next')).click();
    expect(await getHeadingText()).toEqual('Auto');

    // We expect 2 fully loaded images for each of the orientations.
    // 2 images for the original orientation and 2 images for the flipped orientation.
    for (let i = 0; i < 2; i++) {
      browser
        .executeAsyncScript(function (callback) {
          let images = document.getElementsByTagName('img');
          let numLoaded = 0;
          for (let i = 0; i < images.length; i += 1) {
            if (images[i].naturalWidth > 0) {
              numLoaded += 1;
            }
          }
          callback(numLoaded);
        })
        .then(function (numLoaded) {
          expect(numLoaded).toBe(2);
        });

      await element(by.buttonText('Flip')).click();
    }
  });

  it('should: submit note scouting for multiple teams', async () => {
    // Navigate to Notes Page.
    await loadPage();
    await element(by.cssContainingText('.nav-link', 'Notes')).click();
    expect(await element(by.id('page-title')).getText()).toEqual('Notes');

    // Add first team.
    await setTextboxByIdTo('team_number_notes', '1234');
    await element(by.buttonText('Select')).click();

    // Add note and select keyword for first team.
    expect(await element(by.id('team-key-1')).getText()).toEqual('1234');
    await element(by.id('text-input-1')).sendKeys('Good Driving');
    await element(by.id('Good Driving_0')).click();

    // Navigate to add team selection and add another team.
    await element(by.id('add-team-button')).click();
    await setTextboxByIdTo('team_number_notes', '1235');
    await element(by.buttonText('Select')).click();

    // Add note and select keyword for second team.
    expect(await element(by.id('team-key-2')).getText()).toEqual('1235');
    await element(by.id('text-input-2')).sendKeys('Bad Driving');
    await element(by.id('Bad Driving_1')).click();

    // Submit Notes.
    await element(by.buttonText('Submit')).click();
    expect(await element(by.id('team_number_label')).getText()).toEqual(
      'Team Number'
    );
  });

  it('should: switch note text boxes with keyboard shortcuts', async () => {
    // Navigate to Notes Page.
    await loadPage();
    await element(by.cssContainingText('.nav-link', 'Notes')).click();
    expect(await element(by.id('page-title')).getText()).toEqual('Notes');

    // Add first team.
    await setTextboxByIdTo('team_number_notes', '1234');
    await element(by.buttonText('Select')).click();

    // Add second team.
    await element(by.id('add-team-button')).click();
    await setTextboxByIdTo('team_number_notes', '1235');
    await element(by.buttonText('Select')).click();

    // Add third team.
    await element(by.id('add-team-button')).click();
    await setTextboxByIdTo('team_number_notes', '1236');
    await element(by.buttonText('Select')).click();

    for (let i = 1; i <= 3; i++) {
      // Press Control + i
      // Keyup Control for future actions.
      browser
        .actions()
        .keyDown(protractor.Key.CONTROL)
        .sendKeys(i.toString())
        .keyUp(protractor.Key.CONTROL)
        .perform();

      // Expect text input to be focused.
      expect(
        await browser.driver.switchTo().activeElement().getAttribute('id')
      ).toEqual('text-input-' + i);
    }
  });
});
