/// <reference types="cypress" />

function disableAlerts() {
  cy.get('#block_alerts').check({force: true}).should('be.checked');
}

function switchToTab(tabName) {
  cy.contains('.nav-link', tabName).click();
}

function headerShouldBe(text) {
  cy.get('.header').should('have.text', text);
}

function clickButton(buttonName) {
  cy.contains('button', buttonName).click();
}

function setInputTo(fieldSelector, value) {
  cy.get(fieldSelector).type('{selectAll}' + value);
}

// Moves the nth slider left or right. A positive "adjustBy" value moves the
// slider to the right. A negative value moves the slider to the left.
//
//   negative/left <--- 0 ---> positive/right
function adjustNthSliderBy(n, adjustBy) {
  let element = cy.get('input[type=range]').eq(n);
  element.scrollIntoView();
  element.invoke('val').then((currentValue) => {
    // We need to query for the slider here again because `invoke('val')` above
    // somehow invalidates further calls to `val`.
    cy.get('input[type=range]')
      .eq(n)
      .invoke('val', currentValue + adjustBy)
      .trigger('change');
  });
}

// Asserts that the field on the "Submit and Review" screen has a specific
// value.
function expectReviewFieldToBe(fieldName, expectedValue) {
  expectNthReviewFieldToBe(fieldName, 0, expectedValue);
}

// Asserts that the n'th instance of a field on the "Submit and Review"
// screen has a specific value.
function expectNthReviewFieldToBe(fieldName, n, expectedValue) {
  getNthReviewField(fieldName, n).should(
    'have.text',
    `${fieldName}: ${expectedValue}`
  );
}

function getNthReviewField(fieldName, n) {
  let element = cy.get('li').filter(`:contains("${fieldName}: ")`).eq(n);
  element.scrollIntoView();
  return element;
}

before(() => {
  cy.visit('/');
  disableAlerts();
  cy.title().should('eq', 'FRC971 Scouting Application');
});

beforeEach(() => {
  cy.visit('/');
  disableAlerts();
});

describe('Scouting app tests', () => {
  it('should: show matches in chronological order.', () => {
    headerShouldBe('Matches');
    cy.get('.badge').eq(0).contains('Quals Match 1');
    cy.get('.badge').eq(1).contains('Quals Match 2');
    cy.get('.badge').eq(2).contains('Quals Match 3');
    cy.get('.badge').eq(9).contains('Quals Match 10');
    cy.get('.badge').eq(72).contains('Quarter Final 1 Match 1');
    cy.get('.badge').eq(73).contains('Quarter Final 2 Match 1');
    cy.get('.badge').eq(74).contains('Quarter Final 3 Match 1');
    cy.get('.badge').eq(75).contains('Quarter Final 4 Match 1');
    cy.get('.badge').eq(76).contains('Quarter Final 1 Match 2');
    cy.get('.badge').eq(82).contains('Semi Final 1 Match 1');
    cy.get('.badge').eq(83).contains('Semi Final 2 Match 1');
    cy.get('.badge').eq(84).contains('Semi Final 1 Match 2');
    cy.get('.badge').eq(85).contains('Semi Final 2 Match 2');
    cy.get('.badge').eq(89).contains('Final 1 Match 3');
  });

  it('should: prefill the match information.', () => {
    headerShouldBe('Matches');

    // On the 87th row of matches (index 86) click on the second team
    // (index 1) which resolves to team 5254 in semi final 2 match 3.
    cy.get('button.match-item')
      .eq(86 * 6 + 1)
      .click();

    headerShouldBe('Team Selection');
    cy.get('#match_number').should('have.value', '3');
    cy.get('#team_number').should('have.value', '5254');
    cy.get('#set_number').should('have.value', '2');
    cy.get('#comp_level').should('have.value', '3: sf');
  });

  //TODO(FILIP): Rewrite tests for the new scouting interface.
  /*
  it('should: error on unknown match.', () => {
    switchToTab('Data Entry');
    headerShouldBe('Team Selection');

    // Pick a match that doesn't exist in the 2016nytr match list.
    setInputTo('#match_number', '3');
    setInputTo('#team_number', '971');

    // Click Next until we get to the submit screen.
    for (let i = 0; i < 5; i++) {
      clickButton('Next');
    }
    headerShouldBe('Review and Submit');

    // Attempt to submit and validate the error.
    clickButton('Submit');
    cy.get('.error_message').contains(
      'Failed to find team 971 in match 3 in the schedule.'
    );
  });

  // Make sure that each page on the Entry tab has both "Next" and "Back"
  // buttons. The only screens exempted from this are the first page and the
  // last page.
  it('should: have forwards and backwards buttons.', () => {
    switchToTab('Data Entry');

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
      headerShouldBe(expectedOrder[i]);
      if (i != expectedOrder.length - 1) {
        clickButton('Next');
      }
    }

    // Go backwards through the screens.
    for (let i = 0; i < expectedOrder.length; i++) {
      headerShouldBe(expectedOrder[expectedOrder.length - i - 1]);
      if (i != expectedOrder.length - 1) {
        clickButton('Back');
      }
    }
  });
  

  it('should: review and submit correct data.', () => {
    switchToTab('Data Entry');

    // Submit scouting data for a random team that attended 2016nytr.
    headerShouldBe('Team Selection');
    setInputTo('#match_number', '2');
    setInputTo('#team_number', '5254');
    setInputTo('#set_number', '42');
    cy.get('#comp_level').select('Semi Finals');
    clickButton('Next');

    headerShouldBe('Auto');
    cy.get('#quadrant3').check();
    clickButton('Next');

    headerShouldBe('TeleOp');
    clickButton('Next');

    headerShouldBe('Climb');
    cy.get('#high').check();
    clickButton('Next');

    headerShouldBe('Other');
    adjustNthSliderBy(0, 3);
    adjustNthSliderBy(1, 1);
    cy.get('#no_show').check();
    cy.get('#mechanically_broke').check();
    setInputTo('#comment', 'A very useful comment here.');
    clickButton('Next');

    headerShouldBe('Review and Submit');
    cy.get('.error_message').should('have.text', '');

    // Validate Team Selection.
    expectReviewFieldToBe('Match number', '2');
    expectReviewFieldToBe('Team number', '5254');
    expectReviewFieldToBe('SetNumber', '42');
    expectReviewFieldToBe('Comp Level', 'Semi Finals');

    // Validate Auto.
    expectNthReviewFieldToBe('Upper Shots Made', 0, '0');
    expectNthReviewFieldToBe('Lower Shots Made', 0, '0');
    expectNthReviewFieldToBe('Missed Shots', 0, '0');
    expectReviewFieldToBe('Quadrant', '3');

    // Validate TeleOp.
    expectNthReviewFieldToBe('Upper Shots Made', 1, '0');
    expectNthReviewFieldToBe('Lower Shots Made', 1, '0');
    expectNthReviewFieldToBe('Missed Shots', 1, '0');

    // Validate Climb.
    expectReviewFieldToBe('Climb Level', 'High');

    // Validate Other.
    expectReviewFieldToBe('Defense Played On Rating', '3');
    expectReviewFieldToBe('Defense Played Rating', '1');
    expectReviewFieldToBe('No show', 'true');
    expectReviewFieldToBe('Never moved', 'false');
    expectReviewFieldToBe('Battery died', 'false');
    expectReviewFieldToBe('Broke (mechanically)', 'true');
    expectReviewFieldToBe('Comments', 'A very useful comment here.');

    clickButton('Submit');
    headerShouldBe('Success');
  });

  */

  it('should: submit note scouting for multiple teams', () => {
    // Navigate to Notes Page.
    switchToTab('Notes');
    headerShouldBe('Notes');

    // Add first team.
    setInputTo('#team_number_notes', '1234');
    clickButton('Select');

    // Add note and select keyword for first team.
    cy.get('#team-key-1').should('have.text', '1234');
    setInputTo('#text-input-1', 'Good Driving');
    cy.get('#good_driving_0').click();

    // Navigate to add team selection and add another team.
    clickButton('Add team');
    setInputTo('#team_number_notes', '1235');
    clickButton('Select');

    // Add note and select keyword for second team.
    cy.get('#team-key-2').should('have.text', '1235');
    setInputTo('#text-input-2', 'Bad Driving');
    cy.get('#bad_driving_1').click();

    // Submit Notes.
    clickButton('Submit');
    cy.get('#team_number_label').should('have.text', ' Team Number ');
  });

  it('should: switch note text boxes with keyboard shortcuts', () => {
    // Navigate to Notes Page.
    switchToTab('Notes');
    headerShouldBe('Notes');

    // Add first team.
    setInputTo('#team_number_notes', '1234');
    clickButton('Select');

    // Add second team.
    clickButton('Add team');
    setInputTo('#team_number_notes', '1235');
    clickButton('Select');

    // Add third team.
    clickButton('Add team');
    setInputTo('#team_number_notes', '1236');
    clickButton('Select');

    for (let i = 1; i <= 3; i++) {
      // Press Control + i
      cy.get('body').type(`{ctrl}${i}`);

      // Expect text input to be focused.
      cy.focused().then(($element) => {
        expect($element).to.have.id(`text-input-${i}`);
      });
    }
  });

  it('should: submit driver ranking', () => {
    // Navigate to Driver Ranking Page.
    switchToTab('Driver Ranking');
    headerShouldBe('Driver Ranking');

    // Input match and team numbers.
    setInputTo('#match_number_selection', '11');
    setInputTo('#team_input_0', '123');
    setInputTo('#team_input_1', '456');
    setInputTo('#team_input_2', '789');
    clickButton('Select');

    // Verify match and team key input.
    cy.get('#match_number_heading').should('have.text', 'Match #11');
    cy.get('#team_key_label_0').should('have.text', ' 123 ');
    cy.get('#team_key_label_1').should('have.text', ' 456 ');
    cy.get('#team_key_label_2').should('have.text', ' 789 ');

    // Rank teams.
    cy.get('#up_button_2').click();
    cy.get('#down_button_0').click();

    // Verify ranking change.
    cy.get('#team_key_label_0').should('have.text', ' 789 ');
    cy.get('#team_key_label_1').should('have.text', ' 123 ');
    cy.get('#team_key_label_2').should('have.text', ' 456 ');

    // Submit.
    clickButton('Submit');
  });
});
