/// <reference types="cypress" />

// On the 87th row of matches (index 86) click on the second team
// (index 1) which resolves to team 5254 in semi final 2 match 3.
const SEMI_FINAL_2_MATCH_3_TEAM_5254 = 86 * 6 + 1;

// On the 1st row of matches (index 0) click on the fourth team
// (index 3) which resolves to team 3990 in quals match 1.
const QUALS_MATCH_1_TEAM_3990 = 0 * 6 + 3;

// On the 2st row of matches (index 1) click on the fourth team
// (index 3) which resolves to team 4481 in quals match 1.
const QUALS_MATCH_2_TEAM_4481 = 1 * 6 + 3;

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

function submitDataScouting(
  matchButtonKey = SEMI_FINAL_2_MATCH_3_TEAM_5254,
  teamNumber = 5254
) {
  // Click on a random team in the Match list. The exact details here are not
  // important, but we need to know what they are. This could as well be any
  // other team from any other match.
  cy.get('button.match-item').eq(matchButtonKey).click();

  // Select Starting Position.
  headerShouldBe('2016nytr Team ' + teamNumber + ' Init ');
  cy.get('[type="radio"]').first().check();
  clickButton('Start Match');

  // Pick and Place Coral in Auto.
  clickButton('PICKUP CORAL');
  clickButton('SCORE CORAL');
  clickButton('L2');

  // Pick and Place Algae in Teleop.
  clickButton('Start Teleop');
  clickButton('PICKUP ALGAE');
  clickButton('SCORE ALGAE');
  clickButton('Processor');

  // Robot dead and revive.
  clickButton('DEAD');
  clickButton('Revive');

  // Endgame.
  clickButton('Endgame');
  cy.contains(/Deep Cage/).click();

  clickButton('End Match');

  clickButton('UNDO');
  clickButton('End Match');

  headerShouldBe('2016nytr Team ' + teamNumber + ' Review and Submit ');

  cy.get('#review_data li')
    .eq(0)
    .should('have.text', ' Started match at position 1 ');
  cy.get('#review_data li').eq(1).should('have.text', ' Coral placed at kL2 ');
  cy.get('#review_data li').last().should('have.text', ' Defense: false ');
  // Ensure that the penalties action is only submitted once.
  cy.get('#review_data li:contains("Penalties")').its('length').should('eq', 1);

  clickButton('Submit');
  headerShouldBe('2016nytr Team ' + teamNumber + ' End ');
}
function visit(path) {
  cy.visit(path, {
    onBeforeLoad(win) {
      // The service worker seems to interfere with Cypress somehow. There
      // doesn't seem to be a proper fix for this issue. Work around it with
      // this hack that disables the service worker.
      // https://github.com/cypress-io/cypress/issues/16192#issuecomment-870421667
      // https://github.com/cypress-io/cypress/issues/702#issuecomment-587127275
      delete win.navigator.__proto__.serviceWorker;
    },
  });
}

before(() => {
  visit('/');
  disableAlerts();
  cy.title().should('eq', 'FRC971 Scouting Application');
});

beforeEach(() => {
  visit('/');
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

  it('should: prevent users from enter invalid match information.', () => {
    switchToTab('Entry');
    headerShouldBe(' Team Selection ');

    setInputTo('#match_number', '1');
    setInputTo('#team_number', '5254');
    setInputTo('#set_number', '1');
    cy.get('#comp_level').select('Qualifications');

    cy.contains('button', 'Next').should('be.disabled');
  });

  it('should: allow users to scout non-existent matches when pre-scouting.', () => {
    switchToTab('Entry');
    headerShouldBe(' Team Selection ');
    setInputTo('#team_number', '1');

    // The default team information should be invalid.
    cy.contains('button', 'Next').should('be.disabled');

    // Click the checkmark to designate this as pre-scouting.
    // We should now be able to continue scouting.
    cy.get('#pre_scouting').click();
    clickButton('Next');
    headerShouldBe('2016nytr Team 1 Init ');
  });

  it('should: allow users to scout practice matches.', () => {
    switchToTab('Entry');
    headerShouldBe(' Team Selection ');
    setInputTo('#team_number', '1');

    // The default team information should be invalid.
    cy.contains('button', 'Next').should('be.disabled');

    // Click the checkmark to designate this as pre-scouting.
    // We should now be able to continue scouting.
    cy.get('#practice_match').click();
    clickButton('Next');
    headerShouldBe('2016nytr Team 1 Init ');
  });

  it('should: let users enter match information manually.', () => {
    switchToTab('Entry');
    headerShouldBe(' Team Selection ');

    setInputTo('#match_number', '3');
    setInputTo('#team_number', '5254');
    setInputTo('#set_number', '2');
    cy.get('#comp_level').select('Semi Finals');

    clickButton('Next');

    headerShouldBe('2016nytr Team 5254 Init ');
  });

  //TODO(FILIP): Verify last action when the last action header gets added.
  it('should: be able to submit data scouting.', () => {
    submitDataScouting();

    // Now that the data is submitted, the button should be disabled.
    switchToTab('Match List');
    cy.get('button.match-item')
      .eq(SEMI_FINAL_2_MATCH_3_TEAM_5254)
      .should('be.disabled');
  });

  it('should: be able to delete data scouting entry', () => {
    // Submit data to delete.
    submitDataScouting(QUALS_MATCH_2_TEAM_4481, 4481);

    switchToTab('View');

    cy.get('[data-bs-toggle="dropdown"]').click();
    cy.get('[id="stats_source_dropdown"]').click();

    // Check that table contains data.
    cy.get('table.table tbody td').should('contain', '4481');

    // Find and click the delete button for the row containing team 4481.
    cy.get('table.table tbody td')
      .contains('4481')
      .parent()
      .find('[id^="delete_button_"]')
      .click();
    cy.on('window:confirm', () => true);

    // Check that deleted data is not in table.
    cy.get('table.table tbody').should('not.contain', '4481');
  });

  it('should: be able to return to correct screen with undo for pick and place.', () => {
    cy.get('button.match-item').eq(QUALS_MATCH_1_TEAM_3990).click();

    // Select Starting Position.
    cy.get('[type="radio"]').first().check();
    clickButton('Start Match');

    // Pick up algae.
    clickButton('PICKUP ALGAE');

    // Undo that pick up.
    clickButton('UNDO');

    // User should be back on pickup screen.
    headerShouldBe('2016nytr Team 3990 Pickup ');

    // Check the same thing but for undoing place.
    clickButton('PICKUP ALGAE');
    clickButton('SCORE ALGAE');
    clickButton('Net');
    clickButton('UNDO');
    headerShouldBe('2016nytr Team 3990 Place Algae ');
  });

  it('should: submit note scouting for multiple teams', () => {
    // Navigate to Notes Page.
    switchToTab('Notes');
    headerShouldBe('Notes');

    // Add first team.
    setInputTo('#team_number_notes', '1234');
    clickButton('Select');

    // Add note and select keyword for first team.
    cy.get('#team-key-1').should(
      'have.text',
      ' Team: 1234, Comp Code: 2016nytr '
    );
    setInputTo('#text-input-1', 'Good Driving');
    cy.get('#good_driving_0').click();

    // Navigate to add team selection and add another team.
    clickButton('Add team');
    setInputTo('#comp_code_selection', '0');
    setInputTo('#team_number_notes', '1235');
    setInputTo('#match_number_notes', 1);
    setInputTo('#set_number_notes', 2);
    setInputTo('#comp_level_notes', 'qm');
    clickButton('Select');

    // Add note and select keyword for second team.
    cy.get('#team-key-2').should(
      'have.text',
      ' Team: 1235, Comp Code: 2016nytr '
    );
    setInputTo('#text-input-2', 'Solid Algae Shooting');
    cy.get('#solid_algae_shooting_1').click();

    // Submit Notes.
    clickButton('Submit');
    cy.get('#team_number_label').should('have.text', ' Team Number ');
  });

  it('should: switch note text boxes with keyboard shortcuts', () => {
    // Navigate to Notes Page.
    switchToTab('Notes');
    headerShouldBe('Notes');

    // Add first team.
    setInputTo('#comp_code_selection', '0');
    setInputTo('#team_number_notes', '1234');
    setInputTo('#match_number_notes', 1);
    setInputTo('#set_number_notes', 2);
    setInputTo('#comp_level_notes', 'qm');
    clickButton('Select');

    // Add second team.
    clickButton('Add team');
    setInputTo('#comp_code_selection', '0');
    setInputTo('#team_number_notes', '1235');
    setInputTo('#match_number_notes', 1);
    setInputTo('#set_number_notes', 2);
    setInputTo('#comp_level_notes', 'qm');
    clickButton('Select');

    // Add third team.
    clickButton('Add team');
    setInputTo('#team_number_notes', '1236');
    setInputTo('#match_number_notes', 1);
    setInputTo('#set_number_notes', 2);
    setInputTo('#comp_level_notes', 'qm');
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
    switchToTab('Ranking');
    headerShouldBe('Ranking');

    // Input comp code match and team numbers.
    setInputTo('#comp_code_selection', 'none');
    setInputTo('#match_number_selection', '11');
    setInputTo('#team_input_0', '123');
    setInputTo('#team_input_1', '456');
    setInputTo('#team_input_2', '789');
    setInputTo('#team_input_3', '1011');
    setInputTo('#team_input_4', '1213');
    setInputTo('#team_input_5', '1415');
    clickButton('Select');

    // Verify match and team key input.
    cy.get('#match_number_heading').should('have.text', 'Match #11');
    cy.get('#team_key_label_0').should('have.text', ' 123 ');
    cy.get('#team_key_label_1').should('have.text', ' 456 ');
    cy.get('#team_key_label_2').should('have.text', ' 789 ');
    cy.get('#team_key_label_3').should('have.text', ' 1011 ');
    cy.get('#team_key_label_4').should('have.text', ' 1213 ');
    cy.get('#team_key_label_5').should('have.text', ' 1415 ');

    // Rank drivers
    setInputTo('#rank_0', 2);
    setInputTo('#rank_1', 1);
    setInputTo('#rank_2', 5);
    setInputTo('#rank_3', 4);
    setInputTo('#rank_4', 3);
    setInputTo('#rank_5', 6);

    // Submit.
    clickButton('Submit');
  });
});
