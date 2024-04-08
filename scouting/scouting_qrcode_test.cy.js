/// <reference types="cypress" />

// On the 3rd row of matches (index 2) click on the third team
// (index 2) which resolves to team 333 in quals match 3.
const QUALS_MATCH_3_TEAM_333 = 2 * 6 + 2;

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

// Wrapper around cy.exec() because it truncates the output of the subprocess
// if it fails. This is a work around to manually print the full error on the
// console if a failure happends.
function exec(command) {
  cy.exec(command, {failOnNonZeroExit: false}).then((result) => {
    if (result.code) {
      throw new Error(`Execution of "${command}" failed
      Exit code: ${result.code}
      Stdout:\n${result.stdout}
      Stderr:\n${result.stderr}`);
    }
  });
}

// Prepares data entry so that we _could_ hit Submit.
//
// Options:
//  matchButtonKey: The index into the big matchlist table that we want to
//    click on to start the data entry.
//  teamNumber: The team number that matches the button that we click on as
//    specified by `matchButtonKey`.
//
// TODO(phil): Deduplicate with scouting_test.cy.js.
function prepareDataScouting(options) {
  const {matchButtonKey = SEMI_FINAL_2_MATCH_3_TEAM_5254, teamNumber = 5254} =
    options;

  // Click on a random team in the Match list. The exact details here are not
  // important, but we need to know what they are. This could as well be any
  // other team from any other match.
  cy.get('button.match-item').eq(matchButtonKey).click();

  // Select Starting Position.
  headerShouldBe(teamNumber + ' Init ');
  cy.get('[type="radio"]').first().check();
  clickButton('Start Match');

  // Pick and Place Note in Auto.
  clickButton('NOTE');
  clickButton('AMP');

  // Pick and Place Cube in Teleop.
  clickButton('Start Teleop');
  clickButton('NOTE');
  clickButton('SPEAKER AMPLIFIED');

  // Generate some extra actions so that we are guaranteed to have at least 2
  // QR codes.
  for (let i = 0; i < 5; i++) {
    clickButton('NOTE');
    clickButton('AMP');
  }

  // Robot dead and revive.
  clickButton('DEAD');
  clickButton('Revive');

  // Endgame.
  clickButton('Endgame');
  cy.contains(/Harmony/).click();

  clickButton('End Match');
  headerShouldBe(teamNumber + ' Review and Submit ');
  cy.get('#review_data li')
    .eq(0)
    .should('have.text', ' Started match at position 1 ');
  cy.get('#review_data li').eq(1).should('have.text', ' Picked up Note ');
  cy.get('#review_data li')
    .last()
    .should(
      'have.text',
      ' Ended Match; stageType: kHARMONY, trapNote: false, spotlight: false '
    );
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
  // This test collects some scouting data and then generates the corresponding
  // QR codes. The test takes screenshots of those QR codes. The QR codes get
  // turned into a little video file for the browser to use as a fake camera
  // input. The test then switches to the Scan tab to scan the QR codes from
  // the "camera". We then make sure that the data gets submitted.
  it('should: be able to generate and scan QR codes.', () => {
    prepareDataScouting({
      matchButtonKey: QUALS_MATCH_3_TEAM_333,
      teamNumber: 333,
    });
    clickButton('Create QR Code');
    headerShouldBe('333 QR Code ');

    cy.get('#qr_code_piece_size').select('150');

    // Go into a mobile-phone view so that we can guarantee that the QR code is
    // visible.
    cy.viewport(400, 660);

    cy.get('.qrcode-buttons > li > a')
      .should('have.length.at.least', 4)
      .each(($button, index, $buttons) => {
        if (index == 0 || index + 1 == $buttons.length) {
          // Skip the "Previous" and "Next" buttons.
          return;
        }
        // Click on the button to switch to that particular QR code.
        // We use force:true here because without bootstrap (inside the
        // sandbox) the buttons overlap one another a bit.
        cy.wrap($button).click({force: true});
        cy.get('div.qrcode').screenshot(`qrcode_${index}_screenshot`);
      });

    exec('./testing/camera_simulator/camera_simulator_/camera_simulator');

    switchToTab('Scan');

    // Since we cannot reliably predict how long it will take to scan all the
    // QR codes, we use a really long timeout here.
    cy.get('.progress_message', {timeout: 80000}).should('contain', 'Success!');

    // Now that the data is submitted, the button should be disabled.
    switchToTab('Match List');
    cy.get('button.match-item')
      .eq(QUALS_MATCH_3_TEAM_333)
      .should('be.disabled');
  });
});
