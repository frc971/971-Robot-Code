import {Component, HostListener} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '@org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {RequestNotes2025ForTeam} from '@org_frc971/scouting/webserver/requests/messages/request_notes_2025_for_team_generated';
import {
  Note2025 as NoteFb,
  RequestNotes2025ForTeamResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_notes_2025_for_team_response_generated';
import {SubmitNotes2025} from '@org_frc971/scouting/webserver/requests/messages/submit_notes_2025_generated';
import {SubmitNotes2025Response} from '@org_frc971/scouting/webserver/requests/messages/submit_notes_2025_response_generated';

/*
For new games, the keywords being used will likely need to be updated.
To update the keywords complete the following: 
  1) Update the Keywords Interface and KEYWORD_CHECKBOX_LABELS in notes.component.ts
    The keys of Keywords and KEYWORD_CHECKBOX_LABELS should match.
  2) In notes.component.ts, update the setTeamNumber() method with the new keywords.
  3) Add/Edit the new keywords in /scouting/webserver/requests/messages/submit_notes.fbs.
  4) In notes.component.ts, update the submitData() method with the newKeywords 
    so that it matches the updated flatbuffer
  5) In db.go, update the NotesData struct and the 
    AddNotes method with the new keywords        
  6) In db_test.go update the TestNotes method so the test uses the keywords
  7) Update the submitNoteScoutingHandler in requests.go with the new keywords
  8) Finally, update the corresponding test in requests_test.go (TestSubmitNotes)
  
  Note: If you change the number of keywords you might need to 
    update how they are displayed in notes.ng.html 
*/

// TeamSelection: Display form to add a team to the teams being scouted.
// Data: Display the note textbox and keyword selection form
// for all the teams being scouted.
type Section = 'TeamSelection' | 'Data';

const COMP_LEVELS = ['qm', 'ef', 'qf', 'sf', 'f'] as const;
type CompLevel = typeof COMP_LEVELS[number];

// TODO(phil): Deduplicate with match_list.component.ts.
const COMP_LEVEL_LABELS: Record<CompLevel, string> = {
  qm: 'Qualifications',
  ef: 'Eighth Finals',
  qf: 'Quarter Finals',
  sf: 'Semi Finals',
  f: 'Finals',
};

// Every keyword checkbox corresponds to a boolean.
// If the boolean is True, the checkbox is selected
// and the note scout saw that the robot being scouted
// displayed said property (ex. Driving really well -> goodDriving)
interface Keywords {
  goodDriving: boolean;
  badDriving: boolean;
  coralGroundIntake: boolean;
  coralHpIntake: boolean;
  algaeGroundIntake: boolean;
  solidAlgaeShooting: boolean;
  sketchyAlgaeShooting: boolean;
  solidCoralShooting: boolean;
  sketchyCoralShooting: boolean;
  shuffleCoral: boolean;
  reefIntake: boolean;
  penalties: boolean;
  goodDefense: boolean;
  badDefense: boolean;
  easilyDefended: boolean;
  noShow: boolean;
}

interface Input {
  compCode: string;
  teamNumber: string;
  notesData: string;
  keywordsData: Keywords;
  matchNumber: number;
  setNumber: number;
  compLevel: string;
}

const KEYWORD_CHECKBOX_LABELS = {
  goodDriving: 'Good Driving',
  badDriving: 'Bad Driving',
  coralGroundIntake: 'Coral Ground Intake',
  coralHpIntake: 'Coral HP Intake',
  algaeGroundIntake: 'Algae Ground Intake',
  solidAlgaeShooting: 'Solid Algae Shooting',
  sketchyAlgaeShooting: 'Sketchy Algae Shooting',
  solidCoralShooting: 'Solid Coral Shooting',
  sketchyCoralShooting: 'Sketchy Coral Shooting',
  shuffleCoral: 'Shuffle Coral',
  reefIntake: 'Reef Intake',
  penalties: 'Penalties',
  goodDefense: 'Good Defense',
  badDefense: 'Bad Defense',
  easilyDefended: 'Easily Defended',
  noShow: 'No Show',
} as const;

@Component({
  selector: 'frc971-notes',
  templateUrl: './notes.ng.html',
  styleUrls: ['../app/common.css', './notes.component.css'],
})
export class Notes {
  // Re-export KEYWORD_CHECKBOX_LABELS so that we can
  // use it in the checkbox properties.
  readonly COMP_LEVELS = COMP_LEVELS;
  readonly COMP_LEVEL_LABELS = COMP_LEVEL_LABELS;
  readonly KEYWORD_CHECKBOX_LABELS = KEYWORD_CHECKBOX_LABELS;

  // Necessary in order to iterate the keys of KEYWORD_CHECKBOX_LABELS.
  Object = Object;

  section: Section = 'TeamSelection';

  errorMessage = '';
  teamNumber: string = '1';
  compCode: string = '2016nytr';
  matchNumber: number = 1;
  setNumber: number = 1;
  compLevel: CompLevel = 'qm';

  // Data inputted by user is stored in this array.
  // Includes the team number, notes, and keyword selection.
  newData: Input[] = [];

  // Keyboard shortcuts to switch between text areas.
  // Listens for Ctrl + number and focuses on the
  // corresponding textbox.
  // More Info: https://angular.io/api/core/HostListener

  @HostListener('window:keyup', ['$event'])
  onEvent(event: KeyboardEvent) {
    if (event.ctrlKey) {
      if (event.code.includes('Digit')) {
        this.handleFocus(event.key);
      }
    }
  }

  handleFocus(digit: string) {
    let textArea = <HTMLInputElement>(
      document.getElementById('text-input-' + digit)
    );
    if (textArea != null) {
      textArea.focus();
    }
  }

  setTeamData() {
    let data: Input = {
      compCode: this.compCode,

      teamNumber: this.teamNumber,
      notesData:
        'Match ' +
        this.matchNumber +
        ' Set ' +
        this.setNumber +
        ' ' +
        COMP_LEVEL_LABELS[this.compLevel] +
        ' \nAuto: \nTeleop: \nEndgame: ',
      keywordsData: {
        goodDriving: false,
        badDriving: false,
        coralGroundIntake: false,
        coralHpIntake: false,
        algaeGroundIntake: false,
        solidAlgaeShooting: false,
        sketchyAlgaeShooting: false,
        solidCoralShooting: false,
        sketchyCoralShooting: false,
        shuffleCoral: false,
        reefIntake: false,
        penalties: false,
        goodDefense: false,
        badDefense: false,
        easilyDefended: false,
        noShow: false,
      },
      matchNumber: this.matchNumber,
      setNumber: this.setNumber,
      compLevel: this.compLevel,
    };

    this.newData.push(data);
    this.section = 'Data';
  }

  removeTeam(index: number) {
    this.newData.splice(index, 1);
    if (this.newData.length == 0) {
      this.section = 'TeamSelection';
    } else {
      this.section = 'Data';
    }
  }

  addTeam() {
    this.section = 'TeamSelection';
  }

  async submitData() {
    for (let i = 0; i < this.newData.length; i++) {
      const builder = new Builder();
      const dataFb = builder.createString(this.newData[i].notesData);

      const teamNumber = builder.createString(this.newData[i].teamNumber);
      const compLevel = builder.createString(this.newData[i].compLevel);
      const compCode = builder.createString(this.newData[i].compCode);

      builder.finish(
        SubmitNotes2025.createSubmitNotes2025(
          builder,
          compCode,
          teamNumber,
          this.newData[i].matchNumber,
          this.newData[i].setNumber,
          compLevel,

          dataFb,
          this.newData[i].keywordsData.goodDriving,
          this.newData[i].keywordsData.badDriving,
          this.newData[i].keywordsData.coralGroundIntake,
          this.newData[i].keywordsData.coralHpIntake,
          this.newData[i].keywordsData.algaeGroundIntake,
          this.newData[i].keywordsData.solidAlgaeShooting,
          this.newData[i].keywordsData.sketchyAlgaeShooting,
          this.newData[i].keywordsData.solidCoralShooting,
          this.newData[i].keywordsData.sketchyCoralShooting,
          this.newData[i].keywordsData.shuffleCoral,
          this.newData[i].keywordsData.reefIntake,
          this.newData[i].keywordsData.penalties,
          this.newData[i].keywordsData.goodDefense,
          this.newData[i].keywordsData.badDefense,
          this.newData[i].keywordsData.easilyDefended,
          this.newData[i].keywordsData.noShow
        )
      );

      const buffer = builder.asUint8Array();
      const res = await fetch('/requests/submit/submit_notes_2025', {
        method: 'POST',
        body: buffer,
      });

      if (!res.ok) {
        const resBuffer = await res.arrayBuffer();
        const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
        const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);
        const errorMessage = parsedResponse.errorMessage();
        this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
      }
    }

    this.newData = [];
    this.errorMessage = '';
    this.section = 'TeamSelection';
  }

  labelToId(label: String): String {
    return label.replaceAll(' ', '_').toLowerCase();
  }
}
