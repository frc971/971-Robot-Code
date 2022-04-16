import {
  Component,
  ElementRef,
  EventEmitter,
  Input,
  OnInit,
  Output,
  ViewChild,
} from '@angular/core';
import {FormsModule} from '@angular/forms';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from 'org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {
  ClimbLevel,
  SubmitDataScouting,
} from 'org_frc971/scouting/webserver/requests/messages/submit_data_scouting_generated';
import {SubmitDataScoutingResponse} from 'org_frc971/scouting/webserver/requests/messages/submit_data_scouting_response_generated';

type Section =
  | 'Team Selection'
  | 'Auto'
  | 'TeleOp'
  | 'Climb'
  | 'Other'
  | 'Review and Submit'
  | 'Success';

// TODO(phil): Deduplicate with match_list.component.ts.
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

@Component({
  selector: 'app-entry',
  templateUrl: './entry.ng.html',
  styleUrls: ['../common.css', './entry.component.css'],
})
export class EntryComponent {
  // Re-export the type here so that we can use it in the `[value]` attribute
  // of radio buttons.
  readonly ClimbLevel = ClimbLevel;
  readonly COMP_LEVELS = COMP_LEVELS;
  readonly COMP_LEVEL_LABELS = COMP_LEVEL_LABELS;

  section: Section = 'Team Selection';
  @Output() switchTabsEvent = new EventEmitter<string>();
  @Input() matchNumber: number = 1;
  @Input() teamNumber: number = 1;
  @Input() setNumber: number = 1;
  @Input() compLevel: CompLevel = 'qm';
  autoUpperShotsMade: number = 0;
  autoLowerShotsMade: number = 0;
  autoShotsMissed: number = 0;
  teleUpperShotsMade: number = 0;
  teleLowerShotsMade: number = 0;
  teleShotsMissed: number = 0;
  defensePlayedOnScore: number = 0;
  defensePlayedScore: number = 0;
  level: ClimbLevel = ClimbLevel.NoAttempt;
  ball1: boolean = false;
  ball2: boolean = false;
  ball3: boolean = false;
  ball4: boolean = false;
  ball5: boolean = false;
  quadrant: number = 1;
  errorMessage: string = '';
  noShow: boolean = false;
  neverMoved: boolean = false;
  batteryDied: boolean = false;
  mechanicallyBroke: boolean = false;
  lostComs: boolean = false;
  comment: string = '';

  @ViewChild('header') header: ElementRef;

  nextSection() {
    if (this.section === 'Team Selection') {
      this.section = 'Auto';
    } else if (this.section === 'Auto') {
      this.section = 'TeleOp';
    } else if (this.section === 'TeleOp') {
      this.section = 'Climb';
    } else if (this.section === 'Climb') {
      this.section = 'Other';
    } else if (this.section === 'Other') {
      this.section = 'Review and Submit';
    } else if (this.section === 'Review and Submit') {
      this.submitDataScouting();
      return;
    } else if (this.section === 'Success') {
      this.switchTabsEvent.emit('MatchList');
      return;
    }
    // Scroll back to the top so that we can be sure the user sees the
    // entire next screen. Otherwise it's easy to overlook input fields.
    this.scrollToTop();
  }

  prevSection() {
    if (this.section === 'Auto') {
      this.section = 'Team Selection';
    } else if (this.section === 'TeleOp') {
      this.section = 'Auto';
    } else if (this.section === 'Climb') {
      this.section = 'TeleOp';
    } else if (this.section === 'Other') {
      this.section = 'Climb';
    } else if (this.section === 'Review and Submit') {
      this.section = 'Other';
    }
    // Scroll back to the top so that we can be sure the user sees the
    // entire previous screen. Otherwise it's easy to overlook input
    // fields.
    this.scrollToTop();
  }

  private scrollToTop() {
    this.header.nativeElement.scrollIntoView();
  }

  async submitDataScouting() {
    this.errorMessage = '';

    const builder = new Builder();
    const compLevel = builder.createString(this.compLevel);
    const comment = builder.createString(this.comment);
    SubmitDataScouting.startSubmitDataScouting(builder);
    SubmitDataScouting.addTeam(builder, this.teamNumber);
    SubmitDataScouting.addMatch(builder, this.matchNumber);
    SubmitDataScouting.addSetNumber(builder, this.setNumber);
    SubmitDataScouting.addCompLevel(builder, compLevel);
    SubmitDataScouting.addMissedShotsAuto(builder, this.autoShotsMissed);
    SubmitDataScouting.addUpperGoalAuto(builder, this.autoUpperShotsMade);
    SubmitDataScouting.addLowerGoalAuto(builder, this.autoLowerShotsMade);
    SubmitDataScouting.addMissedShotsTele(builder, this.teleShotsMissed);
    SubmitDataScouting.addUpperGoalTele(builder, this.teleUpperShotsMade);
    SubmitDataScouting.addLowerGoalTele(builder, this.teleLowerShotsMade);
    SubmitDataScouting.addDefenseRating(builder, this.defensePlayedScore);
    SubmitDataScouting.addDefenseReceivedRating(
      builder,
      this.defensePlayedOnScore
    );
    SubmitDataScouting.addAutoBall1(builder, this.ball1);
    SubmitDataScouting.addAutoBall2(builder, this.ball2);
    SubmitDataScouting.addAutoBall3(builder, this.ball3);
    SubmitDataScouting.addAutoBall4(builder, this.ball4);
    SubmitDataScouting.addAutoBall5(builder, this.ball5);
    SubmitDataScouting.addStartingQuadrant(builder, this.quadrant);
    SubmitDataScouting.addClimbLevel(builder, this.level);
    SubmitDataScouting.addComment(builder, comment);
    builder.finish(SubmitDataScouting.endSubmitDataScouting(builder));

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/submit/data_scouting', {
      method: 'POST',
      body: buffer,
    });

    if (res.ok) {
      // We successfully submitted the data. Report success.
      this.section = 'Success';
    } else {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
}
