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
import {ErrorResponse} from '../../webserver/requests/messages/error_response_generated';
import {
  StartMatchAction,
  ScoreType,
  StageType,
  Submit2024Actions,
  MobilityAction,
  PenaltyAction,
  PickupNoteAction,
  PlaceNoteAction,
  RobotDeathAction,
  EndMatchAction,
  ActionType,
  Action,
} from '../../webserver/requests/messages/submit_2024_actions_generated';
import {Match} from '../../webserver/requests/messages/request_all_matches_response_generated';
import {MatchListRequestor} from '../rpc';

type Section =
  | 'Team Selection'
  | 'Init'
  | 'Pickup'
  | 'Place'
  | 'Endgame'
  | 'Dead'
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

type ActionT =
  | {
      type: 'startMatchAction';
      timestamp?: number;
      position: number;
    }
  | {
      type: 'mobilityAction';
      timestamp?: number;
      mobility: boolean;
    }
  | {
      type: 'pickupNoteAction';
      timestamp?: number;
      auto?: boolean;
    }
  | {
      type: 'placeNoteAction';
      timestamp?: number;
      scoreType: ScoreType;
      auto?: boolean;
    }
  | {
      type: 'robotDeathAction';
      timestamp?: number;
      robotOn: boolean;
    }
  | {
      type: 'penaltyAction';
      timestamp?: number;
      penalties: number;
    }
  | {
      type: 'endMatchAction';
      stageType: StageType;
      trapNote: boolean;
      spotlight: boolean;
      timestamp?: number;
    }
  | {
      // This is not a action that is submitted,
      // It is used for undoing purposes.
      type: 'endAutoPhase';
      timestamp?: number;
    }
  | {
      // This is not a action that is submitted,
      // It is used for undoing purposes.
      type: 'endTeleopPhase';
      timestamp?: number;
    };

@Component({
  selector: 'app-entry',
  templateUrl: './entry.ng.html',
  styleUrls: ['../app/common.css', './entry.component.css'],
})
export class EntryComponent implements OnInit {
  // Re-export the type here so that we can use it in the `[value]` attribute
  // of radio buttons.
  readonly COMP_LEVELS = COMP_LEVELS;
  readonly COMP_LEVEL_LABELS = COMP_LEVEL_LABELS;
  readonly ScoreType = ScoreType;
  readonly StageType = StageType;

  section: Section = 'Team Selection';
  @Input() matchNumber: number = 1;
  @Input() teamNumber: string = '1';
  @Input() setNumber: number = 1;
  @Input() compLevel: CompLevel = 'qm';
  @Input() skipTeamSelection = false;

  matchList: Match[] = [];

  actionList: ActionT[] = [];
  progressMessage: string = '';
  errorMessage: string = '';
  autoPhase: boolean = true;
  mobilityCompleted: boolean = false;

  preScouting: boolean = false;
  matchStartTimestamp: number = 0;
  penalties: number = 0;

  teamSelectionIsValid = false;

  constructor(private readonly matchListRequestor: MatchListRequestor) {}

  ngOnInit() {
    // When the user navigated from the match list, we can skip the team
    // selection. I.e. we trust that the user clicked the correct button.
    this.section = this.skipTeamSelection ? 'Init' : 'Team Selection';

    if (this.section == 'Team Selection') {
      this.fetchMatchList();
    }
  }

  async fetchMatchList() {
    this.progressMessage = 'Fetching match list. Please be patient.';
    this.errorMessage = '';

    try {
      this.matchList = await this.matchListRequestor.fetchMatchList();
      this.progressMessage = 'Successfully fetched match list.';
    } catch (e) {
      this.errorMessage = e;
      this.progressMessage = '';
    }
  }

  // This gets called when the user changes something on the Init screen.
  // It makes sure that the user can't click "Next" until the information is
  // valid, or this is for pre-scouting.
  updateTeamSelectionValidity(): void {
    this.teamSelectionIsValid = this.preScouting || this.matchIsInMatchList();
  }

  matchIsInMatchList(): boolean {
    // If the user deletes the content of the teamNumber field, the value here
    // is undefined. Guard against that.
    if (this.teamNumber == null) {
      return false;
    }
    const teamNumber = this.teamNumber;

    for (const match of this.matchList) {
      if (
        this.matchNumber == match.matchNumber() &&
        this.setNumber == match.setNumber() &&
        this.compLevel == match.compLevel() &&
        (teamNumber === match.r1() ||
          teamNumber === match.r2() ||
          teamNumber === match.r3() ||
          teamNumber === match.b1() ||
          teamNumber === match.b2() ||
          teamNumber === match.b3())
      ) {
        return true;
      }
    }
    return false;
  }

  addPenalty(): void {
    this.penalties += 1;
  }

  removePenalty(): void {
    if (this.penalties > 0) {
      this.penalties -= 1;
    }
  }

  addPenalties(): void {
    this.addAction({type: 'penaltyAction', penalties: this.penalties});
  }

  addAction(action: ActionT): void {
    if (action.type == 'startMatchAction') {
      // Unix nanosecond timestamp.
      this.matchStartTimestamp = Date.now() * 1e6;
      action.timestamp = 0;
    } else {
      // Unix nanosecond timestamp relative to match start.
      action.timestamp = Date.now() * 1e6 - this.matchStartTimestamp;
    }

    if (action.type == 'endMatchAction') {
      // endMatchAction occurs at the same time as penaltyAction so add to its timestamp to make it unique.
      action.timestamp += 1;
    }

    if (action.type == 'mobilityAction') {
      this.mobilityCompleted = true;
    }

    if (action.type == 'pickupNoteAction' || action.type == 'placeNoteAction') {
      action.auto = this.autoPhase;
    }
    this.actionList.push(action);
  }

  undoLastAction() {
    if (this.actionList.length > 0) {
      let lastAction = this.actionList.pop();
      switch (lastAction?.type) {
        case 'endAutoPhase':
          this.autoPhase = true;
          this.section = 'Pickup';
        case 'pickupNoteAction':
          this.section = 'Pickup';
          break;
        case 'endTeleopPhase':
          this.section = 'Pickup';
          break;
        case 'placeNoteAction':
          this.section = 'Place';
          break;
        case 'endMatchAction':
          this.section = 'Endgame';
        case 'mobilityAction':
          this.mobilityCompleted = false;
          break;
        case 'startMatchAction':
          this.section = 'Init';
          break;
        case 'robotDeathAction':
          // TODO(FILIP): Return user to the screen they
          // clicked dead robot on. Pickup is fine for now but
          // might cause confusion.
          this.section = 'Pickup';
          break;
        default:
          break;
      }
    }
  }

  stringifyScoreType(scoreType: ScoreType): String {
    return ScoreType[scoreType];
  }

  stringifyStageType(stageType: StageType): String {
    return StageType[stageType];
  }

  changeSectionTo(target: Section) {
    // Clear the messages since they won't be relevant in the next section.
    this.errorMessage = '';
    this.progressMessage = '';

    this.section = target;
  }

  @ViewChild('header') header: ElementRef;

  private scrollToTop() {
    this.header.nativeElement.scrollIntoView();
  }

  async submit2024Actions() {
    const builder = new Builder();
    const actionOffsets: number[] = [];

    for (const action of this.actionList) {
      let actionOffset: number | undefined;
      console.log(action.type);

      switch (action.type) {
        case 'startMatchAction':
          const startMatchActionOffset =
            StartMatchAction.createStartMatchAction(builder, action.position);
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.StartMatchAction,
            startMatchActionOffset
          );
          break;
        case 'mobilityAction':
          const mobilityActionOffset = MobilityAction.createMobilityAction(
            builder,
            action.mobility
          );
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.MobilityAction,
            mobilityActionOffset
          );
          break;
        case 'penaltyAction':
          const penaltyActionOffset = PenaltyAction.createPenaltyAction(
            builder,
            action.penalties
          );
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.PenaltyAction,
            penaltyActionOffset
          );
          break;
        case 'pickupNoteAction':
          const pickupNoteActionOffset =
            PickupNoteAction.createPickupNoteAction(
              builder,
              action.auto || false
            );
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.PickupNoteAction,
            pickupNoteActionOffset
          );
          break;
        case 'placeNoteAction':
          const placeNoteActionOffset = PlaceNoteAction.createPlaceNoteAction(
            builder,
            action.scoreType,
            action.auto || false
          );
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.PlaceNoteAction,
            placeNoteActionOffset
          );
          break;

        case 'robotDeathAction':
          const robotDeathActionOffset =
            RobotDeathAction.createRobotDeathAction(builder, action.robotOn);
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.RobotDeathAction,
            robotDeathActionOffset
          );
          break;

        case 'endMatchAction':
          const endMatchActionOffset = EndMatchAction.createEndMatchAction(
            builder,
            action.stageType,
            action.trapNote,
            action.spotlight
          );
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.EndMatchAction,
            endMatchActionOffset
          );
          break;

        case 'endAutoPhase':
          // Not important action.
          break;

        case 'endTeleopPhase':
          // Not important action.
          break;

        default:
          throw new Error(`Unknown action type`);
      }

      if (actionOffset !== undefined) {
        actionOffsets.push(actionOffset);
      }
    }
    const teamNumberFb = builder.createString(this.teamNumber);
    const compLevelFb = builder.createString(this.compLevel);

    const actionsVector = Submit2024Actions.createActionsListVector(
      builder,
      actionOffsets
    );
    Submit2024Actions.startSubmit2024Actions(builder);
    Submit2024Actions.addTeamNumber(builder, teamNumberFb);
    Submit2024Actions.addMatchNumber(builder, this.matchNumber);
    Submit2024Actions.addSetNumber(builder, this.setNumber);
    Submit2024Actions.addCompLevel(builder, compLevelFb);
    Submit2024Actions.addActionsList(builder, actionsVector);
    Submit2024Actions.addPreScouting(builder, this.preScouting);
    builder.finish(Submit2024Actions.endSubmit2024Actions(builder));

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/submit/submit_2024_actions', {
      method: 'POST',
      body: buffer,
    });

    if (res.ok) {
      // We successfully submitted the data. Report success.
      this.section = 'Success';
      this.actionList = [];
    } else {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
}
