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
  ObjectType,
  ScoreLevel,
  SubmitActions,
  StartMatchAction,
  MobilityAction,
  AutoBalanceAction,
  PickupObjectAction,
  PlaceObjectAction,
  RobotDeathAction,
  EndMatchAction,
  ActionType,
  Action,
} from '../../webserver/requests/messages/submit_actions_generated';
import {Match} from '../../webserver/requests/messages/request_all_matches_response_generated';
import {MatchListRequestor} from '@org_frc971/scouting/www/rpc';

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
      type: 'autoBalanceAction';
      timestamp?: number;
      docked: boolean;
      engaged: boolean;
      balanceAttempt: boolean;
    }
  | {
      type: 'pickupObjectAction';
      timestamp?: number;
      objectType: ObjectType;
      auto?: boolean;
    }
  | {
      type: 'placeObjectAction';
      timestamp?: number;
      objectType?: ObjectType;
      scoreLevel: ScoreLevel;
      auto?: boolean;
    }
  | {
      type: 'robotDeathAction';
      timestamp?: number;
      robotOn: boolean;
    }
  | {
      type: 'endMatchAction';
      docked: boolean;
      engaged: boolean;
      balanceAttempt: boolean;
      timestamp?: number;
    }
  | {
      // This is not a action that is submitted,
      // It is used for undoing purposes.
      type: 'endAutoPhase';
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
  readonly ObjectType = ObjectType;
  readonly ScoreLevel = ScoreLevel;

  section: Section = 'Team Selection';
  @Input() matchNumber: number = 1;
  // TODO(phil): Change the type of teamNumber to a string.
  @Input() teamNumber: number = 1;
  @Input() setNumber: number = 1;
  @Input() compLevel: CompLevel = 'qm';
  @Input() skipTeamSelection = false;

  matchList: Match[] = [];

  actionList: ActionT[] = [];
  progressMessage: string = '';
  errorMessage: string = '';
  autoPhase: boolean = true;
  mobilityCompleted: boolean = false;
  lastObject: ObjectType = null;

  preScouting: boolean = false;
  matchStartTimestamp: number = 0;

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
    const teamNumber = this.teamNumber.toString();

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

  addAction(action: ActionT): void {
    if (action.type == 'startMatchAction') {
      // Unix nanosecond timestamp.
      this.matchStartTimestamp = Date.now() * 1e6;
      action.timestamp = 0;
    } else {
      // Unix nanosecond timestamp relative to match start.
      action.timestamp = Date.now() * 1e6 - this.matchStartTimestamp;
    }

    if (action.type == 'mobilityAction') {
      this.mobilityCompleted = true;
    }

    if (action.type == 'autoBalanceAction') {
      // Timestamp is a unique index in the database so
      // adding one makes sure it dosen't overlap with the
      // start teleop action that is added at the same time.
      action.timestamp += 1;
    }

    if (
      action.type == 'pickupObjectAction' ||
      action.type == 'placeObjectAction'
    ) {
      action.auto = this.autoPhase;
      if (action.type == 'pickupObjectAction') {
        this.lastObject = action.objectType;
      } else if (action.type == 'placeObjectAction') {
        action.objectType = this.lastObject;
      }
    }
    this.actionList.push(action);
  }

  undoLastAction() {
    if (this.actionList.length > 0) {
      let lastAction = this.actionList.pop();
      switch (lastAction?.type) {
        case 'endAutoPhase':
          this.autoPhase = true;
        case 'pickupObjectAction':
          this.section = 'Pickup';
          break;
        case 'placeObjectAction':
          this.section = 'Place';
          break;
        case 'endMatchAction':
          this.section = 'Pickup';
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

  async submitActions() {
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
        case 'autoBalanceAction':
          const autoBalanceActionOffset =
            AutoBalanceAction.createAutoBalanceAction(
              builder,
              action.docked,
              action.engaged,
              action.balanceAttempt
            );
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.AutoBalanceAction,
            autoBalanceActionOffset
          );
          break;

        case 'pickupObjectAction':
          const pickupObjectActionOffset =
            PickupObjectAction.createPickupObjectAction(
              builder,
              action.objectType,
              action.auto || false
            );
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.PickupObjectAction,
            pickupObjectActionOffset
          );
          break;
        case 'placeObjectAction':
          const placeObjectActionOffset =
            PlaceObjectAction.createPlaceObjectAction(
              builder,
              action.objectType,
              action.scoreLevel,
              action.auto || false
            );
          actionOffset = Action.createAction(
            builder,
            BigInt(action.timestamp || 0),
            ActionType.PlaceObjectAction,
            placeObjectActionOffset
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
            action.docked,
            action.engaged,
            action.balanceAttempt
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

        default:
          throw new Error(`Unknown action type`);
      }

      if (actionOffset !== undefined) {
        actionOffsets.push(actionOffset);
      }
    }
    const teamNumberFb = builder.createString(this.teamNumber.toString());
    const compLevelFb = builder.createString(this.compLevel);

    const actionsVector = SubmitActions.createActionsListVector(
      builder,
      actionOffsets
    );
    SubmitActions.startSubmitActions(builder);
    SubmitActions.addTeamNumber(builder, teamNumberFb);
    SubmitActions.addMatchNumber(builder, this.matchNumber);
    SubmitActions.addSetNumber(builder, this.setNumber);
    SubmitActions.addCompLevel(builder, compLevelFb);
    SubmitActions.addActionsList(builder, actionsVector);
    SubmitActions.addPreScouting(builder, this.preScouting);
    builder.finish(SubmitActions.endSubmitActions(builder));

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/submit/submit_actions', {
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
