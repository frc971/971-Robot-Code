import {Component, EventEmitter, OnInit, Output} from '@angular/core';
import {ByteBuffer, Builder} from 'flatbuffers'
import {ErrorResponse} from 'org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {RequestAllMatches} from 'org_frc971/scouting/webserver/requests/messages/request_all_matches_generated';
import {Match, RequestAllMatchesResponse} from 'org_frc971/scouting/webserver/requests/messages/request_all_matches_response_generated';

type TeamInMatch = {
  teamNumber: number,
  matchNumber: number,
  compLevel: string
};

const MATCH_TYPE_ORDERING = ['qm', 'ef', 'qf', 'sf', 'f'];

@Component({
  selector: 'app-match-list',
  templateUrl: './match_list.ng.html',
  styleUrls: ['../common.css', './match_list.component.css']
})
export class MatchListComponent implements OnInit {
  @Output() selectedTeamEvent = new EventEmitter<TeamInMatch>();
  teamInMatch: TeamInMatch = {teamNumber: 1, matchNumber: 1, compLevel: 'qm'};
  progressMessage: string = '';
  errorMessage: string = '';
  matchList: Match[] = [];

  setTeamInMatch(teamInMatch: TeamInMatch) {
    this.teamInMatch = teamInMatch;
    this.selectedTeamEvent.emit(teamInMatch);
  }

  teamsInMatch(match: Match): {number: number, color: 'red'|'blue'}[] {
    return [
      {number: match.r1(), color: 'red'},
      {number: match.r2(), color: 'red'},
      {number: match.r3(), color: 'red'},
      {number: match.b1(), color: 'blue'},
      {number: match.b2(), color: 'blue'},
      {number: match.b3(), color: 'blue'},
    ];
  }

  matchType(match: Match): string|null {
    switch (match.compLevel()) {
      case 'qm':
        return 'Quals';
      case 'ef':
        return 'Eighth Final';
      case 'qf':
        return 'Quarter Final';
      case 'sf':
        return 'Semi Final';
      case 'f':
        return 'Final';
      default:
        return null;
    }
  }

  displayMatchNumber(match: Match): string {
    return `${this.matchType(match)} ${match.matchNumber()}`;
  }

  ngOnInit() {
    this.importMatchList();
  }

  async importMatchList() {
    this.errorMessage = '';

    const builder = new Builder();
    RequestAllMatches.startRequestAllMatches(builder);
    builder.finish(RequestAllMatches.endRequestAllMatches(builder));

    this.progressMessage = 'Fetching match list. Please be patient.';

    const buffer = builder.asUint8Array();
    const res = await fetch(
        '/requests/request/all_matches', {method: 'POST', body: buffer});

    if (res.ok) {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse =
          RequestAllMatchesResponse.getRootAsRequestAllMatchesResponse(
              fbBuffer);

      // Convert the flatbuffer list into an array. That's more useful.
      this.matchList = [];
      for (let i = 0; i < parsedResponse.matchListLength(); i++) {
        this.matchList.push(parsedResponse.matchList(i));
      }

      // Sort the list so it is in chronological order.
      this.matchList.sort((a, b) => {
        const aMatchTypeIndex = MATCH_TYPE_ORDERING.indexOf(a.compLevel());
        const bMatchTypeIndex = MATCH_TYPE_ORDERING.indexOf(b.compLevel());
        if (aMatchTypeIndex < bMatchTypeIndex) {
          return -1;
        }
        if (aMatchTypeIndex > bMatchTypeIndex) {
          return 1;
        }
        const aMatchNumber = a.matchNumber();
        const bMatchNumber = b.matchNumber();
        if (aMatchNumber < bMatchNumber) {
          return -1;
        }
        if (aMatchNumber > bMatchNumber) {
          return 1;
        }
        return 0;
      });

      this.progressMessage = 'Successfully fetched match list.';
    } else {
      this.progressMessage = '';
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage =
          `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
}
