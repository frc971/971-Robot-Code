import {Component, EventEmitter, OnInit, Output} from '@angular/core';
import * as flatbuffer_builder from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';
import * as error_response from 'org_frc971/scouting/webserver/requests/messages/error_response_generated';
import * as request_all_matches from 'org_frc971/scouting/webserver/requests/messages/request_all_matches_generated';
import * as request_all_matches_response from 'org_frc971/scouting/webserver/requests/messages/request_all_matches_response_generated';

import RequestAllMatches = request_all_matches.scouting.webserver.requests.RequestAllMatches;
import RequestAllMatchesResponse = request_all_matches_response.scouting.webserver.requests.RequestAllMatchesResponse;
import Match = request_all_matches_response.scouting.webserver.requests.Match;
import ErrorResponse = error_response.scouting.webserver.requests.ErrorResponse;

type TeamInMatch = {
  teamNumber: number,
  matchNumber: number,
  compLevel: string
};

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

    const builder =
        new flatbuffer_builder.Builder() as unknown as flatbuffers.Builder;
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
              fbBuffer as unknown as flatbuffers.ByteBuffer);

      this.matchList = [];
      for (let i = 0; i < parsedResponse.matchListLength(); i++) {
        this.matchList.push(parsedResponse.matchList(i));
      }
      this.matchList.sort((a, b) => {
        let aString = this.displayMatchNumber(a);
        let bString = this.displayMatchNumber(b);
        if (aString < bString) {
          return -1;
        }
        if (aString > bString) {
          return 1;
        }
        return 0;
      });

      this.progressMessage = 'Successfully fetched match list.';
    } else {
      this.progressMessage = '';
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(
          fbBuffer as unknown as flatbuffers.ByteBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage =
          `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
}