import {Component, EventEmitter, OnInit, Output} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '../../webserver/requests/messages/error_response_generated';
import {RequestAllMatches} from '../../webserver/requests/messages/request_all_matches_generated';
import {
  Match,
  RequestAllMatchesResponse,
} from '../../webserver/requests/messages/request_all_matches_response_generated';

import {MatchListRequestor} from '@org_frc971/scouting/www/rpc';

type TeamInMatch = {
  teamNumber: number;
  matchNumber: number;
  setNumber: number;
  compLevel: string;
};

@Component({
  selector: 'app-match-list',
  templateUrl: './match_list.ng.html',
  styleUrls: ['../app/common.css', './match_list.component.css'],
})
export class MatchListComponent implements OnInit {
  @Output() selectedTeamEvent = new EventEmitter<TeamInMatch>();
  progressMessage: string = '';
  errorMessage: string = '';
  matchList: Match[] = [];
  hideCompletedMatches: boolean = true;

  constructor(private readonly matchListRequestor: MatchListRequestor) {}

  // Returns true if the match is fully scouted. Returns false otherwise.
  matchIsFullyScouted(match: Match): boolean {
    const scouted = match.dataScouted();
    return (
      scouted.r1() &&
      scouted.r2() &&
      scouted.r3() &&
      scouted.b1() &&
      scouted.b2() &&
      scouted.b3()
    );
  }

  // Returns true if at least one team in this match has been scouted. Returns
  // false otherwise.
  matchIsPartiallyScouted(match: Match): boolean {
    const scouted = match.dataScouted();
    return (
      scouted.r1() ||
      scouted.r2() ||
      scouted.r3() ||
      scouted.b1() ||
      scouted.b2() ||
      scouted.b3()
    );
  }

  // Returns a class for the row to hide it if all teams in this match have
  // already been scouted.
  getRowClass(match: Match): string {
    if (this.hideCompletedMatches && this.matchIsFullyScouted(match)) {
      return 'hidden_row';
    }
    return '';
  }

  setTeamInMatch(teamInMatch: TeamInMatch) {
    this.selectedTeamEvent.emit(teamInMatch);
  }

  teamsInMatch(
    match: Match
  ): {teamNumber: number; color: 'red' | 'blue'; disabled: boolean}[] {
    const scouted = match.dataScouted();
    return [
      {
        teamNumber: match.r1(),
        color: 'red',
        disabled: this.hideCompletedMatches && scouted.r1(),
      },
      {
        teamNumber: match.r2(),
        color: 'red',
        disabled: this.hideCompletedMatches && scouted.r2(),
      },
      {
        teamNumber: match.r3(),
        color: 'red',
        disabled: this.hideCompletedMatches && scouted.r3(),
      },
      {
        teamNumber: match.b1(),
        color: 'blue',
        disabled: this.hideCompletedMatches && scouted.b1(),
      },
      {
        teamNumber: match.b2(),
        color: 'blue',
        disabled: this.hideCompletedMatches && scouted.b2(),
      },
      {
        teamNumber: match.b3(),
        color: 'blue',
        disabled: this.hideCompletedMatches && scouted.b3(),
      },
    ];
  }

  matchType(match: Match): string | null {
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
    // Only display the set number for eliminations matches.
    const setNumber = match.compLevel() == 'qm' ? '' : `${match.setNumber()}`;
    const matchType = this.matchType(match);
    const mainText = `${matchType} ${setNumber} Match ${match.matchNumber()}`;

    // When showing the full match list (i.e. not hiding completed matches)
    // it's useful to know if a match has already been scouted or not.
    const suffix = (() => {
      if (this.matchIsFullyScouted(match)) {
        return '(fully scouted)';
      } else if (this.matchIsPartiallyScouted(match)) {
        return '(partially scouted)';
      } else {
        return '';
      }
    })();

    return `${mainText} ${suffix}`;
  }

  ngOnInit() {
    this.fetchMatchList();
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
}
