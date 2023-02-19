import {Component, OnInit} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {SubmitDriverRanking} from '../../webserver/requests/messages/submit_driver_ranking_generated';
import {ErrorResponse} from '../../webserver/requests/messages/error_response_generated';

// TeamSelection: Display form to input which
// teams to rank and the match number.
// Data: Display the ranking interface where
// the scout can reorder teams and submit data.
type Section = 'TeamSelection' | 'Data';

@Component({
  selector: 'app-driver-ranking',
  templateUrl: './driver_ranking.ng.html',
  styleUrls: ['../app/common.css', './driver_ranking.component.css'],
})
export class DriverRankingComponent {
  section: Section = 'TeamSelection';

  // Stores the team keys and rank (order of the array).
  team_ranking: number[] = [971, 972, 973];

  match_number: number = 1;

  errorMessage = '';

  setTeamNumbers() {
    this.section = 'Data';
  }

  rankUp(index: number) {
    if (index > 0) {
      this.changeRank(index, index - 1);
    }
  }

  rankDown(index: number) {
    if (index < 2) {
      this.changeRank(index, index + 1);
    }
  }

  // Change the rank of a team in team_ranking.
  // Move the the team at index 'fromIndex'
  // to the index 'toIndex'.
  // Ex. Moving the rank 2 (index 1) team to rank1 (index 0)
  // would be changeRank(1, 0)

  changeRank(fromIndex: number, toIndex: number) {
    var element = this.team_ranking[fromIndex];
    this.team_ranking.splice(fromIndex, 1);
    this.team_ranking.splice(toIndex, 0, element);
  }

  editTeams() {
    this.section = 'TeamSelection';
  }

  async submitData() {
    const builder = new Builder();
    builder.finish(
      SubmitDriverRanking.createSubmitDriverRanking(
        builder,
        this.match_number,
        this.team_ranking[0],
        this.team_ranking[1],
        this.team_ranking[2]
      )
    );
    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/submit/submit_driver_ranking', {
      method: 'POST',
      body: buffer,
    });

    if (!res.ok) {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
      return;
    }

    // Increment the match number.
    this.match_number = this.match_number + 1;

    // Reset Data.
    this.section = 'TeamSelection';
    this.team_ranking = [971, 972, 973];
  }
}
