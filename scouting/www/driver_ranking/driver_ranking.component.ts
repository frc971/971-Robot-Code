import {Component, OnInit} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {SubmitDriverRanking2025} from '@org_frc971/scouting/webserver/requests/messages/submit_driver_ranking_2025_generated';
import {ErrorResponse} from '@org_frc971/scouting/webserver/requests/messages/error_response_generated';

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
  team_ranking: string[] = ['1', '2', '3', '4', '5', '6'];
  rankings: number[] = [1, 1, 1, 1, 1, 1];

  match_number: number = 1;
  comp_code: string = '2016nytr';

  errorMessage = '';

  setTeamNumbers() {
    this.section = 'Data';
  }

  editTeams() {
    this.section = 'TeamSelection';
  }

  async submitDriverRankingData() {
    for (let i = 0; i < 6; i++) {
      const builder = new Builder();

      builder.finish(
        SubmitDriverRanking2025.createSubmitDriverRanking2025(
          builder,
          builder.createString(this.comp_code),
          this.match_number,
          builder.createString(this.team_ranking[i]),
          this.rankings[i]
        )
      );

      const buffer = builder.asUint8Array();
      const res = await fetch('/requests/submit/submit_driver_ranking_2025', {
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
    }
  }
  async submitData() {
    await this.submitDriverRankingData();

    if (this.errorMessage == '') {
      // Increment the match number.
      this.match_number = this.match_number + 1;

      // Reset Data.
      this.section = 'TeamSelection';
      this.team_ranking = ['1', '2', '3', '4', '5', '6'];
      this.rankings = [1, 1, 1, 1, 1, 1];
      this.errorMessage = '';
    }
  }
}
