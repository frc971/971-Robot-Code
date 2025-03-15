import {Component, OnInit} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '@org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {
  DriverRanking2025,
  RequestAveragedDriverRankings2025Response,
} from '@org_frc971/scouting/webserver/requests/messages/request_averaged_driver_rankings_2025_response_generated';
import {
  HumanRanking2025,
  RequestAveragedHumanRankings2025Response,
} from '@org_frc971/scouting/webserver/requests/messages/request_averaged_human_rankings_2025_response_generated';
import {
  Stats2025,
  Request2025DataScoutingResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_2025_data_scouting_response_generated';

import {
  Note2025,
  RequestAllNotes2025Response,
} from '@org_frc971/scouting/webserver/requests/messages/request_all_notes_2025_response_generated';
import {Delete2025DataScouting} from '@org_frc971/scouting/webserver/requests/messages/delete_2025_data_scouting_generated';
import {Delete2025DataScoutingResponse} from '@org_frc971/scouting/webserver/requests/messages/delete_2025_data_scouting_response_generated';
import {DeleteNotes2025} from '@org_frc971/scouting/webserver/requests/messages/delete_notes_2025_generated';
import {DeleteNotes2025Response} from '@org_frc971/scouting/webserver/requests/messages/delete_notes_2025_response_generated';

import {
  MatchListRequestor,
  ViewDataRequestor,
} from '@org_frc971/scouting/www/rpc';

type Source = 'Notes' | 'Stats2025' | 'DriverRanking' | 'HumanRanking';

//TODO(Filip): Deduplicate
const COMP_LEVEL_LABELS = {
  qm: 'Qualifications',
  ef: 'Eighth Finals',
  qf: 'Quarter Finals',
  sf: 'Semi Finals',
  f: 'Finals',
};

@Component({
  selector: 'app-view',
  templateUrl: './view.ng.html',
  styleUrls: ['../app/common.css', './view.component.css'],
})
export class ViewComponent {
  constructor(private readonly viewDataRequestor: ViewDataRequestor) {}

  // Make COMP_LEVEL_LABELS available in view.ng.html.
  readonly COMP_LEVEL_LABELS = COMP_LEVEL_LABELS;

  // Progress and error messages to display to
  // the user when fetching data.
  progressMessage: string = '';
  errorMessage: string = '';
  compCode: string = '2016nytr';

  // The current data source being displayed.
  currentSource: Source = 'Notes';

  // Current sort (ascending/descending match numbers).
  // noteList is sorted based on team number until match
  // number is added for note scouting.
  ascendingSort = true;

  // Stores the corresponding data.
  noteList: Note2025[] = [];
  driverRankingList: DriverRanking2025[] = [];
  humanRankingList: HumanRanking2025[] = [];
  statList: Stats2025[] = [];

  // Fetch notes on initialization.
  ngOnInit() {
    this.fetchCurrentSource();
  }

  // Called when a user changes the sort direction.
  // Changes the data order between ascending/descending.
  sortData() {
    this.ascendingSort = !this.ascendingSort;
    if (!this.ascendingSort) {
      this.driverRankingList.sort(function (a, b) {
        return b
          .teamNumber()
          .localeCompare(a.teamNumber(), undefined, {numeric: true});
      });
      this.humanRankingList.sort(function (a, b) {
        return b
          .teamNumber()
          .localeCompare(a.teamNumber(), undefined, {numeric: true});
      });
      this.noteList.sort(function (a, b) {
        return b
          .teamNumber()
          .localeCompare(a.teamNumber(), undefined, {numeric: true});
      });

      this.statList.sort((a, b) => b.matchNumber() - a.matchNumber());
    } else {
      this.driverRankingList.sort(function (a, b) {
        return a
          .teamNumber()
          .localeCompare(b.teamNumber(), undefined, {numeric: true});
      });
      this.humanRankingList.sort(function (a, b) {
        return a
          .teamNumber()
          .localeCompare(b.teamNumber(), undefined, {numeric: true});
      });
      this.noteList.sort(function (a, b) {
        return a
          .teamNumber()
          .localeCompare(b.teamNumber(), undefined, {numeric: true});
      });

      this.statList.sort((a, b) => a.matchNumber() - b.matchNumber());
    }
  }

  // Called when a user selects a new data source
  // from the dropdown.
  switchDataSource(target: Source) {
    this.currentSource = target;
    this.progressMessage = '';
    this.errorMessage = '';
    this.noteList = [];
    this.driverRankingList = [];
    this.humanRankingList = [];
    this.statList = [];
    this.fetchCurrentSource();
  }

  // Call the method to fetch data for the current source.
  fetchCurrentSource() {
    switch (this.currentSource) {
      case 'Notes': {
        this.fetchNotes2025();
      }

      case 'Stats2025': {
        this.fetchStats2025();
      }

      case 'DriverRanking': {
        this.fetchDriverRanking2025();
      }

      case 'HumanRanking': {
        this.fetchHumanRanking2025();
      }
    }
  }

  async deleteNotes2025(
    compCode: string,
    compLevel: string,
    matchNumber: number,
    setNumber: number,
    teamNumber: string
  ) {
    const block_alerts = document.getElementById(
      'block_alerts'
    ) as HTMLInputElement;
    if (block_alerts.checked || window.confirm('Actually delete data?')) {
      await this.requestDeleteNotes2025(
        compCode,
        compLevel,
        matchNumber,
        setNumber,
        teamNumber
      );
      await this.fetchNotes2025();
    }
  }

  // TODO(Filip): Add delete functionality.
  // Gets called when a user clicks the delete icon (driver ranking).
  async deleteDriverRankingData() {
    const block_alerts = document.getElementById(
      'block_alerts'
    ) as HTMLInputElement;
    if (block_alerts.checked || window.confirm('Actually delete data?')) {
      this.errorMessage = 'Deleting data has not been implemented yet.';
      return;
    }
  }

  // Gets called when a user clicks the delete icon.
  async delete2025DataScouting(
    compCode: string,
    compLevel: string,
    matchNumber: number,
    setNumber: number,
    teamNumber: string
  ) {
    const block_alerts = document.getElementById(
      'block_alerts'
    ) as HTMLInputElement;
    if (block_alerts.checked || window.confirm('Actually delete data?')) {
      await this.requestDelete2025DataScouting(
        compCode,
        compLevel,
        matchNumber,
        setNumber,
        teamNumber
      );
      await this.fetchStats2025();
    }
  }

  async requestDelete2025DataScouting(
    compCode: string,
    compLevel: string,
    matchNumber: number,
    setNumber: number,
    teamNumber: string
  ) {
    this.progressMessage = 'Deleting data. Please be patient.';
    const builder = new Builder();
    const compLevelData = builder.createString(compLevel);
    const compCodeData = builder.createString(compCode);
    const teamNumberData = builder.createString(teamNumber);

    builder.finish(
      Delete2025DataScouting.createDelete2025DataScouting(
        builder,
        compLevelData,
        matchNumber,
        setNumber,
        teamNumberData,
        compCodeData
      )
    );

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/delete/delete_2025_data_scouting', {
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

  async requestDeleteNotes2025(
    compCode: string,
    compLevel: string,
    matchNumber: number,
    setNumber: number,
    teamNumber: string
  ) {
    this.progressMessage = 'Deleting data. Please be patient.';
    const builder = new Builder();
    const compCodeData = builder.createString(compCode);
    const compLevelData = builder.createString(compLevel);
    const teamNumberData = builder.createString(teamNumber);

    builder.finish(
      DeleteNotes2025.createDeleteNotes2025(
        builder,
        compCodeData,
        compLevelData,
        matchNumber,
        setNumber,
        teamNumberData
      )
    );

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/delete/delete_notes_2025', {
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

  // Fetch all driver ranking data and store in driverRankingList.
  async fetchDriverRanking2025() {
    this.progressMessage = 'Fetching driver ranking data. Please be patient.';
    this.errorMessage = '';

    try {
      this.driverRankingList =
        await this.viewDataRequestor.fetchDriverRanking2025List(this.compCode);
      this.progressMessage = 'Successfully fetched driver ranking data.';
    } catch (e) {
      this.errorMessage = e;
      this.progressMessage = '';
    }
  }

  async fetchHumanRanking2025() {
    this.progressMessage = 'Fetching human ranking data. Please be patient.';
    this.errorMessage = '';

    try {
      this.humanRankingList =
        await this.viewDataRequestor.fetchHumanRanking2025List(this.compCode);
      this.progressMessage = 'Successfully fetched human ranking data.';
    } catch (e) {
      this.errorMessage = e;
      this.progressMessage = '';
    }
  }

  // Fetch all data scouting (stats) data and store in statList.
  async fetchStats2025() {
    this.progressMessage = 'Fetching stats list. Please be patient.';
    this.errorMessage = '';

    try {
      this.statList = await this.viewDataRequestor.fetchStats2025List(
        this.compCode
      );
      this.progressMessage = 'Successfully fetched stats list.';
    } catch (e) {
      this.errorMessage = e;
      this.progressMessage = '';
    }
  }

  // Fetch all notes data and store in noteList.
  async fetchNotes2025() {
    this.progressMessage = 'Fetching notes list. Please be patient.';
    this.errorMessage = '';

    try {
      this.noteList = await this.viewDataRequestor.fetchNote2025List(
        this.compCode
      );
      this.progressMessage = 'Successfully fetched note 2025 list.';
    } catch (e) {
      this.errorMessage = e;
      this.progressMessage = '';
    }
  }

  // Parse all selected keywords for a note entry
  // into one string to be displayed in the table.
  parseKeywords(entry: Note2025) {
    let parsedKeywords = '';

    if (entry.coralGroundIntake()) {
      parsedKeywords += 'Coral Ground Intake, ';
    } else {
      parsedKeywords += 'No Coral Ground Intake, ';
    }
    if (entry.coralHpIntake()) {
      parsedKeywords += 'Coral HP Intake, ';
    } else {
      parsedKeywords += 'No Coral HP Intake, ';
    }
    if (entry.solidAlgaeShooting()) {
      parsedKeywords += 'Solid Algae Shooting, ';
    } else {
      parsedKeywords += 'No Solid Algae Shooting, ';
    }
    if (entry.penalties()) {
      parsedKeywords += 'Penalties, ';
    } else {
      parsedKeywords += 'No Penalties, ';
    }
    if (entry.goodDriving()) {
      parsedKeywords += 'Good Driving, ';
    }
    if (entry.badDriving()) {
      parsedKeywords += 'Bad Driving, ';
    }
    return parsedKeywords;
  }
}
