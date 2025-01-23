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
  Stats2024,
  Request2024DataScoutingResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_2024_data_scouting_response_generated';

import {
  PitImage,
  RequestAllPitImagesResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_all_pit_images_response_generated';

import {
  Note,
  RequestAllNotesResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_all_notes_response_generated';
import {Delete2024DataScouting} from '@org_frc971/scouting/webserver/requests/messages/delete_2024_data_scouting_generated';
import {Delete2024DataScoutingResponse} from '@org_frc971/scouting/webserver/requests/messages/delete_2024_data_scouting_response_generated';

import {
  MatchListRequestor,
  ViewDataRequestor,
} from '@org_frc971/scouting/www/rpc';

type Source =
  | 'Notes'
  | 'Stats2024'
  | 'PitImages'
  | 'DriverRanking'
  | 'HumanRanking';

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

  // The current data source being displayed.
  currentSource: Source = 'Notes';

  // Current sort (ascending/descending match numbers).
  // noteList is sorted based on team number until match
  // number is added for note scouting.
  ascendingSort = true;

  // Stores the corresponding data.
  noteList: Note[] = [];
  driverRankingList: DriverRanking2025[] = [];
  humanRankingList: HumanRanking2025[] = [];
  pitImageList: PitImage[][] = [];
  statList: Stats2024[] = [];
  compCode: string = '2025camb';

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
        return b.team().localeCompare(a.team(), undefined, {numeric: true});
      });
      this.pitImageList.sort(function (a, b) {
        return b[0]
          .teamNumber()
          .localeCompare(a[0].teamNumber(), undefined, {numeric: true});
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
        return a.team().localeCompare(b.team(), undefined, {numeric: true});
      });
      this.pitImageList.sort(function (a, b) {
        return a[0]
          .teamNumber()
          .localeCompare(b[0].teamNumber(), undefined, {numeric: true});
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
    this.pitImageList = [];
    this.fetchCurrentSource();
  }

  // Call the method to fetch data for the current source.
  fetchCurrentSource() {
    switch (this.currentSource) {
      case 'Notes': {
        this.fetchNotes();
      }

      case 'Stats2024': {
        this.fetchStats2024();
      }

      case 'PitImages': {
        this.fetchPitImages();
      }

      case 'DriverRanking': {
        this.fetchDriverRanking2025();
      }

      case 'HumanRanking': {
        this.fetchHumanRanking2025();
      }
    }
  }

  // TODO(Filip): Add delete functionality.
  // Gets called when a user clicks the delete icon (note scouting).
  async deleteNoteData() {
    const block_alerts = document.getElementById(
      'block_alerts'
    ) as HTMLInputElement;
    if (block_alerts.checked || window.confirm('Actually delete data?')) {
      this.errorMessage = 'Deleting data has not been implemented yet.';
      return;
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
  async delete2024DataScouting(
    compLevel: string,
    matchNumber: number,
    setNumber: number,
    teamNumber: string
  ) {
    const block_alerts = document.getElementById(
      'block_alerts'
    ) as HTMLInputElement;
    if (block_alerts.checked || window.confirm('Actually delete data?')) {
      await this.requestDelete2024DataScouting(
        compLevel,
        matchNumber,
        setNumber,
        teamNumber
      );
      await this.fetchStats2024();
    }
  }

  async requestDelete2024DataScouting(
    compLevel: string,
    matchNumber: number,
    setNumber: number,
    teamNumber: string
  ) {
    this.progressMessage = 'Deleting data. Please be patient.';
    const builder = new Builder();
    const compLevelData = builder.createString(compLevel);
    const teamNumberData = builder.createString(teamNumber);

    builder.finish(
      Delete2024DataScouting.createDelete2024DataScouting(
        builder,
        compLevelData,
        matchNumber,
        setNumber,
        teamNumberData
      )
    );

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/delete/delete_2024_data_scouting', {
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

  // Fetch all pit image data and store in pitImageList.
  async fetchPitImages() {
    this.progressMessage = 'Fetching pit image list. Please be patient.';
    this.errorMessage = '';

    try {
      const initialPitImageList =
        await this.viewDataRequestor.fetchPitImagesList();
      let officialPitImageList = [];
      // Use iteration to make an array of arrays containing pit image data for individual teams.
      // Ex. [ [ {971PitImageData} , {971PitImage2Data} ], [ {432PitImageData} ] , [ {213PitImageData} ] ]
      for (let pitImage of initialPitImageList) {
        let found = false;
        for (let arr of officialPitImageList) {
          if (arr[0].teamNumber() == pitImage.teamNumber()) {
            arr.push(pitImage);
            found = true;
          }
        }
        if (!found) {
          officialPitImageList.push([pitImage]);
        }
      }
      // Sort the arrays based on image file names so their order is predictable.
      for (let arr of officialPitImageList) {
        arr.sort((a, b) => (a.imagePath() > b.imagePath() ? 1 : -1));
      }
      this.pitImageList = officialPitImageList;
      this.sortData();
      this.progressMessage = 'Successfully fetched pit image list.';
    } catch (e) {
      this.errorMessage = e;
      this.progressMessage = '';
    }
  }

  // Fetch all data scouting (stats) data and store in statList.
  async fetchStats2024() {
    this.progressMessage = 'Fetching stats list. Please be patient.';
    this.errorMessage = '';

    try {
      this.statList = await this.viewDataRequestor.fetchStats2024List();
      this.progressMessage = 'Successfully fetched stats list.';
    } catch (e) {
      this.errorMessage = e;
      this.progressMessage = '';
    }
  }

  // Fetch all notes data and store in noteList.
  async fetchNotes() {
    this.progressMessage = 'Fetching notes list. Please be patient.';
    this.errorMessage = '';

    try {
      this.noteList = await this.viewDataRequestor.fetchNoteList();
      this.progressMessage = 'Successfully fetched note list.';
    } catch (e) {
      this.errorMessage = e;
      this.progressMessage = '';
    }
  }

  // Parse all selected keywords for a note entry
  // into one string to be displayed in the table.
  parseKeywords(entry: Note) {
    let parsedKeywords = '';

    if (entry.goodDriving()) {
      parsedKeywords += 'Good Driving ';
    }
    if (entry.badDriving()) {
      parsedKeywords += 'Bad Driving ';
    }
    if (entry.noShow()) {
      parsedKeywords += 'No Show ';
    }
    if (entry.solidPlacing()) {
      parsedKeywords += 'Solid Placing ';
    }
    if (entry.sketchyPlacing()) {
      parsedKeywords += 'Sketchy Placing ';
    }
    if (entry.goodDefense()) {
      parsedKeywords += 'Good Defense ';
    }
    if (entry.badDefense()) {
      parsedKeywords += 'Bad Defense ';
    }
    if (entry.easilyDefended()) {
      parsedKeywords += 'Easily Defended';
    }

    return parsedKeywords;
  }
}
