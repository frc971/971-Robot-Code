import { Component, OnInit } from '@angular/core';

import * as flatbuffer_builder from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';
import * as error_response from 'org_frc971/scouting/webserver/requests/messages/error_response_generated';
import * as refresh_match_list_response from 'org_frc971/scouting/webserver/requests/messages/refresh_match_list_response_generated';
import * as refresh_match_list from 'org_frc971/scouting/webserver/requests/messages/refresh_match_list_generated';
import RefreshMatchList = refresh_match_list.scouting.webserver.requests.RefreshMatchList;
import RefreshMatchListResponse = refresh_match_list_response.scouting.webserver.requests.RefreshMatchListResponse;
import ErrorResponse = error_response.scouting.webserver.requests.ErrorResponse;

@Component({
  selector: 'app-import-match-list',
  templateUrl: './import_match_list.ng.html',
  styleUrls: ['../common.css', './import_match_list.component.css']
})
export class ImportMatchListComponent {
  year: number = new Date().getFullYear();
  eventCode: string = '';
  progressMessage: string = '';
  errorMessage: string = '';

  async importMatchList() {
    const block_alerts = document.getElementById('block_alerts') as HTMLInputElement;
    console.log(block_alerts.checked);
    if (!block_alerts.checked) {
      if (!window.confirm('Actually import new matches?')) {
        return;
      }
    }

    this.errorMessage = '';

    const builder = new flatbuffer_builder.Builder() as unknown as flatbuffers.Builder;
    const eventCode = builder.createString(this.eventCode);
    RefreshMatchList.startRefreshMatchList(builder);
    RefreshMatchList.addYear(builder, this.year);
    RefreshMatchList.addEventCode(builder, eventCode);
    builder.finish(RefreshMatchList.endRefreshMatchList(builder));

    this.progressMessage = 'Importing match list. Please be patient.';

    const buffer = builder.asUint8Array();
    const res = await fetch(
        '/requests/refresh_match_list', {method: 'POST', body: buffer});

    if (res.ok) {
      // We successfully submitted the data.
      this.progressMessage = 'Successfully imported match list.';
    } else {
      this.progressMessage = '';
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(
          fbBuffer as unknown as flatbuffers.ByteBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
}
