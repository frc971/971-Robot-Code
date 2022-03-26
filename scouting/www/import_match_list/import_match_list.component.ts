import {Component, OnInit} from '@angular/core';

import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from 'org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {RefreshMatchListResponse} from 'org_frc971/scouting/webserver/requests/messages/refresh_match_list_response_generated';
import {RefreshMatchList} from 'org_frc971/scouting/webserver/requests/messages/refresh_match_list_generated';

@Component({
  selector: 'app-import-match-list',
  templateUrl: './import_match_list.ng.html',
  styleUrls: ['../common.css', './import_match_list.component.css'],
})
export class ImportMatchListComponent {
  year: number = new Date().getFullYear();
  eventCode: string = '';
  progressMessage: string = '';
  errorMessage: string = '';

  async importMatchList() {
    const block_alerts = document.getElementById(
      'block_alerts'
    ) as HTMLInputElement;
    console.log(block_alerts.checked);
    if (!block_alerts.checked) {
      if (!window.confirm('Actually import new matches?')) {
        return;
      }
    }

    this.errorMessage = '';

    const builder = new Builder();
    const eventCode = builder.createString(this.eventCode);
    RefreshMatchList.startRefreshMatchList(builder);
    RefreshMatchList.addYear(builder, this.year);
    RefreshMatchList.addEventCode(builder, eventCode);
    builder.finish(RefreshMatchList.endRefreshMatchList(builder));

    this.progressMessage = 'Importing match list. Please be patient.';

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/refresh_match_list', {
      method: 'POST',
      body: buffer,
    });

    if (res.ok) {
      // We successfully submitted the data.
      this.progressMessage = 'Successfully imported match list.';
    } else {
      this.progressMessage = '';
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
}
