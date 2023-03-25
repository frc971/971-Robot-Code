import {Injectable} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '../../webserver/requests/messages/error_response_generated';
import {RequestAllNotes} from '../../webserver/requests/messages/request_all_notes_generated';
import {
  Note,
  RequestAllNotesResponse,
} from '../../webserver/requests/messages/request_all_notes_response_generated';
import {RequestAllDriverRankings} from '../../webserver/requests/messages/request_all_driver_rankings_generated';
import {
  Ranking,
  RequestAllDriverRankingsResponse,
} from '../../webserver/requests/messages/request_all_driver_rankings_response_generated';
import {Request2023DataScouting} from '../../webserver/requests/messages/request_2023_data_scouting_generated';
import {
  Stats2023,
  Request2023DataScoutingResponse,
} from '../../webserver/requests/messages/request_2023_data_scouting_response_generated';

@Injectable({providedIn: 'root'})
export class ViewDataRequestor {
  async fetchFromServer(start: Function, end: Function, path: string) {
    const builder = new Builder();
    start(builder);
    builder.finish(end(builder));
    const buffer = builder.asUint8Array();
    const res = await fetch(path, {
      method: 'POST',
      body: buffer,
    });

    const resBuffer = await res.arrayBuffer();
    const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));

    if (!res.ok) {
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);
      const errorMessage = parsedResponse.errorMessage();
      throw `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }

    return fbBuffer;
  }

  // Returns all notes from the database.
  async fetchNoteList(): Promise<Note[]> {
    let fbBuffer = await this.fetchFromServer(
      RequestAllNotes.startRequestAllNotes,
      RequestAllNotes.endRequestAllNotes,
      '/requests/request/all_notes'
    );
    const parsedResponse =
      RequestAllNotesResponse.getRootAsRequestAllNotesResponse(fbBuffer);
    // Convert the flatbuffer list into an array. That's more useful.
    const noteList = [];
    for (let i = 0; i < parsedResponse.noteListLength(); i++) {
      noteList.push(parsedResponse.noteList(i));
    }
    return noteList;
  }
  // Returns all driver ranking entries from the database.
  async fetchDriverRankingList(): Promise<Ranking[]> {
    let fbBuffer = await this.fetchFromServer(
      RequestAllDriverRankings.startRequestAllDriverRankings,
      RequestAllDriverRankings.endRequestAllDriverRankings,
      '/requests/request/all_driver_rankings'
    );

    const parsedResponse =
      RequestAllDriverRankingsResponse.getRootAsRequestAllDriverRankingsResponse(
        fbBuffer
      );
    // Convert the flatbuffer list into an array. That's more useful.
    const driverRankingList = [];
    for (let i = 0; i < parsedResponse.driverRankingListLength(); i++) {
      driverRankingList.push(parsedResponse.driverRankingList(i));
    }
    return driverRankingList;
  }
  // Returns all data scouting entries from the database.
  async fetchStats2023List(): Promise<Stats2023[]> {
    let fbBuffer = await this.fetchFromServer(
      Request2023DataScouting.startRequest2023DataScouting,
      Request2023DataScouting.endRequest2023DataScouting,
      '/requests/request/2023_data_scouting'
    );

    const parsedResponse =
      Request2023DataScoutingResponse.getRootAsRequest2023DataScoutingResponse(
        fbBuffer
      );

    // Convert the flatbuffer list into an array. That's more useful.
    const statList = [];
    for (let i = 0; i < parsedResponse.statsListLength(); i++) {
      statList.push(parsedResponse.statsList(i));
    }
    return statList;
  }
}
