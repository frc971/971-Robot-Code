import {Injectable} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '@org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {RequestAllNotes} from '@org_frc971/scouting/webserver/requests/messages/request_all_notes_generated';
import {
  Note,
  RequestAllNotesResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_all_notes_response_generated';
import {RequestAllDriverRankings} from '@org_frc971/scouting/webserver/requests/messages/request_all_driver_rankings_generated';
import {
  Ranking,
  RequestAllDriverRankingsResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_all_driver_rankings_response_generated';
import {Request2024DataScouting} from '@org_frc971/scouting/webserver/requests/messages/request_2024_data_scouting_generated';
import {
  PitImage,
  RequestAllPitImagesResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_all_pit_images_response_generated';
import {RequestAllPitImages} from '@org_frc971/scouting/webserver/requests/messages/request_all_pit_images_generated';
import {
  Stats2024,
  Request2024DataScoutingResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_2024_data_scouting_response_generated';

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
  async fetchStats2024List(): Promise<Stats2024[]> {
    let fbBuffer = await this.fetchFromServer(
      Request2024DataScouting.startRequest2024DataScouting,
      Request2024DataScouting.endRequest2024DataScouting,
      '/requests/request/2024_data_scouting'
    );

    const parsedResponse =
      Request2024DataScoutingResponse.getRootAsRequest2024DataScoutingResponse(
        fbBuffer
      );

    // Convert the flatbuffer list into an array. That's more useful.
    const statList = [];
    for (let i = 0; i < parsedResponse.statsListLength(); i++) {
      statList.push(parsedResponse.statsList(i));
    }
    return statList;
  }

  // Returns all pit image entries from the database.
  async fetchPitImagesList(): Promise<PitImage[]> {
    let fbBuffer = await this.fetchFromServer(
      RequestAllPitImages.startRequestAllPitImages,
      RequestAllPitImages.endRequestAllPitImages,
      '/requests/request/all_pit_images'
    );

    const parsedResponse =
      RequestAllPitImagesResponse.getRootAsRequestAllPitImagesResponse(
        fbBuffer
      );

    // Convert the flatbuffer list into an array. That's more useful.
    const pitImageList = [];
    for (let i = 0; i < parsedResponse.pitImageListLength(); i++) {
      pitImageList.push(parsedResponse.pitImageList(i));
    }
    return pitImageList;
  }
}
