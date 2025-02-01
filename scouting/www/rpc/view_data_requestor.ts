import {Injectable} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '@org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {RequestAllNotes} from '@org_frc971/scouting/webserver/requests/messages/request_all_notes_generated';
import {
  Note,
  RequestAllNotesResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_all_notes_response_generated';
import {RequestAveragedDriverRankings2025} from '@org_frc971/scouting/webserver/requests/messages/request_averaged_driver_rankings_2025_generated';
import {
  DriverRanking2025,
  RequestAveragedDriverRankings2025Response,
} from '@org_frc971/scouting/webserver/requests/messages/request_averaged_driver_rankings_2025_response_generated';
import {RequestAveragedHumanRankings2025} from '@org_frc971/scouting/webserver/requests/messages/request_averaged_human_rankings_2025_generated';
import {
  HumanRanking2025,
  RequestAveragedHumanRankings2025Response,
} from '@org_frc971/scouting/webserver/requests/messages/request_averaged_human_rankings_2025_response_generated';
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
  // Returns driver ranking entries from the database.
  async fetchDriverRanking2025List(
    comp_code: string
  ): Promise<DriverRanking2025[]> {
    const builder = new Builder();

    builder.finish(
      RequestAveragedDriverRankings2025.createRequestAveragedDriverRankings2025(
        builder,
        builder.createString(comp_code)
      )
    );

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/request/averaged_driver_rankings_2025', {
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

    const parsedResponse =
      RequestAveragedDriverRankings2025Response.getRootAsRequestAveragedDriverRankings2025Response(
        fbBuffer
      );
    // Convert the flatbuffer list into an array. That's more useful.
    const driverRankingList = [];
    for (let i = 0; i < parsedResponse.rankings2025ListLength(); i++) {
      driverRankingList.push(parsedResponse.rankings2025List(i));
    }
    return driverRankingList;
  }

  // Returns human ranking entries from the database.
  async fetchHumanRanking2025List(
    comp_code: string
  ): Promise<HumanRanking2025[]> {
    const builder = new Builder();

    builder.finish(
      RequestAveragedHumanRankings2025.createRequestAveragedHumanRankings2025(
        builder,
        builder.createString(comp_code)
      )
    );

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/request/averaged_human_rankings_2025', {
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

    const parsedResponse =
      RequestAveragedHumanRankings2025Response.getRootAsRequestAveragedHumanRankings2025Response(
        fbBuffer
      );
    // Convert the flatbuffer list into an array. That's more useful.
    const humanRankingList = [];
    for (let i = 0; i < parsedResponse.rankings2025ListLength(); i++) {
      humanRankingList.push(parsedResponse.rankings2025List(i));
    }
    return humanRankingList;
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
