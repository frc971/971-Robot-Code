import {Injectable} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '@org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {RequestAllNotes2025} from '@org_frc971/scouting/webserver/requests/messages/request_all_notes_2025_generated';
import {
  Note2025,
  RequestAllNotes2025Response,
} from '@org_frc971/scouting/webserver/requests/messages/request_all_notes_2025_response_generated';
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
import {Request2025DataScouting} from '@org_frc971/scouting/webserver/requests/messages/request_2025_data_scouting_generated';
import {
  PitImage,
  RequestAllPitImagesResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_all_pit_images_response_generated';
import {RequestAllPitImages} from '@org_frc971/scouting/webserver/requests/messages/request_all_pit_images_generated';
import {
  Stats2025,
  Request2025DataScoutingResponse,
} from '@org_frc971/scouting/webserver/requests/messages/request_2025_data_scouting_response_generated';

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
  async fetchNote2025List(comp_code: string): Promise<Note2025[]> {
    const builder = new Builder();

    builder.finish(
      RequestAllNotes2025.createRequestAllNotes2025(
        builder,
        builder.createString(comp_code)
      )
    );

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/request/all_notes_2025', {
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
      RequestAllNotes2025Response.getRootAsRequestAllNotes2025Response(
        fbBuffer
      );
    // Convert the flatbuffer list into an array. That's more useful.
    const note2025List = [];
    for (let i = 0; i < parsedResponse.note2025ListLength(); i++) {
      note2025List.push(parsedResponse.note2025List(i));
    }
    return note2025List;
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
  async fetchStats2025List(comp_code: string): Promise<Stats2025[]> {
    const builder = new Builder();

    builder.finish(
      Request2025DataScouting.createRequest2025DataScouting(
        builder,
        builder.createString(comp_code)
      )
    );

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/request/2025_data_scouting', {
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
      Request2025DataScoutingResponse.getRootAsRequest2025DataScoutingResponse(
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
