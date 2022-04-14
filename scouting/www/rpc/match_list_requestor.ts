import {Injectable} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from 'org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {RequestAllMatches} from 'org_frc971/scouting/webserver/requests/messages/request_all_matches_generated';
import {
  Match,
  RequestAllMatchesResponse,
} from 'org_frc971/scouting/webserver/requests/messages/request_all_matches_response_generated';

const MATCH_TYPE_ORDERING = ['qm', 'ef', 'qf', 'sf', 'f'];

@Injectable({providedIn: 'root'})
export class MatchListRequestor {
  async fetchMatchList(): Promise<Match[]> {
    const builder = new Builder();
    RequestAllMatches.startRequestAllMatches(builder);
    builder.finish(RequestAllMatches.endRequestAllMatches(builder));

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/request/all_matches', {
      method: 'POST',
      body: buffer,
    });

    if (res.ok) {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse =
        RequestAllMatchesResponse.getRootAsRequestAllMatchesResponse(fbBuffer);

      // Convert the flatbuffer list into an array. That's more useful.
      const matchList = [];
      for (let i = 0; i < parsedResponse.matchListLength(); i++) {
        matchList.push(parsedResponse.matchList(i));
      }

      // Sort the list so it is in chronological order.
      matchList.sort((a, b) => {
        const aMatchTypeIndex = MATCH_TYPE_ORDERING.indexOf(a.compLevel());
        const bMatchTypeIndex = MATCH_TYPE_ORDERING.indexOf(b.compLevel());
        if (aMatchTypeIndex < bMatchTypeIndex) {
          return -1;
        }
        if (aMatchTypeIndex > bMatchTypeIndex) {
          return 1;
        }
        const aMatchNumber = a.matchNumber();
        const bMatchNumber = b.matchNumber();
        if (aMatchNumber < bMatchNumber) {
          return -1;
        }
        if (aMatchNumber > bMatchNumber) {
          return 1;
        }
        return 0;
      });

      return matchList;
    } else {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      throw `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
}
