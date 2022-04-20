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
        // First sort by match type. E.g. finals are last.
        const aMatchTypeIndex = MATCH_TYPE_ORDERING.indexOf(a.compLevel());
        const bMatchTypeIndex = MATCH_TYPE_ORDERING.indexOf(b.compLevel());
        if (aMatchTypeIndex < bMatchTypeIndex) {
          return -1;
        }
        if (aMatchTypeIndex > bMatchTypeIndex) {
          return 1;
        }
        // Then sort by match number. E.g. in semi finals, all match 1 rounds
        // are done first. Then come match 2 rounds. And then, if necessary,
        // the match 3 rounds.
        const aMatchNumber = a.matchNumber();
        const bMatchNumber = b.matchNumber();
        if (aMatchNumber < bMatchNumber) {
          return -1;
        }
        if (aMatchNumber > bMatchNumber) {
          return 1;
        }
        // Lastly, sort by set number. I.e. Semi Final 1 Match 1 happens first.
        // Then comes Semi Final 2 Match 1. Then comes Semi Final 1 Match 2. Then
        // Semi Final 2 Match 2.
        const aSetNumber = a.setNumber();
        const bSetNumber = b.setNumber();
        if (aSetNumber < bSetNumber) {
          return -1;
        }
        if (aSetNumber > bSetNumber) {
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
