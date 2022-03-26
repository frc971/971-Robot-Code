import {Component} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from 'org_frc971/scouting/webserver/requests/messages/error_response_generated';
import {RequestNotesForTeam} from 'org_frc971/scouting/webserver/requests/messages/request_notes_for_team_generated';
import {Note as NoteFb, RequestNotesForTeamResponse} from 'org_frc971/scouting/webserver/requests/messages/request_notes_for_team_response_generated';
import {SubmitNotes} from 'org_frc971/scouting/webserver/requests/messages/submit_notes_generated';
import {SubmitNotesResponse} from 'org_frc971/scouting/webserver/requests/messages/submit_notes_response_generated';

type Section = 'TeamSelection'|'Data';

interface Note {
  readonly data: string;
}

@Component({
  selector: 'frc971-notes',
  templateUrl: './notes.ng.html',
  styleUrls: ['../common.css', './notes.component.css']
})
export class Notes {
  section: Section = 'TeamSelection';
  notes: Note[] = [];

  errorMessage = '';

  teamNumber: number = 971;
  newData = '';

  async setTeamNumber() {
    const builder = new Builder();
    RequestNotesForTeam.startRequestNotesForTeam(builder);
    RequestNotesForTeam.addTeam(builder, this.teamNumber);
    builder.finish(RequestNotesForTeam.endRequestNotesForTeam(builder));

    const buffer = builder.asUint8Array();
    const res = await fetch(
        '/requests/request/notes_for_team', {method: 'POST', body: buffer});

    const resBuffer = await res.arrayBuffer();
    const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));

    if (res.ok) {
      this.notes = [];
      const parsedResponse =
          RequestNotesForTeamResponse.getRootAsRequestNotesForTeamResponse(
              fbBuffer);
      for (let i = 0; i < parsedResponse.notesLength(); i++) {
        const fbNote = parsedResponse.notes(i);
        this.notes.push({data: fbNote.data()});
      }
      this.section = 'Data';
    } else {
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage =
          `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }

  changeTeam() {
    this.section = "TeamSelection";
  }
  
  async submitData() {
    const builder = new Builder();
    const dataFb = builder.createString(this.newData);
    builder.finish(
        SubmitNotes.createSubmitNotes(builder, this.teamNumber, dataFb));

    const buffer = builder.asUint8Array();
    const res = await fetch(
        '/requests/submit/submit_notes', {method: 'POST', body: buffer});

    if (res.ok) {
      this.newData = '';
      this.errorMessage = '';
      await this.setTeamNumber();
    } else {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage =
          `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }
}

