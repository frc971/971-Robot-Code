import { Component, OnInit } from '@angular/core';

import * as flatbuffer_builder from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';
import * as error_response from 'org_frc971/scouting/webserver/requests/messages/error_response_generated';
import * as submit_data_scouting_response from 'org_frc971/scouting/webserver/requests/messages/submit_data_scouting_response_generated';
import * as submit_data_scouting from 'org_frc971/scouting/webserver/requests/messages/submit_data_scouting_generated';
import SubmitDataScouting = submit_data_scouting.scouting.webserver.requests.SubmitDataScouting;
import SubmitDataScoutingResponse = submit_data_scouting_response.scouting.webserver.requests.SubmitDataScoutingResponse;
import ErrorResponse = error_response.scouting.webserver.requests.ErrorResponse;

type Section = 'Team Selection'|'Auto'|'TeleOp'|'Climb'|'Defense'|'Review and Submit'|'Home'
type Level = 'Low'|'Medium'|'High'|'Transversal'

@Component({
    selector: 'app-entry',
    templateUrl: './entry.ng.html',
    styleUrls: ['./entry.component.css']
})
export class EntryComponent {
    section: Section = 'Team Selection';
    matchNumber: number = 1
    teamNumber: number = 1
    autoUpperShotsMade: number = 0;
    autoLowerShotsMade: number = 0;
    autoShotsMissed: number = 0;
    teleUpperShotsMade: number = 0;
    teleLowerShotsMade: number = 0;
    teleShotsMissed: number = 0;
    defensePlayedOnScore: number = 3;
    defensePlayedScore: number = 3;
    level: Level;
    proper: boolean = false;
    climbed: boolean = false;
    errorMessage: string = '';

    toggleProper() {
        this.proper = !this.proper;
    }

    setLow() {
        this.level = 'Low';
    }

    setMedium() {
        this.level = 'Medium';
    }

    setHigh() {
        this.level = 'High';
    }

    setTransversal() {
        this.level = 'Transversal';
    }

    defensePlayedOnSlider(event) {
        this.defensePlayedOnScore = event.target.value;
    }

    defensePlayedSlider(event) {
        this.defensePlayedScore = event.target.value;
    }

    setClimbedTrue() {
        this.climbed = true;
    }

    setClimbedFalse() {
        this.climbed = false;
    }

    nextSection() {
        if (this.section === 'Team Selection') {
            this.section = 'Auto';
        } else if (this.section === 'Auto') {
            this.section = 'TeleOp';
        } else if (this.section === 'TeleOp') {
            this.section = 'Climb';
        } else if (this.section === 'Climb') {
            this.section = 'Defense';
        } else if (this.section === 'Defense') {
            this.section = 'Review and Submit';
        } else if (this.section === 'Review and Submit') {
            this.submitDataScouting();
        }
    }

    prevSection() {
      if (this.section === 'Auto') {
        this.section = 'Team Selection';
      } else if (this.section === 'TeleOp') {
        this.section = 'Auto';
      } else if (this.section === 'Climb') {
        this.section = 'TeleOp';
      } else if (this.section === 'Defense') {
        this.section = 'Climb';
      } else if (this.section === 'Review and Submit') {
        this.section = 'Defense';
      }
    }

    adjustAutoUpper(by: number) {
        this.autoUpperShotsMade = Math.max(0, this.autoUpperShotsMade + by);
    }

    adjustAutoLower(by: number) {
        this.autoLowerShotsMade = Math.max(0, this.autoLowerShotsMade + by);
    }

    adjustAutoMissed(by: number) {
        this.autoShotsMissed = Math.max(0, this.autoShotsMissed + by);
    }

    adjustTeleUpper(by: number) {
        this.teleUpperShotsMade = Math.max(0, this.teleUpperShotsMade + by);
    }

    adjustTeleLower(by: number) {
        this.teleLowerShotsMade = Math.max(0, this.teleLowerShotsMade + by);
    }

    adjustTeleMissed(by: number) {
        this.teleShotsMissed = Math.max(0, this.teleShotsMissed + by);
    }

    async submitDataScouting() {
        const builder = new flatbuffer_builder.Builder() as unknown as flatbuffers.Builder;
        SubmitDataScouting.startSubmitDataScouting(builder);
        SubmitDataScouting.addTeam(builder, this.teamNumber);
        SubmitDataScouting.addMatch(builder, this.matchNumber);
        SubmitDataScouting.addMissedShotsAuto(builder, this.autoShotsMissed);
        SubmitDataScouting.addUpperGoalAuto(builder, this.autoUpperShotsMade);
        SubmitDataScouting.addLowerGoalAuto(builder, this.autoLowerShotsMade);
        SubmitDataScouting.addMissedShotsTele(builder, this.teleShotsMissed);
        SubmitDataScouting.addUpperGoalTele(builder, this.teleUpperShotsMade);
        SubmitDataScouting.addLowerGoalTele(builder, this.teleLowerShotsMade);
        SubmitDataScouting.addDefenseRating(builder, this.defensePlayedScore);
        // TODO(phil): Add support for defensePlayedOnScore.
        // TODO(phil): Fix the Climbing score.
        SubmitDataScouting.addClimbing(builder, 1);
        builder.finish(SubmitDataScouting.endSubmitDataScouting(builder));

        const buffer = builder.asUint8Array();
        const res = await fetch(
            '/requests/submit/data_scouting', {method: 'POST', body: buffer});

        if (res.ok) {
            // We successfully submitted the data. Go back to Home.
            this.section = 'Home';
        } else {
            const resBuffer = await res.arrayBuffer();
            const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
            const parsedResponse = ErrorResponse.getRootAsErrorResponse(
                fbBuffer as unknown as flatbuffers.ByteBuffer);

            const errorMessage = parsedResponse.errorMessage();
            this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
        }
    }
}
