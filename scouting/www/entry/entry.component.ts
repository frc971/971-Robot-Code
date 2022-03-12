import { Component, ElementRef, OnInit, ViewChild } from '@angular/core';
import { FormsModule } from '@angular/forms';

import * as flatbuffer_builder from 'org_frc971/external/com_github_google_flatbuffers/ts/builder';
import {ByteBuffer} from 'org_frc971/external/com_github_google_flatbuffers/ts/byte-buffer';
import * as error_response from 'org_frc971/scouting/webserver/requests/messages/error_response_generated';
import * as submit_data_scouting_response from 'org_frc971/scouting/webserver/requests/messages/submit_data_scouting_response_generated';
import * as submit_data_scouting from 'org_frc971/scouting/webserver/requests/messages/submit_data_scouting_generated';
import SubmitDataScouting = submit_data_scouting.scouting.webserver.requests.SubmitDataScouting;
import SubmitDataScoutingResponse = submit_data_scouting_response.scouting.webserver.requests.SubmitDataScoutingResponse;
import ErrorResponse = error_response.scouting.webserver.requests.ErrorResponse;

type Section = 'Team Selection'|'Auto'|'TeleOp'|'Climb'|'Other'|'Review and Submit'|'Home'
type Level = 'NoAttempt'|'Failed'|'FailedWithPlentyOfTime'|'Low'|'Medium'|'High'|'Transversal'

@Component({
    selector: 'app-entry',
    templateUrl: './entry.ng.html',
    styleUrls: ['../common.css', './entry.component.css']
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
    defensePlayedOnScore: number = 0;
    defensePlayedScore: number = 0;
    level: Level = 'NoAttempt';
    ball1: boolean = false;
    ball2: boolean = false;
    ball3: boolean = false;
    ball4: boolean = false;
    ball5: boolean = false;
    errorMessage: string = '';
    noShow: boolean = false;
    neverMoved: boolean = false;
    batteryDied: boolean = false;
    mechanicallyBroke: boolean = false;
    lostComs: boolean = false;

    @ViewChild("header") header: ElementRef;

    nextSection() {
        if (this.section === 'Team Selection') {
            this.section = 'Auto';
        } else if (this.section === 'Auto') {
            this.section = 'TeleOp';
        } else if (this.section === 'TeleOp') {
            this.section = 'Climb';
        } else if (this.section === 'Climb') {
            this.section = 'Other';
        } else if (this.section === 'Other') {
            this.section = 'Review and Submit';
        } else if (this.section === 'Review and Submit') {
            this.submitDataScouting();
            return;
        }
        // Scroll back to the top so that we can be sure the user sees the
        // entire next screen. Otherwise it's easy to overlook input fields.
        this.scrollToTop();
    }

    prevSection() {
      if (this.section === 'Auto') {
        this.section = 'Team Selection';
      } else if (this.section === 'TeleOp') {
        this.section = 'Auto';
      } else if (this.section === 'Climb') {
        this.section = 'TeleOp';
      } else if (this.section === 'Other') {
        this.section = 'Climb';
      } else if (this.section === 'Review and Submit') {
        this.section = 'Other';
      }
      // Scroll back to the top so that we can be sure the user sees the
      // entire previous screen. Otherwise it's easy to overlook input
      // fields.
      this.scrollToTop();
    }

    private scrollToTop() {
        this.header.nativeElement.scrollIntoView();
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
        SubmitDataScouting.addAutoBall1(builder, this.ball1);
        SubmitDataScouting.addAutoBall2(builder, this.ball2);
        SubmitDataScouting.addAutoBall3(builder, this.ball3);
        SubmitDataScouting.addAutoBall4(builder, this.ball4);
        SubmitDataScouting.addAutoBall5(builder, this.ball5);

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
