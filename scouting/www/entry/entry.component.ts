import { Component, OnInit } from '@angular/core';

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
            this.section = 'Home';
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
}
