import {Component, ElementRef, ViewChild} from '@angular/core';

type Tab = 'MatchList'|'Notes'|'Entry'|'ImportMatchList';
type TeamInMatch = {
  teamNumber: number,
  matchNumber: number,
  compLevel: string
};

@Component({
  selector: 'my-app',
  templateUrl: './app.ng.html',
  styleUrls: ['./common.css']
})
export class App {
  selectedTeamInMatch:
      TeamInMatch = {teamNumber: 1, matchNumber: 1, compLevel: 'qm'};
  tab: Tab = 'MatchList';

  @ViewChild('block_alerts') block_alerts: ElementRef;

  constructor() {
    window.addEventListener('beforeunload', (e) => {
      if (!this.block_alerts.nativeElement.checked) {
        // Based on
        // https://developer.mozilla.org/en-US/docs/Web/API/WindowEventHandlers/onbeforeunload#example
        // This combination ensures a dialog will be shown on most browsers.
        e.preventDefault();
        e.returnValue = '';
      }
    });
  }

  tabIs(tab: Tab) {
    return this.tab == tab;
  }

  selectTeamInMatch(teamInMatch: TeamInMatch) {
    this.selectedTeamInMatch = teamInMatch;
    this.switchTabTo('Entry');
  }

  switchTabToGuarded(tab: Tab) {
    let shouldSwitch = true;
    if (this.tab !== tab) {
      if (!this.block_alerts.nativeElement.checked) {
        shouldSwitch = window.confirm('Leave current page?');
      }
    }
    if (shouldSwitch) {
      this.switchTabTo(tab);
    }
  }

  private switchTabTo(tab: Tab) {
    this.tab = tab;
  }
}
