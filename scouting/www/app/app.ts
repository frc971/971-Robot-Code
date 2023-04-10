import {Component, ElementRef, ViewChild} from '@angular/core';

type Tab =
  | 'MatchList'
  | 'Notes'
  | 'Entry'
  | 'DriverRanking'
  | 'ShiftSchedule'
  | 'View';

// Ignore the guard for tabs that don't require the user to enter any data.
const unguardedTabs: Tab[] = ['MatchList'];

type TeamInMatch = {
  teamNumber: number;
  matchNumber: number;
  setNumber: number;
  compLevel: string;
};

@Component({
  selector: 'my-app',
  templateUrl: './app.ng.html',
  styleUrls: ['../app/common.css'],
})
export class App {
  selectedTeamInMatch: TeamInMatch = {
    teamNumber: 1,
    matchNumber: 1,
    setNumber: 1,
    compLevel: 'qm',
  };
  // Keep track of the match list automatically navigating the user to the
  // Entry tab.
  navigatedFromMatchList: boolean = false;
  tab: Tab = 'MatchList';

  @ViewChild('block_alerts') block_alerts: ElementRef;

  constructor() {
    window.addEventListener('beforeunload', (e) => {
      if (!unguardedTabs.includes(this.tab)) {
        if (!this.block_alerts.nativeElement.checked) {
          // Based on
          // https://developer.mozilla.org/en-US/docs/Web/API/WindowEventHandlers/onbeforeunload#example
          // This combination ensures a dialog will be shown on most browsers.
          e.preventDefault();
          e.returnValue = '';
        }
      }
    });
  }

  tabIs(tab: Tab) {
    return this.tab == tab;
  }

  selectTeamInMatch(teamInMatch: TeamInMatch) {
    this.selectedTeamInMatch = teamInMatch;
    this.navigatedFromMatchList = true;
    this.switchTabTo('Entry', false);
  }

  switchTabToGuarded(tab: Tab) {
    let shouldSwitch = true;
    if (this.tab !== tab) {
      if (!unguardedTabs.includes(this.tab)) {
        if (!this.block_alerts.nativeElement.checked) {
          shouldSwitch = window.confirm(
            'Leave current page? You will lose all data.'
          );
        }
      }
    }
    if (shouldSwitch) {
      this.switchTabTo(tab, true);
    }
  }

  private switchTabTo(tab: Tab, wasGuarded: boolean) {
    if (wasGuarded) {
      // When the user navigated between tabs manually, we want to reset some
      // state.
      this.navigatedFromMatchList = false;
    }
    this.tab = tab;
  }
}
