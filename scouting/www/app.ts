import {Component, ElementRef, ViewChild} from '@angular/core';

type Tab = 'Entry'|'ImportMatchList';

@Component({
  selector: 'my-app',
  templateUrl: './app.ng.html',
  styleUrls: ['./common.css']
})
export class App {
  tab: Tab = 'Entry';

  @ViewChild("block_alerts") block_alerts: ElementRef;

  constructor() {
    window.addEventListener('beforeunload', (e) => {
      if (!this.block_alerts.nativeElement.checked) {
        // Based on https://developer.mozilla.org/en-US/docs/Web/API/WindowEventHandlers/onbeforeunload#example
        // This combination ensures a dialog will be shown on most browsers.
        e.preventDefault();
        e.returnValue = '';
      }
    });
  }

  tabIs(tab: Tab) {
    return this.tab == tab;
  }

  switchTabTo(tab: Tab) {
    let shouldSwitch = true;
    if (tab === 'ImportMatchList') {
      shouldSwitch = window.confirm('Leave data scouting page?');
    }
    if (shouldSwitch) {
      this.tab = tab;
    }
  }
}
