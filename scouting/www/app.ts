import {Component} from '@angular/core';

type Tab = 'Entry'|'ImportMatchList';

@Component({
  selector: 'my-app',
  templateUrl: './app.ng.html',
  styleUrls: ['./common.css']
})
export class App {
  tab: Tab = 'Entry';

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
