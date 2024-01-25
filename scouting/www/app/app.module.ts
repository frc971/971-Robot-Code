import {NgModule, isDevMode} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';
import {BrowserAnimationsModule} from '@angular/platform-browser/animations';
import {ServiceWorkerModule} from '@angular/service-worker';

import {App} from './app';
import {EntryModule} from '../entry';
import {MatchListModule} from '../match_list';
import {NotesModule} from '../notes';
import {ShiftScheduleModule} from '../shift_schedule';
import {ViewModule} from '../view';
import {DriverRankingModule} from '../driver_ranking';
import {PitScoutingModule} from '../pit_scouting';
import {ScanModule} from '../scan';

@NgModule({
  declarations: [App],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    ServiceWorkerModule.register('./ngsw-worker.js', {
      enabled: !isDevMode(),
      // Register the ServiceWorker as soon as the application is stable
      // or after 30 seconds (whichever comes first).
      registrationStrategy: 'registerWhenStable:30000',
    }),
    EntryModule,
    NotesModule,
    MatchListModule,
    ShiftScheduleModule,
    DriverRankingModule,
    ViewModule,
    PitScoutingModule,
    ScanModule,
  ],
  exports: [App],
  bootstrap: [App],
})
export class AppModule {}
