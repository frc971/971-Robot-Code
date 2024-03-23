import {NgModule, isDevMode} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';
import {BrowserAnimationsModule} from '@angular/platform-browser/animations';
import {ServiceWorkerModule} from '@angular/service-worker';

import {App} from './app';
import {PipeModule} from '@org_frc971/scouting/www/pipes';
import {EntryModule} from '@org_frc971/scouting/www/entry';
import {MatchListModule} from '@org_frc971/scouting/www/match_list';
import {NotesModule} from '@org_frc971/scouting/www/notes';
import {ShiftScheduleModule} from '@org_frc971/scouting/www/shift_schedule';
import {ViewModule} from '@org_frc971/scouting/www/view';
import {DriverRankingModule} from '@org_frc971/scouting/www/driver_ranking';
import {PitScoutingModule} from '@org_frc971/scouting/www/pit_scouting';
import {ScanModule} from '@org_frc971/scouting/www/scan';

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
    PipeModule,
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
