import {NgModule} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';
import {BrowserAnimationsModule} from '@angular/platform-browser/animations';

import {App} from './app';
import {EntryModule} from '../entry';
import {MatchListModule} from '../match_list';
import {NotesModule} from '../notes';
import {ShiftScheduleModule} from '../shift_schedule';
import {ViewModule} from '../view';
import {DriverRankingModule} from '../driver_ranking';
import {PitScoutingModule} from '../pit_scouting';

@NgModule({
  declarations: [App],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    EntryModule,
    NotesModule,
    MatchListModule,
    ShiftScheduleModule,
    DriverRankingModule,
    ViewModule,
    PitScoutingModule,
  ],
  exports: [App],
  bootstrap: [App],
})
export class AppModule {}
