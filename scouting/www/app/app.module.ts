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
  ],
  exports: [App],
  bootstrap: [App],
})
export class AppModule {}
