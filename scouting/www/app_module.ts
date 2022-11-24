import {NgModule} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';
import {BrowserAnimationsModule} from '@angular/platform-browser/animations';

import {App} from './app';
import {EntryModule} from './entry/entry.module';
import {ImportMatchListModule} from './import_match_list/import_match_list.module';
import {MatchListModule} from './match_list/match_list.module';
import {NotesModule} from './notes/notes.module';
import {ShiftScheduleModule} from './shift_schedule/shift_schedule.module';
import {ViewModule} from './view/view.module';
import {DriverRankingModule} from './driver_ranking/driver_ranking.module';

@NgModule({
  declarations: [App],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    EntryModule,
    NotesModule,
    ImportMatchListModule,
    MatchListModule,
    ShiftScheduleModule,
    DriverRankingModule,
    ViewModule,
  ],
  exports: [App],
  bootstrap: [App],
})
export class AppModule {}
