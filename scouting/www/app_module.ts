import {NgModule} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';
import {BrowserAnimationsModule} from '@angular/platform-browser/animations';

import {App} from './app';
import {EntryModule} from './entry/entry.module';
import {ImportMatchListModule} from './import_match_list/import_match_list.module';
import {MatchListModule} from './match_list/match_list.module';
import {NotesModule} from './notes/notes.module';

@NgModule({
  declarations: [App],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    EntryModule,
    NotesModule,
    ImportMatchListModule,
    MatchListModule,
  ],
  exports: [App],
  bootstrap: [App],
})
export class AppModule {}
