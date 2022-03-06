import {NgModule} from '@angular/core';
import {BrowserModule} from '@angular/platform-browser';
import {BrowserAnimationsModule} from '@angular/platform-browser/animations';
import {EntryModule} from './entry/entry.module';

import {App} from './app';

@NgModule({
  declarations: [App],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    EntryModule,
  ],
  exports: [App],
  bootstrap: [App],
})
export class AppModule {
}
