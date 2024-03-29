import {BrowserModule} from '@angular/platform-browser';
import {NgModule} from '@angular/core';

import {App} from './app';

@NgModule({
  declarations: [App],
  imports: [BrowserModule],
  exports: [App],
  bootstrap: [App],
})
export class AppModule {}
