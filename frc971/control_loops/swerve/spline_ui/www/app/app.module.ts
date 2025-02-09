import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { FormsModule } from '@angular/forms';
import { UiModule } from './components/ui/ui.module';
import { App } from './app';
import { HttpClientModule } from '@angular/common/http';

@NgModule({
  declarations: [App],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    FormsModule,
    HttpClientModule,
    UiModule
  ],
  exports: [App],
  bootstrap: [App],
})
export class AppModule { }
