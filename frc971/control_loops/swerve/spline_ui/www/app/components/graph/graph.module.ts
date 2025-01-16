import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { FormsModule } from '@angular/forms';
import { Graph } from './graph';

@NgModule({
  declarations: [Graph],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    FormsModule,
  ],
  exports: [Graph],
  bootstrap: [Graph],
})
export class GraphModule { }