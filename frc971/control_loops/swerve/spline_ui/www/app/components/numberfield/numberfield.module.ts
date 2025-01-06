import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { FormsModule } from '@angular/forms';
import { NumberField } from './numberfield';

@NgModule({
  declarations: [NumberField],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    FormsModule,
  ],
  exports: [NumberField],
  bootstrap: [NumberField],
})
export class NumberFieldModule { }