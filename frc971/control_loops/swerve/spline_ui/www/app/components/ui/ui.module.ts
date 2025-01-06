import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { FormsModule } from '@angular/forms';
import { Ui } from './ui';
import { GraphModule } from '../graph/graph.module';
import { NumberFieldModule } from '../numberfield/numberfield.module';

@NgModule({
  declarations: [Ui],
  imports: [
    BrowserModule,
    BrowserAnimationsModule,
    FormsModule,
    GraphModule,
    NumberFieldModule
  ],
  exports: [Ui],
  bootstrap: [Ui],
})
export class UiModule { }
