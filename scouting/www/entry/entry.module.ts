import {NgModule, Pipe, PipeTransform} from '@angular/core';
import {CommonModule} from '@angular/common';
import {FormsModule} from '@angular/forms';
import {EntryComponent} from './entry.component';

@NgModule({
  declarations: [EntryComponent],
  exports: [EntryComponent],
  imports: [CommonModule, FormsModule],
})
export class EntryModule {}
