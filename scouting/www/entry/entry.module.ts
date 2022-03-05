import {NgModule} from '@angular/core';
import {CommonModule} from '@angular/common';
import {EntryComponent} from './entry.component';

@NgModule({
  declarations: [EntryComponent],
  exports: [EntryComponent],
  imports: [CommonModule],
})
export class EntryModule {
}
