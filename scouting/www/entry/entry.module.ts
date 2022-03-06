import {NgModule} from '@angular/core';
import {CommonModule} from '@angular/common';
import {EntryComponent} from './entry.component';
import {FormsModule} from '@angular/forms';

@NgModule({
  declarations: [EntryComponent],
  exports: [EntryComponent],
  imports: [CommonModule, FormsModule],
})
export class EntryModule {
}
