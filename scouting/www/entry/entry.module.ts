import {NgModule} from '@angular/core';
import {CommonModule} from '@angular/common';
import {FormsModule} from '@angular/forms';

import {CounterButtonModule} from '../counter_button/counter_button.module';
import {EntryComponent} from './entry.component';

@NgModule({
  declarations: [EntryComponent],
  exports: [EntryComponent],
  imports: [CommonModule, FormsModule, CounterButtonModule],
})
export class EntryModule {
}
