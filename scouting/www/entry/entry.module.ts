import {NgModule, Pipe, PipeTransform} from '@angular/core';
import {CommonModule} from '@angular/common';
import {FormsModule} from '@angular/forms';

import {CounterButtonModule} from '../counter_button/counter_button.module';
import {EntryComponent} from './entry.component';

import {ClimbLevel} from 'org_frc971/scouting/webserver/requests/messages/submit_data_scouting_generated';

@Pipe({name: 'levelToString'})
export class LevelToStringPipe implements PipeTransform {
  transform(level: ClimbLevel): string {
    return ClimbLevel[level];
  }
}

@NgModule({
  declarations: [EntryComponent, LevelToStringPipe],
  exports: [EntryComponent],
  imports: [CommonModule, FormsModule, CounterButtonModule],
})
export class EntryModule {
}
