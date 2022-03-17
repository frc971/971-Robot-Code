import {CommonModule} from '@angular/common';
import {NgModule} from '@angular/core';
import {FormsModule} from '@angular/forms';

import {MatchListComponent} from './match_list.component';

@NgModule({
  declarations: [MatchListComponent],
  exports: [MatchListComponent],
  imports: [CommonModule, FormsModule],
})
export class MatchListModule {
}
