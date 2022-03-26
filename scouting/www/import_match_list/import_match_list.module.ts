import {NgModule} from '@angular/core';
import {CommonModule} from '@angular/common';
import {ImportMatchListComponent} from './import_match_list.component';
import {FormsModule} from '@angular/forms';

@NgModule({
  declarations: [ImportMatchListComponent],
  exports: [ImportMatchListComponent],
  imports: [CommonModule, FormsModule],
})
export class ImportMatchListModule {}
