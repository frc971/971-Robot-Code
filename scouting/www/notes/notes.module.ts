import {NgModule} from '@angular/core';
import {CommonModule} from '@angular/common';
import {FormsModule} from '@angular/forms';

import {Notes} from './notes.component';

@NgModule({
  declarations: [Notes],
  exports: [Notes],
  imports: [CommonModule, FormsModule],
})
export class NotesModule {}
