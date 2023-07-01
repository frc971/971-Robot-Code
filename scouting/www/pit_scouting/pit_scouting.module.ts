import {NgModule} from '@angular/core';
import {CommonModule} from '@angular/common';
import {FormsModule} from '@angular/forms';
import {PitScoutingComponent} from './pit_scouting.component';

@NgModule({
  declarations: [PitScoutingComponent],
  exports: [PitScoutingComponent],
  imports: [CommonModule, FormsModule],
})
export class PitScoutingModule {}
