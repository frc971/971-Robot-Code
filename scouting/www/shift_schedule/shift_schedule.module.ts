import {CommonModule} from '@angular/common';
import {NgModule} from '@angular/core';
import {FormsModule} from '@angular/forms';

import {ShiftsComponent} from './shift_schedule.component';

@NgModule({
  declarations: [ShiftsComponent],
  exports: [ShiftsComponent],
  imports: [CommonModule, FormsModule],
})
export class ShiftScheduleModule {}
