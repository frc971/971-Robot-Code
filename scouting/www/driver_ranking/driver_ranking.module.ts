import {CommonModule} from '@angular/common';
import {NgModule} from '@angular/core';
import {FormsModule} from '@angular/forms';
import {DriverRankingComponent} from './driver_ranking.component';

@NgModule({
  declarations: [DriverRankingComponent],
  exports: [DriverRankingComponent],
  imports: [CommonModule, FormsModule],
})
export class DriverRankingModule {}
