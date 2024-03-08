import {NgModule} from '@angular/core';
import {CommonModule} from '@angular/common';
import {ScanComponent} from './scan.component';

@NgModule({
  declarations: [ScanComponent],
  exports: [ScanComponent],
  imports: [CommonModule],
})
export class ScanModule {}
