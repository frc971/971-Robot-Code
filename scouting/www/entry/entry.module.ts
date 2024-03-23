import {NgModule} from '@angular/core';
import {CommonModule} from '@angular/common';
import {FormsModule} from '@angular/forms';
import {EntryComponent} from './entry.component';
import {QRCodeModule} from 'angularx-qrcode';

import {PipeModule} from '@org_frc971/scouting/www/pipes';

@NgModule({
  declarations: [EntryComponent],
  exports: [EntryComponent],
  imports: [PipeModule, CommonModule, FormsModule, QRCodeModule],
})
export class EntryModule {}
