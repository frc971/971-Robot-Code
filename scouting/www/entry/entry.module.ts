import {NgModule, Pipe, PipeTransform} from '@angular/core';
import {CommonModule} from '@angular/common';
import {FormsModule} from '@angular/forms';
import {EntryComponent} from './entry.component';
import {QRCodeModule} from 'angularx-qrcode';

@NgModule({
  declarations: [EntryComponent],
  exports: [EntryComponent],
  imports: [CommonModule, FormsModule, QRCodeModule],
})
export class EntryModule {}
