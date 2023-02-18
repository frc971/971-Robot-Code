import {Component, OnInit} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '../../webserver/requests/messages/error_response_generated';

@Component({
  selector: 'shift-schedule',
  templateUrl: './shift_schedule.ng.html',
  styleUrls: ['../common.css', './shift_schedule.component.css'],
})
export class ShiftsComponent {
  progressMessage: string = '';
  errorMessage: string = '';
  // used to calculate shift blocks from starting match to ending match
  numMatches: number[] = [20, 40, 60, 80, 100, 120, 140, 160, 180, 200];
}
