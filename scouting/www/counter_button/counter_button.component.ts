import {Component, Input, Output, EventEmitter} from '@angular/core';

@Component({
  selector: 'frc971-counter-button',
  templateUrl: './counter_button.ng.html',
  styleUrls: ['./counter_button.component.css'],
})
export class CounterButton {
  @Input() value: number = 0;
  @Output() valueChange = new EventEmitter<number>();

  update(delta: number) {
    this.value = Math.max(this.value + delta, 0);

    this.valueChange.emit(this.value);
  }
}
