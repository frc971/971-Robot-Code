import {CommonModule} from '@angular/common';
import {NgModule} from '@angular/core';
import {FormsModule} from '@angular/forms';
import {ViewComponent} from './view.component';

@NgModule({
  declarations: [ViewComponent],
  exports: [ViewComponent],
  imports: [CommonModule, FormsModule],
})
export class ViewModule {}
