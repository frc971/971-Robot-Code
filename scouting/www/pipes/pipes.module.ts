import {NgModule} from '@angular/core';
import {CastPipe} from './cast';

// Export types needed for the public API.
export {CastPipe};

@NgModule({
  declarations: [CastPipe],
  exports: [CastPipe],
  imports: [],
})
export class PipeModule {}
