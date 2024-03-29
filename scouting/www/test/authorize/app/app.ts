import {Component} from '@angular/core';

import {RequestAuthorizer} from '@org_frc971/scouting/www/rpc';

@Component({
  selector: 'test-app',
  templateUrl: './app.ng.html',
})
export class App {
  message: string = 'Waiting for button click';

  constructor(private readonly requestAuthorizer: RequestAuthorizer) {}

  // A dummy request submission that is expected to fail initially.
  async performSubmissionWithoutReAuth() {
    this.performSubmission(() => {
      return fetch('/submit', {method: 'POST', body: new Uint8Array()});
    });
  }

  // A dummy request that is expected to authorize itself if it hits a 401 error.
  async performSubmissionWithReAuth() {
    this.performSubmission(() => {
      return this.requestAuthorizer.submit('/submit', new Uint8Array());
    });
  }

  async performSubmission(callback: () => Promise<Response>) {
    this.message = 'Starting submission';
    try {
      const result = await callback();
      if (result.status != 200) {
        this.message = `Failed to perform submission: ${result.status}!`;
      } else {
        this.message = await result.text();
      }
    } catch (error) {
      this.message = error;
    }
  }
}
