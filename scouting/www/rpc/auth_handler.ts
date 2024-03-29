import {Injectable} from '@angular/core';

const AUTHORIZATION_URL = '/authorize';
const AUTHORIZATION_CHECK_INTERVAL = 500; // ms
const AUTHORIZATION_CHECK_DURATION = 20000; // ms
const AUTHORIZATION_CHECK_ATTEMPTS =
  AUTHORIZATION_CHECK_DURATION / AUTHORIZATION_CHECK_INTERVAL;

@Injectable({providedIn: 'root'})
export class RequestAuthorizer {
  // Submits the buffer to the specified end point.
  async submit(path: string, actionBuffer: Uint8Array): Promise<Response> {
    const result = await this.singleSubmit(path, actionBuffer);
    if (result.status === 401) {
      await this.authorize();
      return await this.singleSubmit(path, actionBuffer);
    } else {
      return result;
    }
  }

  // Actually performs the underlying submission.
  private async singleSubmit(
    path: string,
    actionBuffer: Uint8Array
  ): Promise<Response> {
    return await fetch(path, {
      method: 'POST',
      body: actionBuffer,
    });
  }

  // Open a new tab in an attempt to re-authorize the user with the server.
  private async authorize() {
    const authorizationWindow = window.open(AUTHORIZATION_URL);
    // We should deal with errors opening the tab, but this is good enough for now.

    const tabIsClosed = new Promise<void>((resolve, reject) => {
      let checkCounter = 0;
      const tabCheckTimer = setInterval(() => {
        if (authorizationWindow.closed) {
          clearInterval(tabCheckTimer);
          resolve();
        }
        checkCounter++;
        if (checkCounter >= AUTHORIZATION_CHECK_ATTEMPTS) {
          clearInterval(tabCheckTimer);
          reject();
        }
      }, 500);
    });
    await tabIsClosed;
  }
}
