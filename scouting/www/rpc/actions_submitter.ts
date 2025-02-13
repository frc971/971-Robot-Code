import {Injectable} from '@angular/core';
import {RequestAuthorizer} from './auth_handler';

@Injectable({providedIn: 'root'})
export class ActionsSubmitter {
  constructor(private readonly requestAuthorizer: RequestAuthorizer) {}

  async submit(actionBuffer: Uint8Array): Promise<Response> {
    return this.requestAuthorizer.submit(
      '/requests/submit/submit_2025_actions',
      actionBuffer
    );
  }
}
