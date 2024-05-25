import {ByteBuffer} from 'flatbuffers'
import {Connection} from '../../aos/network/www/proxy'
import {Status, ApplicationStatus, State, LastStopReason, FileState} from '../../aos/starter/starter_generated'

const NODES = ['/orin1', '/imu', '/roborio'];

export class StarterHandler {
  private statuses = new Map<string, ApplicationStatus[]>();

  private statusList: HTMLElement =
      (document.getElementById('status_list') as HTMLElement);

  constructor(private readonly connection: Connection) {
    for (const node in NODES) {
      this.connection.addConfigHandler(() => {
        this.connection.addHandler(
          NODES[node] + '/aos', 'aos.starter.Status', (data) => {
            this.handleStatus(data, NODES[node]);
          });
      });
    }
  }

  private handleStatus(data: Uint8Array, node: string): void {
    const fbBuffer = new ByteBuffer(data);
    const status: Status = Status.getRootAsStatus(fbBuffer);

    const temp: ApplicationStatus[] = new Array(status.statusesLength());

    for (let i = 0; i < status.statusesLength(); i++) {
      temp[i] = status.statuses(i);
    }

    this.statuses.set(node, temp);
  }

  setStateColor(div: HTMLElement): void {
    div.className = '';
    switch (div.innerHTML) {
      case 'WAITING':
        div.classList.add('yellow');
        break;
      case 'STARTING':
        div.classList.add('yellowgreen');
        break;
      case 'RUNNING':
        div.classList.add('lightgreen');
        break;
      case 'STOPPING':
        div.classList.add('lightcoral');
        break;
      case 'STOPPED':
        div.classList.add('lightcoral');
        break;
    }
  }

  setFileStateColor(div: HTMLElement): void {
    div.className = '';
    switch (div.innerHTML) {
      case 'NOT_RUNNING':
        div.classList.add('lightgreen');
        break;
      case 'NO_CHANGE':
        div.classList.add('lightgreen');
        break;
      case 'CHANGED_DURING_STARTUP':
        div.classList.add('lightcoral');
        break;
      case 'CHANGED':
        div.classList.add('lightcoral');
        break;
    }
  }

  private populateStatusList() : void {
    this.clearStatusList();

    const ELEMENTS: string[] = [
        'name',
        'node',
        'state',
        'last_exit_code',
        'pid',
        // 'id',
        'last_start_time',
        'last_stop_reason',
        // 'process_info',
        // 'has_active_timing_report',
        'file_state'
    ];

    // This is to add the field names as the top row
    //---------------------------------------------------
    const row = document.createElement('div');
    row.className = "column_names";

    for (const e in ELEMENTS) {
      const element = document.createElement('div');
      element.className = ELEMENTS[e];
      element.innerHTML = ELEMENTS[e].toUpperCase();
      element.style.fontWeight = 'bold';
      row.appendChild(element);
    }

    this.statusList.appendChild(row);
    //---------------------------------------------------

    for (const node in NODES) {
      const currentStatus = this.statuses.get(NODES[node]);

      if (currentStatus) {
        currentStatus.forEach(status => {
          const row = document.createElement('div');
          row.className = status.name();

          for (const e in ELEMENTS) {
            const element = document.createElement('div');
            element.className = ELEMENTS[e];

            switch (element.className) {
              case 'name':
                element.innerHTML = status.name();
                break;
              case 'node':
                element.innerHTML = NODES[node];
                break;
              case 'state':
                element.innerHTML = State[status.state()];
                this.setStateColor(element);
                break;
              case 'last_exit_code':
                element.innerHTML = status.lastExitCode().toString();
                break;
              case 'pid':
                element.innerHTML = status.pid().toString();
                break;
              case 'last_start_time':
                element.innerHTML = Number(status.lastStartTime() / 1000000000n).toString() + ' sec';
                break;
              case 'last_stop_reason':
                element.innerHTML = LastStopReason[status.lastStopReason()];
                break;
              case 'file_state':
                element.innerHTML = FileState[status.fileState()];
                this.setFileStateColor(element);
                break;
              default:
                element.innerHTML = "NA";
                break;
            }
            row.appendChild(element);
          }

          this.statusList.appendChild(row);
        })
      }
    }
  }

  private clearStatusList(): void {
    if (!this.statusList) {
        return;
    }

    while (this.statusList.firstChild) {
        this.statusList.removeChild(this.statusList.firstChild);
    }
  }

  draw(): void {
    if (this.statuses) {
      this.populateStatusList();
    }

    window.requestAnimationFrame(() => this.draw());
  }
}
