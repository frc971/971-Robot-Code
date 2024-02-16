import {
  Component,
  ElementRef,
  QueryList,
  ViewChild,
  ViewChildren,
} from '@angular/core';
import {Builder, ByteBuffer} from 'flatbuffers';
import {ErrorResponse} from '../../webserver/requests/messages/error_response_generated';
import {SubmitPitImage} from '../../webserver/requests/messages/submit_pit_image_generated';

type Section = 'TeamSelection' | 'Data';

interface Input {
  teamNumber: string;
  pitImage: HTMLImageElement;
}

@Component({
  selector: 'app-pit-scouting',
  templateUrl: './pit_scouting.ng.html',
  styleUrls: ['../app/common.css', './pit_scouting.component.css'],
})
export class PitScoutingComponent {
  @ViewChild('preview') preview: ElementRef<HTMLImageElement>;
  section: Section = 'Data';

  errorMessage = '';
  teamNumber: string = '1';
  pitImage: string = '';

  async readFile(file): Promise<ArrayBuffer> {
    return new Promise((resolve, reject) => {
      let reader = new FileReader();
      reader.addEventListener('loadend', (e) =>
        resolve(e.target.result as ArrayBuffer)
      );
      reader.addEventListener('error', reject);
      reader.readAsArrayBuffer(file);
    });
  }

  async getImageData() {
    const selectedFile = (<HTMLInputElement>document.getElementById('pitImage'))
      .files[0];
    return new Uint8Array(await this.readFile(selectedFile));
  }

  async setPitImage(event) {
    const src = URL.createObjectURL(event.target.files[0]);
    const previewElement = this.preview.nativeElement;
    previewElement.src = src;
    previewElement.style.display = 'block';
  }

  async submitData() {
    const builder = new Builder();
    const teamNumber = builder.createString(this.teamNumber);
    const pitImage = builder.createString(
      // Remove anything between /s in order to end up with only the file name.
      // Example : path/to/file.txt -> file.txt
      this.pitImage.toString().replace(/^.*[\\\/]/, '')
    );
    const imageData = SubmitPitImage.createImageDataVector(
      builder,
      await this.getImageData()
    );
    builder.finish(
      SubmitPitImage.createSubmitPitImage(
        builder,
        teamNumber,
        pitImage,
        imageData
      )
    );

    const buffer = builder.asUint8Array();
    const res = await fetch('/requests/submit/submit_pit_image', {
      method: 'POST',
      body: buffer,
    });
    if (!res.ok) {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.errorMessage = `Received ${res.status} ${res.statusText}: "${errorMessage}"`;
      return;
    }

    // Reset Data.
    this.section = 'Data';
    this.errorMessage = '';
    this.pitImage = '';
    const previewElement = this.preview.nativeElement;
    previewElement.src = '';
  }
}
