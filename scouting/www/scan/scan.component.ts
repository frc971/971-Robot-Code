import {Component, NgZone, OnInit, ViewChild, ElementRef} from '@angular/core';
import {ErrorResponse} from '../../webserver/requests/messages/error_response_generated';
import {Builder, ByteBuffer} from 'flatbuffers';
import * as pako from 'pako';

declare var cv: any;
declare var Module: any;

// The number of milliseconds between QR code scans.
const SCAN_PERIOD = 500;

@Component({
  selector: 'app-scan',
  templateUrl: './scan.ng.html',
  styleUrls: ['../app/common.css', './scan.component.css'],
})
export class ScanComponent implements OnInit {
  @ViewChild('video')
  public video: ElementRef;

  @ViewChild('canvas')
  public canvas: ElementRef;

  errorMessage: string = '';
  progressMessage: string = 'Waiting for QR code(s)';
  scanComplete: boolean = false;
  videoStartedSuccessfully = false;

  qrCodeValuePieces: string[] = [];
  qrCodeValuePieceSize = 0;

  scanStream: MediaStream | null = null;
  scanTimer: ReturnType<typeof setTimeout> | null = null;

  constructor(private ngZone: NgZone) {}

  ngOnInit() {
    // If the user switched away from this tab, then the onRuntimeInitialized
    // attribute will already be set. No need to load OpenCV again. If it's not
    // loaded, however, we need to load it.
    if (!Module['onRuntimeInitialized']) {
      Module['onRuntimeInitialized'] = () => {
        // Since the WASM code doesn't know about the Angular zone, we force
        // it into the correct zone so that the UI gets updated properly.
        this.ngZone.run(() => {
          this.startScanning();
        });
      };
      // Now that we set up the hook, we can load OpenCV.
      this.loadOpenCv();
    } else {
      this.startScanning();
    }
  }

  ngOnDestroy() {
    clearInterval(this.scanTimer);

    // Explicitly stop the streams so that the camera isn't being locked
    // unnecessarily. I.e. other processes can use it too.
    if (this.scanStream) {
      this.scanStream.getTracks().forEach((track) => {
        track.stop();
      });
    }
  }

  public ngAfterViewInit() {
    // Start the video playback.
    // It would be nice to let the user select which camera gets used. For now,
    // we give the "environment" hint so that it faces away from the user.
    if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
      navigator.mediaDevices
        .getUserMedia({video: {facingMode: 'environment'}})
        .then(
          (stream) => {
            this.scanStream = stream;
            this.video.nativeElement.srcObject = stream;
            this.video.nativeElement.play();
            this.videoStartedSuccessfully = true;
          },
          (reason) => {
            this.progressMessage = '';
            this.errorMessage = `Failed to start video: ${reason}`;
          }
        );
    }
  }

  async scan() {
    if (!this.videoStartedSuccessfully) {
      return;
    }

    // Take a capture of the video stream. That capture can then be used by
    // OpenCV to perform the QR code detection. Due to my inexperience, I could
    // only make this code work if I size the (invisible) canvas to match the
    // video element. Otherwise, I'd get cropped images.
    // Can we stream the video directly into the canvas?
    const width = this.video.nativeElement.clientWidth;
    const height = this.video.nativeElement.clientHeight;
    this.canvas.nativeElement.width = width;
    this.canvas.nativeElement.height = height;
    this.canvas.nativeElement
      .getContext('2d')
      .drawImage(this.video.nativeElement, 0, 0, width, height);

    // Perform the QR code detection. We use the Aruco-variant of the detector
    // here because it appears to detect QR codes much more reliably than the
    // standard detector.
    let mat = cv.imread('canvas');
    let qrDecoder = new cv.QRCodeDetectorAruco();
    const result = qrDecoder.detectAndDecode(mat);
    mat.delete();

    // Handle the result.
    if (result) {
      await this.scanSuccessHandler(result);
    } else {
      await this.scanFailureHandler();
    }
  }

  async scanSuccessHandler(scanResult: string) {
    // Reverse the conversion and obtain the original Uint8Array. In other
    // words, undo the work in `scouting/www/entry/entry.component.ts`.
    const [indexStr, numPiecesStr, pieceSizeStr, splitPiece] = scanResult.split(
      '_',
      4
    );

    // If we didn't get enough data, then maybe we scanned some non-scouting
    // related QR code? Try to give a hint to the user.
    if (!indexStr || !numPiecesStr || !pieceSizeStr || !splitPiece) {
      this.progressMessage = '';
      this.errorMessage = `Couldn't find scouting data in the QR code.`;
      return;
    }

    const index = Number(indexStr);
    const numPieces = Number(numPiecesStr);
    const pieceSize = Number(pieceSizeStr);

    if (
      numPieces != this.qrCodeValuePieces.length ||
      pieceSize != this.qrCodeValuePieceSize
    ) {
      // The number of pieces or the piece size changed. We need to reset our accounting.
      this.qrCodeValuePieces = new Array<string>(numPieces);
      this.qrCodeValuePieceSize = pieceSize;
    }

    this.qrCodeValuePieces[index] = splitPiece;
    this.progressMessage = `Scanned QR code ${index + 1} out of ${
      this.qrCodeValuePieces.length
    }`;

    // Count up the number of missing pieces so we can give a progress update.
    let numMissingPieces = 0;
    for (const piece of this.qrCodeValuePieces) {
      if (!piece) {
        numMissingPieces++;
      }
    }
    if (numMissingPieces > 0) {
      this.progressMessage = `Waiting for ${numMissingPieces} out of ${this.qrCodeValuePieces.length} QR codes.`;
      this.errorMessage = '';
      return;
    }

    // Stop scanning now that we have all the pieces.
    this.progressMessage = 'Scanned all QR codes. Submitting.';
    this.scanComplete = true;
    clearInterval(this.scanTimer);

    const encodedData = this.qrCodeValuePieces.join('');
    const deflatedData = Uint8Array.from(atob(encodedData), (c) =>
      c.charCodeAt(0)
    );
    const actionBuffer = pako.inflate(deflatedData);

    const res = await fetch('/requests/submit/submit_2024_actions', {
      method: 'POST',
      body: actionBuffer,
    });

    if (res.ok) {
      // We successfully submitted the data. Report success.
      this.progressMessage = 'Success!';
      this.errorMessage = '';
    } else {
      const resBuffer = await res.arrayBuffer();
      const fbBuffer = new ByteBuffer(new Uint8Array(resBuffer));
      const parsedResponse = ErrorResponse.getRootAsErrorResponse(fbBuffer);

      const errorMessage = parsedResponse.errorMessage();
      this.progressMessage = '';
      this.errorMessage = `Submission failed with ${res.status} ${res.statusText}: "${errorMessage}"`;
    }
  }

  async scanFailureHandler() {
    this.progressMessage = '';
    this.errorMessage = 'Failed to scan!';
  }

  loadOpenCv() {
    // Make the browser load OpenCV.
    let body = <HTMLDivElement>document.body;
    let script = document.createElement('script');
    script.innerHTML = '';
    script.src = 'assets/opencv_4.9.0/opencv.js';
    script.async = false;
    script.defer = true;
    script.onerror = (error) => {
      this.progressMessage = '';
      if (typeof error === 'string') {
        this.errorMessage = `OpenCV failed to load: ${error}`;
      } else {
        this.errorMessage = 'OpenCV failed to load.';
      }
      // Since we use the onRuntimeInitialized property as a flag to see if we
      // need to perform loading, we need to delete the property. When the user
      // switches away from this tab and then switches back, we want to attempt
      // loading again.
      delete Module['onRuntimeInitialized'];
    };
    body.appendChild(script);
  }

  startScanning() {
    this.scanTimer = setInterval(() => {
      this.scan();
    }, SCAN_PERIOD);
  }
}
