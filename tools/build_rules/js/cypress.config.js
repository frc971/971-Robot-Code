import {defineConfig} from 'cypress';

export default defineConfig({
  e2e: {
    specPattern: ['*.cy.js'],
    supportFile: false,
    setupNodeEvents(on, config) {
      on('before:browser:launch', (browser = {}, launchOptions) => {
        launchOptions.args.push('--disable-gpu-shader-disk-cache');
        launchOptions.args.push('--enable-logging');
        launchOptions.args.push('--v=stderr');

        // Point the browser at a video file to use as a webcam. This lets us
        // validate things like QR code scanning.
        launchOptions.args.push('--use-fake-ui-for-media-stream');
        launchOptions.args.push('--use-fake-device-for-media-stream');
        const fakeCameraVideo = `${process.env.TEST_UNDECLARED_OUTPUTS_DIR}/fake_camera.mjpeg`;
        launchOptions.args.push(`--use-file-for-fake-video-capture=${fakeCameraVideo}`);

        return launchOptions;
      });

      // Lets users print to the console:
      //    cy.task('log', 'message here');
      on('task', {
        log(message) {
          console.log(message);
          return null;
        },
      });
    },
  },
});
