const {defineConfig} = require('cypress');

module.exports = defineConfig({
  e2e: {
    specPattern: ['*.cy.js'],
    supportFile: false,
    setupNodeEvents(on, config) {
      on('before:browser:launch', (browser = {}, launchOptions) => {
        launchOptions.args.push('--disable-gpu-shader-disk-cache');
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
