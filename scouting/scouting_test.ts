import {browser, by, element} from 'protractor';

class AppPage {
  async navigateTo() {
    await browser.get(browser.baseUrl);
  }

  // Wait for basically forever for these elements to appear.
  // Bazel will manage the timeouts.
  async waitForElement(el, timeout = 1000000) {
    await browser.wait(() => el.isPresent(), timeout);
    await browser.wait(() => el.isDisplayed(), timeout);
    return el;
  }

  async getParagraphText() {
    return (await this.waitForElement(element(by.css('.header')))).getText();
  }
}

describe('The scouting web page', () => {
  let page: AppPage;

  beforeEach(() => {
    page = new AppPage();
  });

  it('should display: This is an app.', async () => {
    await page.navigateTo();
    expect(await page.getParagraphText()).toEqual('Team Selection');
  });
});
