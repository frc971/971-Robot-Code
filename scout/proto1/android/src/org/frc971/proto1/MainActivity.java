package org.frc971.proto1;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.Window;
import android.webkit.JavascriptInterface;
import android.webkit.WebChromeClient;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

public class MainActivity extends Activity {
    private WebView browser = null;

    final class JSHandler {
	// write Info to LogCat
	@JavascriptInterface
	public void info(String str) {
	    Log.i("FRC", str);
	}

	// write Error to LogCat
	@JavascriptInterface
	public void error(String str) {
	    Log.e("FRC", str);
	}

	// Exit the app
	@JavascriptInterface
	public void exitApp() {
	    finish();
	}
    }

    @SuppressLint("SetJavaScriptEnabled") // Don't warn about XSS potential.
    @Override
    protected void onCreate(Bundle savedInstanceState) {
	super.onCreate(savedInstanceState);
	// The web page shows a heading so hide the app title.
	requestWindowFeature(Window.FEATURE_NO_TITLE);

	setContentView(R.layout.activity_main);

	browser = (WebView)findViewById(R.id.webView);
	browser.clearCache(true); // includeDiskFiles

	// Configure the WebView.
        WebSettings settings = browser.getSettings();
        settings.setJavaScriptEnabled(true);

        // Needed for "alert()" to work.
        // TODO: Configure or subclass the WebChromeClient?
        browser.setWebChromeClient(new WebChromeClient());

        // TODO: more setup...
        browser.setWebViewClient(new WebViewClient());

        // Add custom functionality to the JavaScript environment.
        browser.addJavascriptInterface(new JSHandler(), "frc");

        browser.loadUrl("file:///android_asset/index.html");
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
	// Inflate the menu; this adds items to the action bar if it is present.
	getMenuInflater().inflate(R.menu.activity_main, menu);
	return true;
    }

}
