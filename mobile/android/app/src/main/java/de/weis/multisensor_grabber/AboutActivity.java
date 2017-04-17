package de.weis.multisensor_grabber;

import android.app.Activity;
import android.os.Bundle;
import android.text.Html;
import android.text.method.LinkMovementMethod;
import android.widget.TextView;

import java.util.Locale;

/**
 * Created by weis on 27.12.16.
 */

public class AboutActivity extends Activity {
  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_about);

    final TextView tv = (TextView) findViewById(R.id.textView_about);
    final String aboutText = String.format(Locale.US,
        "<h1>PilotGuru Recorder (%s)</h1><br>" + "<p>Source code: %s" +
            "<p>Based on Multisensor-Grabber: %s" +
            "<p>Records video, video frame timestamps, GPS, accelerometer, and gyroscope data, " +
            "with timestamps in unified space for easy fusion in post-processing.",
        BuildConfig.VERSION_NAME, htmlLink("https://github.com/waiwnf/pilotguru"),
        htmlLink("https://github.com/TobiasWeis/android-multisensorgrabber-2"));
    if (android.os.Build.VERSION.SDK_INT < 24) {
      tv.setText(Html.fromHtml(aboutText));
    } else {
      tv.setText(Html.fromHtml(aboutText, Html.FROM_HTML_MODE_LEGACY));
    }
    tv.setMovementMethod(LinkMovementMethod.getInstance());
  }

  private static String htmlLink(String url) {
    return String.format(Locale.US, "<a href=\"%s\">%s</a>", url, url);
  }
}
