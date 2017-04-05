package de.weis.multisensor_grabber;

import android.os.Bundle;
import android.preference.PreferenceActivity;

/**
 * Created by weis on 27.12.16.
 */

public class SettingsActivity extends PreferenceActivity {
  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    // Display the fragment as the main content.
    getFragmentManager().beginTransaction()
        .replace(android.R.id.content, new SettingsFragment())
        .commit();
  }
}
