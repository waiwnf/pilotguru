package ru.pilotguru.recorder;

import android.app.Activity;
import android.widget.Toast;

public class Errors {
  public static void dieOnException(Activity activity, Exception e, String message) {
    e.printStackTrace();
    Toast.makeText(activity, message, Toast.LENGTH_LONG).show();
    activity.finish();
  }
}
