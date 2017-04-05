package de.weis.multisensor_grabber;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.text.Html;
import android.text.method.ScrollingMovementMethod;
import android.util.Pair;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static android.hardware.camera2.CameraCharacteristics.CONTROL_AE_LOCK_AVAILABLE;
import static android.hardware.camera2.CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES;
import static android.hardware.camera2.CameraCharacteristics.CONTROL_AWB_AVAILABLE_MODES;
import static android.hardware.camera2.CameraCharacteristics.CONTROL_AWB_LOCK_AVAILABLE;
import static android.hardware.camera2.CameraMetadata.CONTROL_AE_ANTIBANDING_MODE_AUTO;
import static android.hardware.camera2.CameraMetadata.CONTROL_AE_MODE_OFF;
import static android.hardware.camera2.CameraCharacteristics.CONTROL_AE_AVAILABLE_MODES;
import static android.hardware.camera2.CameraMetadata.CONTROL_AE_MODE_ON;
import static android.hardware.camera2.CameraMetadata.CONTROL_AE_MODE_ON_ALWAYS_FLASH;
import static android.hardware.camera2.CameraMetadata.CONTROL_AE_MODE_ON_AUTO_FLASH;
import static android.hardware.camera2.CameraMetadata.CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE;
import static android.hardware.camera2.CameraMetadata.CONTROL_AF_MODE_AUTO;
import static android.hardware.camera2.CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_PICTURE;
import static android.hardware.camera2.CameraMetadata.CONTROL_AF_MODE_CONTINUOUS_VIDEO;
import static android.hardware.camera2.CameraMetadata.CONTROL_AF_MODE_EDOF;
import static android.hardware.camera2.CameraMetadata.CONTROL_AF_MODE_MACRO;
import static android.hardware.camera2.CameraMetadata.CONTROL_AF_MODE_OFF;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_AUTO;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_CLOUDY_DAYLIGHT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_DAYLIGHT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_FLUORESCENT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_INCANDESCENT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_OFF;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_SHADE;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_TWILIGHT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_WARM_FLUORESCENT;
import static android.hardware.camera2.CameraMetadata.INFO_SUPPORTED_HARDWARE_LEVEL_FULL;
import static android.hardware.camera2.CameraMetadata.INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY;
import static android.hardware.camera2.CameraMetadata.INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
import static java.lang.System.in;


/**
 * Created by weis on 29.12.16.
 */

public class ProbeActivity extends Activity {
  android.hardware.camera2.CameraManager manager;
  CameraCharacteristics characteristics = null;
  SharedPreferences prefs;

  TextView tv;
  String result = "Probing...";

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_probe);
    tv = (TextView) findViewById(R.id.textview_probe);
    tv.setText(result);

    manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
    String cameraId = null;
    try {
      cameraId = manager.getCameraIdList()[0];
      characteristics = manager.getCameraCharacteristics(cameraId);
    } catch (CameraAccessException e) {
      e.printStackTrace();
    }

    startProbe();
  }

  public void startProbe() {
    result = "";
    model();
    general();
    ae();
    af();
    awb();
    tv.setText(Html.fromHtml(result, Html.FROM_HTML_MODE_LEGACY));
    tv.setMovementMethod(new ScrollingMovementMethod());
  }

  public void model() {
    result += "<h2>Model</h2>";
    result += "Model: " + Build.MODEL + "<br>";
    result += "Manufacturer: " + Build.MANUFACTURER + "<br>";
    result += "Build version: " + android.os.Build.VERSION.RELEASE + "<br>";
    result += "SDK version: " + android.os.Build.VERSION.SDK_INT + "<br>";
  }

  private static String boolToColor(boolean b) {
    return b ? "#00ff00" : "#ff0000";
  }

  private static String coloredText(boolean b, String text) {
    return String.format(
        Locale.US, "<font color = \"%s\">%s</font><br>",
        boolToColor(b),
        text);
  }

  public void general() {
    result += "<h2>Hardware Level Support Category</h2>";
    Integer mylevel = characteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL);
    List<Pair> levels = new ArrayList<>();
    levels.add(new Pair<>(INFO_SUPPORTED_HARDWARE_LEVEL_FULL, "Full"));
    levels.add(new Pair<>(INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED, "Limited"));
    levels.add(new Pair<>(INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY, "Legacy"));

    for (Pair<Integer, String> l : levels) {
      result += coloredText(l.first.equals(mylevel), l.second);
    }
  }

  public void awb() {
    result += "<h2>Whitebalance</h2>";
    List<Pair> ml = new ArrayList<>();
    ml.add(new Pair<>(CONTROL_AWB_MODE_OFF, "Whitebalance off"));
    ml.add(new Pair<>(CONTROL_AWB_MODE_AUTO, "Automatic whitebalance"));
    ml.add(new Pair<>(CONTROL_AWB_MODE_CLOUDY_DAYLIGHT, "WB: cloudy day"));
    ml.add(new Pair<>(CONTROL_AWB_MODE_DAYLIGHT, "WB: day"));
    ml.add(new Pair<>(CONTROL_AWB_MODE_FLUORESCENT, "WB: fluorescent"));
    ml.add(new Pair<>(CONTROL_AWB_MODE_INCANDESCENT, "WB: incandescent"));
    ml.add(new Pair<>(CONTROL_AWB_MODE_SHADE, "WB: shade"));
    ml.add(new Pair<>(CONTROL_AWB_MODE_TWILIGHT, "WB: twilight"));
    ml.add(new Pair<>(CONTROL_AWB_MODE_WARM_FLUORESCENT, "WB: warm fluorescent"));

    final int[] awbAvailableModes = characteristics.get(CONTROL_AWB_AVAILABLE_MODES);
    List<Integer> aelist = new ArrayList<Integer>();
    for (int index = 0; index < awbAvailableModes.length; index++) {
      aelist.add(awbAvailableModes[index]);
    }

    for (Pair<Integer, String> kv : ml) {
      result += coloredText(aelist.contains(kv.first), kv.second);
    }

    result += coloredText(characteristics.get(CONTROL_AWB_LOCK_AVAILABLE), "AWB Lock ");
  }

  public void af() {
    result += "<h2>Focus</h2>";
    // not able to get the enum/key names from the ints,
    // so I am doing it myself
    List<Pair> ml = new ArrayList<>();
    ml.add(new Pair<>(CONTROL_AF_MODE_OFF, "Manual focus"));
    ml.add(new Pair<>(CONTROL_AF_MODE_AUTO, "Auto focus"));
    ml.add(new Pair<>(CONTROL_AF_MODE_MACRO, "Auto focus macro"));
    ml.add(new Pair<>(CONTROL_AF_MODE_CONTINUOUS_PICTURE, "Auto focus continuous picture"));
    ml.add(new Pair<>(CONTROL_AF_MODE_CONTINUOUS_VIDEO, "Auto focus continuous video"));
    ml.add(new Pair<>(CONTROL_AF_MODE_EDOF, "Auto focus EDOF"));

    int[] tmp = characteristics.get(CONTROL_AF_AVAILABLE_MODES);
    List<Integer> aelist = new ArrayList<Integer>();
    for (int index = 0; index < tmp.length; index++) {
      aelist.add(tmp[index]);
    }

    for (Pair<Integer, String> kv : ml) {
      result += coloredText(aelist.contains(kv.first), kv.second);
    }
  }

  public void ae() {
    result += "<h2>Exposure</h2>";
    // not able to get the enum/key names from the ints,
    // so I am doing it myself
    List<Pair> ml = new ArrayList<>();
    ml.add(new Pair<>(CONTROL_AE_MODE_OFF, "Manual exposure"));
    ml.add(new Pair<>(CONTROL_AE_MODE_ON, "Auto exposure"));
    ml.add(new Pair<>(CONTROL_AE_MODE_ON_ALWAYS_FLASH, "Auto exposure, always flash"));
    ml.add(new Pair<>(CONTROL_AE_MODE_ON_AUTO_FLASH, "Auto exposure, auto flash"));
    ml.add(new Pair<>(CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE, "Auto exposure, auto flash redeye"));

    int[] tmp = characteristics.get(CONTROL_AE_AVAILABLE_MODES);
    List<Integer> aelist = new ArrayList<Integer>();
    for (int index = 0; index < tmp.length; index++) {
      aelist.add(tmp[index]);
    }

    for (Pair<Integer, String> kv : ml) {
      result += coloredText(aelist.contains(kv.first), kv.second);
    }

    result += coloredText(characteristics.get(CONTROL_AE_LOCK_AVAILABLE), "AE Lock");
  }
}
