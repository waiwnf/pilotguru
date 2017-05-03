package ru.pilotguru.recorder;

import android.app.Activity;
import android.content.Context;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.os.Build;
import android.os.Bundle;
import android.text.Html;
import android.text.method.ScrollingMovementMethod;
import android.widget.TextView;

import java.util.Locale;
import java.util.Set;
import java.util.TreeSet;

import static android.hardware.camera2.CameraCharacteristics.CONTROL_AF_AVAILABLE_MODES;
import static android.hardware.camera2.CameraCharacteristics.CONTROL_AWB_AVAILABLE_MODES;
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


/**
 * Created by weis on 29.12.16.
 */

public class ProbeActivity extends Activity {
  TextView probeResultTextView;

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_probe);
    probeResultTextView = (TextView) findViewById(R.id.textview_probe);
    probeResultTextView.setText("Probing...");

    final CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
    String cameraId = null;
    try {
      cameraId = manager.getCameraIdList()[0];
      final CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
      startProbe(characteristics);
    } catch (CameraAccessException e) {
      e.printStackTrace();
    }
  }

  private static String htmlLine(String a, String b) {
    return a + b + "<br/>";
  }

  public void startProbe(CameraCharacteristics characteristics) {
    final String result = model() + supportedHardwareLevel(characteristics) +
        availableAutoExposureModes(characteristics) + availableAutoFocusModes(characteristics) +
        availableAwbModes(characteristics);
    if (android.os.Build.VERSION.SDK_INT < 24) {
      probeResultTextView.setText(Html.fromHtml(result));
    } else {
      probeResultTextView.setText(Html.fromHtml(result, Html.FROM_HTML_MODE_LEGACY));
    }
    probeResultTextView.setMovementMethod(new ScrollingMovementMethod());
  }

  public String model() {
    return "<h2>Model</h2>" + htmlLine("Model: ", Build.MODEL) +
        htmlLine("Manufacturer: ", Build.MANUFACTURER) +
        htmlLine("Build version: ", android.os.Build.VERSION.RELEASE) +
        htmlLine("SDK version: ", Integer.toString(android.os.Build.VERSION.SDK_INT));
  }

  private static String boolToColor(boolean b) {
    return b ? "#00ff00" : "#ff0000";
  }

  private static String coloredText(boolean b, String text) {
    return String.format(Locale.US, "<font color = \"%s\">%s</font><br>", boolToColor(b), text);
  }

  private static Set<Integer> intArrayToSet(int[] inArray) {
    final Set<Integer> result = new TreeSet();
    for (final int v : inArray) {
      result.add(v);
    }
    return result;
  }

  public String supportedHardwareLevel(CameraCharacteristics characteristics) {
    final Integer myLevel =
        characteristics.get(CameraCharacteristics.INFO_SUPPORTED_HARDWARE_LEVEL);
    return "<h2>Hardware Level Support Category</h2>" +
        coloredText(INFO_SUPPORTED_HARDWARE_LEVEL_FULL == myLevel, "Full") +
        coloredText(INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED == myLevel, "Limited") +
        coloredText(INFO_SUPPORTED_HARDWARE_LEVEL_LEGACY == myLevel, "Legacy");
  }

  public String availableAwbModes(CameraCharacteristics characteristics) {
    final Set<Integer> available = intArrayToSet(characteristics.get(CONTROL_AWB_AVAILABLE_MODES));

    return "<h2>Whitebalance</h2>" +
        coloredText(available.contains(CONTROL_AWB_MODE_OFF), "Whitebalance off") +
        coloredText(available.contains(CONTROL_AWB_MODE_AUTO), "Automatic whitebalance") +
        coloredText(available.contains(CONTROL_AWB_MODE_CLOUDY_DAYLIGHT), "WB: cloudy day") +
        coloredText(available.contains(CONTROL_AWB_MODE_DAYLIGHT), "WB: day") +
        coloredText(available.contains(CONTROL_AWB_MODE_FLUORESCENT), "WB: fluorescent") +
        coloredText(available.contains(CONTROL_AWB_MODE_INCANDESCENT), "WB: incandescent") +
        coloredText(available.contains(CONTROL_AWB_MODE_SHADE), "WB: shade") +
        coloredText(available.contains(CONTROL_AWB_MODE_TWILIGHT), "WB: twilight") +
        coloredText(available.contains(CONTROL_AWB_MODE_WARM_FLUORESCENT), "WB: warm fluorescent");
  }

  public String availableAutoFocusModes(CameraCharacteristics characteristics) {
    final Set<Integer> available = intArrayToSet(characteristics.get(CONTROL_AF_AVAILABLE_MODES));

    return "<h2>Focus</h2>" + coloredText(available.contains(CONTROL_AF_MODE_OFF), "Manual focus") +
        coloredText(available.contains(CONTROL_AF_MODE_AUTO), "Auto focus") +
        coloredText(available.contains(CONTROL_AF_MODE_MACRO), "Auto focus macro") +
        coloredText(available.contains(CONTROL_AF_MODE_CONTINUOUS_PICTURE),
            "Auto focus continuous picture") +
        coloredText(available.contains(CONTROL_AF_MODE_CONTINUOUS_VIDEO),
            "Auto focus continuous video") +
        coloredText(available.contains(CONTROL_AF_MODE_EDOF), "Auto focus EDOF");

  }

  public String availableAutoExposureModes(CameraCharacteristics characteristics) {
    final Set<Integer> available = intArrayToSet(characteristics.get(CONTROL_AE_AVAILABLE_MODES));

    return "<h2>Exposure</h2>" +
        coloredText(available.contains(CONTROL_AE_MODE_OFF), "Manual exposure") +
        coloredText(available.contains(CONTROL_AE_MODE_ON), "Auto exposure") +
        coloredText(available.contains(CONTROL_AE_MODE_ON_ALWAYS_FLASH),
            "Auto exposure, always flash") +
        coloredText(available.contains(CONTROL_AE_MODE_ON_AUTO_FLASH),
            "Auto exposure, auto flash") +
        coloredText(available.contains(CONTROL_AE_MODE_ON_AUTO_FLASH_REDEYE),
            "Auto exposure, auto flash redeye");
  }
}
