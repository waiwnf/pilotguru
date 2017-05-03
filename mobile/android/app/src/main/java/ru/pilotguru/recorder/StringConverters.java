package ru.pilotguru.recorder;

import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_AUTO;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_CLOUDY_DAYLIGHT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_DAYLIGHT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_FLUORESCENT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_INCANDESCENT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_OFF;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_SHADE;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_TWILIGHT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_WARM_FLUORESCENT;

public class StringConverters {
  public static String whiteBalanceModeToString(int wb) {
    switch (wb) {
      case CONTROL_AWB_MODE_CLOUDY_DAYLIGHT:
        return "Cloudy daylight";
      case CONTROL_AWB_MODE_DAYLIGHT:
        return "Daylight";
      case CONTROL_AWB_MODE_FLUORESCENT:
        return "Fluorescent";
      case CONTROL_AWB_MODE_INCANDESCENT:
        return "Incandescent";
      case CONTROL_AWB_MODE_SHADE:
        return "Shade";
      case CONTROL_AWB_MODE_TWILIGHT:
        return "Twilight";
      case CONTROL_AWB_MODE_WARM_FLUORESCENT:
        return "Warm Fluorescent";
      case CONTROL_AWB_MODE_OFF:
        return "Off";
      case CONTROL_AWB_MODE_AUTO:
        return "Auto";
      case -1:
        return "Not available";
      default:
        return "N/A: " + wb;
    }
  }
}
