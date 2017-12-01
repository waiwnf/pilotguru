package ru.pilotguru.recorder;

import android.hardware.camera2.CaptureResult;
import android.os.StatFs;
import android.support.annotation.NonNull;
import android.widget.TextView;

import java.io.File;
import java.util.Locale;

class CameraInfoIntervalUpdater extends TimeSpacedUpdater {
  private final TextView textViewCamera;
  private final File recordingDir;
  private CaptureResult captureResult = null;

  public CameraInfoIntervalUpdater(long updateIntervalNanos,
      @NonNull TextView textViewCamera,
      @NonNull File recordingDir) {
    super(updateIntervalNanos);
    this.textViewCamera = textViewCamera;
    this.recordingDir = recordingDir;
  }

  private static String getFocalLengthText(CaptureResult result) {
    final boolean isFixedFocusDistance =
        result.get(CaptureResult.CONTROL_AF_MODE) == CaptureResult.CONTROL_AF_MODE_OFF;
    final Float focusDistance = result.get(CaptureResult.LENS_FOCUS_DISTANCE);
    if (isFixedFocusDistance) {
      if (focusDistance != null) {
        return String.format(Locale.US, "Fixed: %.01f", focusDistance);
      } else {
        return "NA";
      }
    } else {
      if (focusDistance != null) {
        return String.format(Locale.US, "Auto: %.01f", focusDistance);
      } else {
        return "Auto";
      }
    }
  }

  private static String getIsoSensitivity(CaptureResult result) {
    final Integer sensitivity = result.get(CaptureResult.SENSOR_SENSITIVITY);
    if (sensitivity != null) {
      return sensitivity.toString();
    } else {
      return "NA";
    }
  }

  public void maybeUpdate(long currentTimeNanos, @NonNull CaptureResult captureResult) {
    this.captureResult = captureResult;
    maybeUpdate(currentTimeNanos);
  }


  @Override
  public void doUpdate(long currentTimeNanos) {
    if (captureResult == null) {
      return;
    }

    final int whiteBalanceMode = captureResult.get(CaptureResult.CONTROL_AWB_MODE);

    final StatFs stat = new StatFs(recordingDir.getPath());
    final long bytesAvailable = stat.getBlockSizeLong() * stat.getAvailableBlocksLong();
    final double gbAvailable = ((double) bytesAvailable) * 1e-9;

    final String cameraText = String.format(
        Locale.US,
        "FOC: %s,  ISO: %s,  WB: %s,  Free space: %.02f Gb",
        getFocalLengthText(captureResult),
        getIsoSensitivity(captureResult),
        StringConverters.whiteBalanceModeToString(whiteBalanceMode),
        gbAvailable);
    textViewCamera.setText(cameraText);
  }
}
