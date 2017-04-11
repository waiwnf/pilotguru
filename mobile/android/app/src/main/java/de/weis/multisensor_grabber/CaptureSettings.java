package de.weis.multisensor_grabber;

import android.content.SharedPreferences;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.preference.PreferenceManager;
import android.view.Surface;

import org.xmlpull.v1.XmlSerializer;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

import static android.hardware.camera2.CameraMetadata.CONTROL_AE_MODE_OFF;
import static android.hardware.camera2.CameraMetadata.CONTROL_AF_MODE_OFF;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_AUTO;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_EXPOSURE;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_FIXED_EXPOSURE;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_FIXED_FOCUS_DIST;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_FIXED_ISO;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_FOCUS_DIST;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_ISO;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_WHITE_BALANCE;

public class CaptureSettings implements SerializableSequenceElement {
  private boolean isFixedExposure;
  private long exposureTimeMillis;
  private boolean isFixedFocusDistance;
  private boolean isFixedIso;
  private float focusDistance;
  private int whiteBalanceMode;
  private int isoSensitivity;

  CaptureSettings(SharedPreferences prefs) {
    isFixedExposure = prefs.getBoolean(PREF_FIXED_EXPOSURE, false);
    exposureTimeMillis = Long.parseLong(prefs.getString(PREF_EXPOSURE, "0"));
    isFixedFocusDistance = prefs.getBoolean(PREF_FIXED_FOCUS_DIST, false);
    isFixedIso = prefs.getBoolean(PREF_FIXED_ISO, false);

    focusDistance = Float.parseFloat(prefs.getString(PREF_FOCUS_DIST, "-1"));
    whiteBalanceMode = Integer
        .parseInt(prefs.getString(PREF_WHITE_BALANCE, Integer.toString(CONTROL_AWB_MODE_AUTO)));
    isoSensitivity = Integer.parseInt(prefs.getString(PREF_ISO, "-1"));
  }

  public boolean isFixedExposure() {
    return isFixedExposure;
  }

  public long getExposureTimeMillis() {
    return exposureTimeMillis;
  }

  public boolean isFixedFocusDistance() {
    return isFixedFocusDistance;
  }

  public boolean isFixedIso() {
    return isFixedIso;
  }

  public float getFocusDistance() {
    return focusDistance;
  }

  public int getWhiteBalanceMode() {
    return whiteBalanceMode;
  }

  public int getIsoSensitivity() {
    return isoSensitivity;
  }

  public String getExposureText() {
    return isFixedExposure ? ("Fixed: " + exposureTimeMillis + "ms") : "Auto";
  }

  public String getFocalLengthText() {
    return isFixedFocusDistance ? ("Fixed: " + focusDistance) : "Auto";
  }

  public String getIsoValueText() {
    return isFixedIso ? Integer.toString(isoSensitivity) : "Auto";
  }

  public void serialize(XmlSerializer serializer) throws IOException {
    serializer.attribute(null, "wb_value", Integer.toString(whiteBalanceMode));

    if (isFixedIso) {
      serializer.attribute(null, "iso_value", Integer.toString(isoSensitivity));
    } else {
      serializer.attribute(null, "iso_value", "-1");
    }

    if (isFixedExposure) {
      serializer.attribute(null, "exp_time",
          Long.toString(TimeUnit.MILLISECONDS.toNanos(exposureTimeMillis)));
    } else {
      //FIXME: is it possible to get the exposure time of each single image if auto-exposure is on?
      //FIXME: look at the callback, for some cellphones we can get these values
      serializer.attribute(null, "exp_time", "-1");
    }

    if (isFixedFocusDistance) {
      serializer.attribute(null, "foc_dist", Float.toString(focusDistance));
    } else {
      serializer.attribute(null, "foc_dist", "-1");
    }
  }

  public CaptureRequest.Builder makeCaptureRequestBuilder(CameraDevice cameraDevice,
                                                          Iterable<Surface> outputSurfaces)
      throws CameraAccessException {
    final CaptureRequest.Builder captureBuilder =
        cameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
    captureBuilder.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
    for (Surface outputSurface : outputSurfaces) {
      captureBuilder.addTarget(outputSurface);
    }

    if (isFixedExposure) {
      captureBuilder.set(CaptureRequest.CONTROL_AE_MODE, CONTROL_AE_MODE_OFF);
      captureBuilder.set(CaptureRequest.SENSOR_EXPOSURE_TIME,
          TimeUnit.MILLISECONDS.toNanos(exposureTimeMillis));
      captureBuilder.set(CaptureRequest.CONTROL_AE_LOCK, true);
    }

    if (isFixedIso && isoSensitivity != -1) {
      captureBuilder.set(CaptureRequest.SENSOR_SENSITIVITY, isoSensitivity);
    }


    /************************************** FOCUS ********************************/
    if (isFixedFocusDistance) {
      captureBuilder.set(CaptureRequest.LENS_FOCUS_DISTANCE, focusDistance);
      captureBuilder.set(CaptureRequest.CONTROL_AF_MODE, CONTROL_AF_MODE_OFF);
    }

    /************************************** WHITEBALANCE ********************************/

    captureBuilder.set(CaptureRequest.CONTROL_AWB_MODE, whiteBalanceMode);

    /************************************** REST ********************************/
    // http://stackoverflow.com/questions/29265126/android-camera2-capture-burst-is-too-slow
    captureBuilder.set(CaptureRequest.EDGE_MODE, CaptureRequest.EDGE_MODE_OFF);
    captureBuilder.set(CaptureRequest.COLOR_CORRECTION_ABERRATION_MODE,
        CaptureRequest.COLOR_CORRECTION_ABERRATION_MODE_OFF);
    captureBuilder
        .set(CaptureRequest.NOISE_REDUCTION_MODE, CaptureRequest.NOISE_REDUCTION_MODE_OFF);
    captureBuilder.set(CaptureRequest.CONTROL_AF_TRIGGER, CaptureRequest.CONTROL_AF_TRIGGER_CANCEL);

    return captureBuilder;
  }
}
