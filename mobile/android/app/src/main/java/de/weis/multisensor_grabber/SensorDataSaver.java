package de.weis.multisensor_grabber;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.location.Location;
import android.location.LocationListener;
import android.media.MediaScannerConnection;
import android.os.Bundle;
import android.os.StatFs;
import android.support.annotation.NonNull;
import android.util.JsonWriter;
import android.widget.TextView;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class SensorDataSaver extends CameraCaptureSession.CaptureCallback implements
    SensorEventListener, LocationListener {
  public static String HEADINGS = "headings";
  public static String ACCELERATIONS = "accelerations";
  public static String LOCATIONS = "locations";
  public static String FRAMES = "frames";

  public static String SYSTEM_TIME_MSEC = "system_time_msec";
  public static String TIME_USEC = "time_usec";

  private JsonWriter headingsWriter = null, accelerationsWriter = null, locationsWriter = null,
      framesWriter = null;
  private List<String> jsonFiles = new LinkedList<>();

  private boolean isRecording = false;
  private final ReadWriteLock recordingStatusLock = new ReentrantReadWriteLock();
  private final Lock gyroLock = recordingStatusLock.readLock();
  private final Lock accelerometerLock = recordingStatusLock.readLock();
  private final Lock locationLock = recordingStatusLock.readLock();
  private final Lock frameCaptureLock = recordingStatusLock.readLock();
  private final Lock recordingStatusChangeLock = recordingStatusLock.writeLock();

  private TextView textViewFps, textViewCamera;

  private final Activity parentActivity;
  private File recordingDir;    // Parent directory where to write the current recording.
  // Frame timestamps for FPS computations.
  private long prevFrameSystemMicros = 0, currentFrameSystemMicros = 0;
  // Last filesystem free space query time.
  private long lastSpaceQueryTimeMicros = 0;
  // Most recent cached filesystem free space result in Gb.
  private double spaceAvailableGb = 0;
  // Multiple video recording sequences have unified frame id space. We want to have every sequence
  // have its frames be counted from 0 onwards. We will store the first frame id of the current
  // sequence here and subtract it from all the subsequent frame ids of that sequence.
  private long firstFrameNumberInSequence = -1;

  public SensorDataSaver(@NonNull Activity parentActivity) {
    this.parentActivity = parentActivity;
  }

  public void start(@NonNull File recordingDir, TextView textViewFps, TextView textViewCamera) {
    try {
      recordingStatusChangeLock.lock();
      if (isRecording) {
        throw new AssertionError("Called start() but SensorDataSaver is already recording.");
      }
      isRecording = true;

      this.textViewFps = textViewFps;
      this.textViewCamera = textViewCamera;
      this.recordingDir = recordingDir;

      headingsWriter = initJsonListWriter(recordingDir, HEADINGS);
      accelerationsWriter = initJsonListWriter(recordingDir, ACCELERATIONS);
      locationsWriter = initJsonListWriter(recordingDir, LOCATIONS);
      framesWriter = initJsonListWriter(recordingDir, FRAMES);
    } finally {
      recordingStatusChangeLock.unlock();
    }
  }

  private JsonWriter initJsonListWriter(@NonNull File recordingDir, @NonNull String name) {
    JsonWriter writer = null;
    try {
      // Init the file IO stuff.
      final File outFile = new File(recordingDir, name + ".json");
      // Save the file name so that we can scan it after writing is finished to make the file
      // visible over USB connection without reboot.
      jsonFiles.add(outFile.toString());
      writer = new JsonWriter(new BufferedWriter(new FileWriter(outFile)));

      // Configure the writer and write the preamble for a named list.
      writer.setIndent("  ");
      writer.beginObject();
      writer.name(name);
      writer.beginArray();
    } catch (IOException e) {
      Errors.dieOnException(parentActivity, e,
          "Error trying to initialize " + name + " JSON writer.");
    }
    return writer;
  }

  private void finishJsonListWriter(@NonNull JsonWriter writer, @NonNull String name) {
    try {
      writer.endArray();
      writer.endObject();
      writer.close();
    } catch (IOException e) {
      Errors.dieOnException(parentActivity, e, "Error trying to close " + name + "JSON writer.");
    }
  }

  public double getLastFps() {
    final long interFrameMicros =
        (prevFrameSystemMicros > 0) ? (currentFrameSystemMicros - prevFrameSystemMicros) : 0;
    return (interFrameMicros == 0) ? Double.NaN :
        ((double) TimeUnit.SECONDS.toMicros(1)) / (double) interFrameMicros;
  }

  public double getGbAvailable(File f, long currentTimeMicros) {
    // Only query the file system every couple of seconds to keep the load and latency down.
    if (currentTimeMicros - lastSpaceQueryTimeMicros > TimeUnit.SECONDS.toMicros(2)) {
      final StatFs stat = new StatFs(f.getPath());
      final long bytesAvailable = stat.getBlockSizeLong() * stat.getAvailableBlocksLong();
      spaceAvailableGb = ((double) bytesAvailable) * 1e-9;

      lastSpaceQueryTimeMicros = currentTimeMicros;
    }
    return spaceAvailableGb;
  }

  public void stop(Context context) {
    recordingStatusChangeLock.lock();
    if (!isRecording) {
      throw new AssertionError("Called stop() but SensorDataSaver is not recording.");
    }
    isRecording = false;

    finishJsonListWriter(headingsWriter, HEADINGS);
    finishJsonListWriter(accelerationsWriter, ACCELERATIONS);
    finishJsonListWriter(locationsWriter, LOCATIONS);
    finishJsonListWriter(framesWriter, FRAMES);
    MediaScannerConnection.scanFile(context, jsonFiles.toArray(new String[0]), null, null);
    jsonFiles.clear();

    // Reset the in-sequence frame numbering.
    firstFrameNumberInSequence = -1;

    recordingStatusChangeLock.unlock();
  }

  public boolean isRecording() {
    return isRecording;
  }

  // SensorEventListener: gyro and accelerations.

  public void onSensorChanged(SensorEvent event) {
    switch (event.sensor.getType()) {
      case Sensor.TYPE_GYROSCOPE:
        try {
          gyroLock.lock();
          if (isRecording) {
            headingsWriter.beginObject();
            headingsWriter.name("yaw").value(event.values[0]);
            headingsWriter.name("pitch").value(event.values[1]);
            headingsWriter.name("roll").value(event.values[2]);
            headingsWriter.name(TIME_USEC).value(TimeUnit.NANOSECONDS.toMicros(event.timestamp));
            headingsWriter.name(SYSTEM_TIME_MSEC).value(System.currentTimeMillis());
            headingsWriter.endObject();
          }
        } catch (IOException e) {
          Errors.dieOnException(parentActivity, e, "Error writing headings JSON.");
        } finally {
          gyroLock.unlock();
        }
        break;
      case Sensor.TYPE_ACCELEROMETER:
        try {
          accelerometerLock.lock();
          if (isRecording) {
            accelerationsWriter.beginObject();
            accelerationsWriter.name("x").value(event.values[0]);
            accelerationsWriter.name("y").value(event.values[1]);
            accelerationsWriter.name("z").value(event.values[2]);
            accelerationsWriter.name(TIME_USEC)
                .value(TimeUnit.NANOSECONDS.toMicros(event.timestamp));
            accelerationsWriter.name(SYSTEM_TIME_MSEC).value(System.currentTimeMillis());
            accelerationsWriter.endObject();
          }
        } catch (IOException e) {
          Errors.dieOnException(parentActivity, e, "Error writing accelerations JSON.");
        } finally {
          accelerometerLock.unlock();
        }
        break;
      default:
        break;
    }
  }

  public void onAccuracyChanged(Sensor sensor, int accuracy) {
  }

  // LocationListener - GPS.
  public void onLocationChanged(Location location) {
    try {
      locationLock.lock();
      if (isRecording) {
        locationsWriter.beginObject();
        locationsWriter.name("lat").value(location.getLatitude());
        locationsWriter.name("lon").value(location.getLongitude());
        locationsWriter.name("accuracy_m").value(location.getAccuracy());
        locationsWriter.name("speed_m_s").value(location.getSpeed());
        locationsWriter.name("bearing_degrees").value(location.getBearing());
        locationsWriter.name("location_time_msec").value(location.getTime());
        locationsWriter.name(SYSTEM_TIME_MSEC).value(System.currentTimeMillis());
        locationsWriter.name("time_since_boot_usec")
            .value(TimeUnit.NANOSECONDS.toMicros(location.getElapsedRealtimeNanos()));
        locationsWriter.endObject();
      }
    } catch (IOException e) {
      Errors.dieOnException(parentActivity, e, "Error writing accelerations JSON.");
    } finally {
      locationLock.unlock();
    }
  }

  public void onProviderDisabled(String provider) {
  }

  public void onProviderEnabled(String provider) {
  }

  public void onStatusChanged(String provider, int status, Bundle extras) {
  }

  // CaptureCallback - captured frames timestamps.
  public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request,
                                 TotalCaptureResult result) {
    try {
      frameCaptureLock.lock();
      if (isRecording) {
        final long globalFrameNumber = result.getFrameNumber();
        if (firstFrameNumberInSequence < 0) {
          firstFrameNumberInSequence = globalFrameNumber;
        }
        final long currentFrameNumberInSequence = globalFrameNumber - firstFrameNumberInSequence;
        final long frameSensorMicros =
            TimeUnit.NANOSECONDS.toMicros(result.get(CaptureResult.SENSOR_TIMESTAMP));

        framesWriter.beginObject();
        framesWriter.name("frame_id").value(currentFrameNumberInSequence);
        framesWriter.name(TIME_USEC).value(frameSensorMicros);
        framesWriter.endObject();

        prevFrameSystemMicros = currentFrameSystemMicros;
        currentFrameSystemMicros = frameSensorMicros;

        // Update FPS text view.
        if (textViewFps != null) {
          textViewFps.setText(String.format(Locale.US, "FPS: %.01f", getLastFps()));
        }

        // Update focus distance and stuff.
        if (textViewCamera != null) {
          final int whiteBalanceMode = result.get(CaptureResult.CONTROL_AWB_MODE);
          final double gbAvailable = getGbAvailable(recordingDir, frameSensorMicros);
          final String cameraText = String
              .format(Locale.US, "FOC: %s,  ISO: %s,  WB: %s,  Free space: %.02f Gb",
                  getFocalLengthText(result), getIsoSensitivity(result),
                  StringConverters.whiteBalanceModeToString(whiteBalanceMode), gbAvailable);
          textViewCamera.setText(cameraText);
        }
      }
    } catch (IOException e) {
      Errors.dieOnException(parentActivity, e, "Error writing frames timestamps JSON.");
    } finally {
      frameCaptureLock.unlock();
    }
  }

  // Helpers for extracting camera info status bits for the status string.

  private String getFocalLengthText(CaptureResult result) {
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

  private String getIsoSensitivity(CaptureResult result) {
    final Integer sensitivity = result.get(CaptureResult.SENSOR_SENSITIVITY);
    if (sensitivity != null) {
      return sensitivity.toString();
    } else {
      return "NA";
    }
  }
}
