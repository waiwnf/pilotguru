package ru.pilotguru.recorder;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.location.Location;
import android.location.LocationListener;
import android.media.MediaScannerConnection;
import android.os.Build;
import android.os.Bundle;
import android.os.StatFs;
import android.os.SystemClock;
import android.support.annotation.NonNull;
import android.util.JsonWriter;
import android.util.Log;
import android.widget.TextView;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import ru.pilotguru.recorder.elm327.ELM327Receiver;

public class SensorDataSaver extends CameraCaptureSession.CaptureCallback implements
    SensorEventListener, LocationListener, ELM327Receiver.ELM327Listener {
  public static String ROTATIONS = "rotations";
  public static String ACCELERATIONS = "accelerations";
  public static String LOCATIONS = "locations";
  public static String FRAMES = "frames";
  public static String CAN_FRAMES = "can_frames";

  public static String TIME_USEC = "time_usec";

  private JsonWriter rotationsWriter = null, accelerationsWriter = null, locationsWriter = null,
      framesWriter = null, elm327Writer = null;
  private List<String> jsonFiles = new LinkedList<>();

  private boolean isRecording = false;
  private final ReadWriteLock recordingStatusLock = new ReentrantReadWriteLock();
  private final Lock gyroLock = recordingStatusLock.readLock();
  private final Lock accelerometerLock = recordingStatusLock.readLock();
  private final Lock locationLock = recordingStatusLock.readLock();
  private final Lock frameCaptureLock = recordingStatusLock.readLock();
  private final Lock elm327Lock = recordingStatusLock.readLock();
  private final Lock recordingStatusChangeLock = recordingStatusLock.writeLock();

  private TextView textViewFps, textViewCamera;

  private final Activity parentActivity;
  private File recordingDir;    // Parent directory where to write the current recording.
  // Frame timestamps for FPS computations.
  private long prevFrameSystemNanos = 0, currentFrameSystemNanos = 0;
  // Last filesystem free space query time.
  private long lastSpaceQueryTimeNanos = 0;
  // Most recent cached filesystem free space result in Gb.
  private double spaceAvailableGb = 0;

  // The difference between timestamp bases for camera frames and all the other sensors (GPS, gyro).
  private long cameraTimestampsShiftWrtSensors = 0;

  // Multiple video recording sequences have unified frame id space. We want to have every sequence
  // have its frames be counted from 0 onwards. We will store the first frame id of the current
  // sequence here and subtract it from all the subsequent frame ids of that sequence.
  private long firstFrameNumberInSequence = -1;

  public SensorDataSaver(@NonNull Activity parentActivity) {
    this.parentActivity = parentActivity;
  }

  public void start(@NonNull File recordingDir, TextView textViewFps, TextView textViewCamera,
                    CameraCharacteristics cameraCharacteristics) {
    try {
      recordingStatusChangeLock.lock();
      if (isRecording) {
        throw new AssertionError("Called start() but SensorDataSaver is already recording.");
      }
      isRecording = true;

      this.textViewFps = textViewFps;
      this.textViewCamera = textViewCamera;
      this.recordingDir = recordingDir;

      final int cameraTimestampSource =
          cameraCharacteristics.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE);
      switch (cameraTimestampSource) {
        case CameraMetadata.SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN:
          // Assume that the camera timer is based on System.nanoTime(). This is the case on Nexus 5
          // and Nexus 5X.

          // Allocate storage for multiple attempts at computing the difference between
          // SystemClock.elapsedRealtimeNanos() (which is not paused when sleeping) and
          // System.nanoTime() (which is paused in deep sleep).
          // We want to compute the difference repeatedly several times to warm up the caches and
          // achieve better accuracy.
          final long[] elapsedNanoTimeDiffCache = new long[5];
          for (int i = 0; i < elapsedNanoTimeDiffCache.length; ++i) {
            elapsedNanoTimeDiffCache[i] = SystemClock.elapsedRealtimeNanos() - System.nanoTime();
          }
          // Log all the cached timer differences to prevent the compiler from optimizing away the
          // function calls.
          Log.i("SensorDataSaver", "elapsedRealtimeNanos - nanoTime difference values: " +
              Arrays.toString(elapsedNanoTimeDiffCache));

          // Use the las estimate, which should be more accurate than the first.
          cameraTimestampsShiftWrtSensors =
              elapsedNanoTimeDiffCache[elapsedNanoTimeDiffCache.length - 1];
          break;

        case CameraMetadata.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME:
          cameraTimestampsShiftWrtSensors = 0;
          break;

        default:
          throw new AssertionError("Unknown camera timestamps source: " + cameraTimestampSource);
      }

      rotationsWriter = initJsonListWriter(recordingDir, ROTATIONS);
      accelerationsWriter = initJsonListWriter(recordingDir, ACCELERATIONS);
      locationsWriter = initJsonListWriter(recordingDir, LOCATIONS);
      framesWriter = initJsonListWriter(recordingDir, FRAMES);
      elm327Writer = initJsonListWriter(recordingDir, CAN_FRAMES);
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

  private double getLastFps() {
    final long interFrameNanos =
        (prevFrameSystemNanos > 0) ? (currentFrameSystemNanos - prevFrameSystemNanos) : 0;
    return (interFrameNanos == 0) ? Double.NaN :
        ((double) TimeUnit.SECONDS.toNanos(1)) / (double) interFrameNanos;
  }

  private double getGbAvailable(File f, long currentTimeNanos) {
    // Only query the file system every couple of seconds to keep the load and latency down.
    if (currentTimeNanos - lastSpaceQueryTimeNanos > TimeUnit.SECONDS.toNanos(2)) {
      final StatFs stat = new StatFs(f.getPath());
      final long bytesAvailable = stat.getBlockSizeLong() * stat.getAvailableBlocksLong();
      spaceAvailableGb = ((double) bytesAvailable) * 1e-9;

      lastSpaceQueryTimeNanos = currentTimeNanos;
    }
    return spaceAvailableGb;
  }

  public void stop(Context context) {
    recordingStatusChangeLock.lock();
    if (!isRecording) {
      throw new AssertionError("Called stop() but SensorDataSaver is not recording.");
    }
    isRecording = false;

    finishJsonListWriter(rotationsWriter, ROTATIONS);
    finishJsonListWriter(accelerationsWriter, ACCELERATIONS);
    finishJsonListWriter(locationsWriter, LOCATIONS);
    finishJsonListWriter(framesWriter, FRAMES);
    finishJsonListWriter(elm327Writer, CAN_FRAMES);
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

  private void writeXYZSensor(Lock lock, SensorEvent event, JsonWriter writer, String sensorName) {
    try {
      lock.lock();
      if (isRecording) {
        writer.beginObject();
        writer.name("x").value(event.values[0]);
        writer.name("y").value(event.values[1]);
        writer.name("z").value(event.values[2]);
        writer.name(TIME_USEC).value(TimeUnit.NANOSECONDS.toMicros(event.timestamp));
        writer.endObject();
      }
    } catch (IOException e) {
      Errors.dieOnException(parentActivity, e, "Error writing " + sensorName + " JSON.");
    } finally {
      lock.unlock();
    }
  }

  public void onSensorChanged(SensorEvent event) {
    switch (event.sensor.getType()) {
      case Sensor.TYPE_GYROSCOPE:
        writeXYZSensor(gyroLock, event, rotationsWriter, ROTATIONS);
        break;
      case Sensor.TYPE_ACCELEROMETER:
        writeXYZSensor(accelerometerLock, event, accelerationsWriter, ACCELERATIONS);
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
        locationsWriter.name("altitude_m").value(location.getAltitude());
        locationsWriter.name("accuracy_m").value(location.getAccuracy());
        if (Build.VERSION.SDK_INT >= 26) {
          locationsWriter.name("vertical_accuracy_m").value(location.getVerticalAccuracyMeters());
        }
        locationsWriter.name("speed_m_s").value(location.getSpeed());
        locationsWriter.name("bearing_degrees").value(location.getBearing());
        locationsWriter.name(TIME_USEC)
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

  // ELM327Listener
  @Override
  public void onELM327ResponseReceived(@NonNull ELM327Receiver.TimestampedResponse response) {
    try {
      elm327Lock.lock();
      if (isRecording) {
        elm327Writer.beginObject();
        elm327Writer.name("can_frame").value(response.getText());
        elm327Writer.name(TIME_USEC).value(TimeUnit.NANOSECONDS.toMicros(response.getStartNanos()));
        elm327Writer.endObject();
      }
    } catch (IOException e) {
      Errors.dieOnException(
          parentActivity, e, "Error writing CAN frames JSON from ELM327 adapter.");
    } finally {
      elm327Lock.unlock();
    }
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
        final long frameSensorNanos = result.get(CaptureResult.SENSOR_TIMESTAMP);
        framesWriter.beginObject();
        framesWriter.name("frame_id").value(currentFrameNumberInSequence);
        framesWriter.name("sensor_timestamp").value(frameSensorNanos);
        final long timeUsec =
            TimeUnit.NANOSECONDS.toMicros(frameSensorNanos + cameraTimestampsShiftWrtSensors);
        framesWriter.name(TIME_USEC).value(timeUsec);
        framesWriter.endObject();

        prevFrameSystemNanos = currentFrameSystemNanos;
        currentFrameSystemNanos = frameSensorNanos;

        // Update FPS text view.
        if (textViewFps != null) {
          textViewFps.setText(String.format(Locale.US, "FPS: %.01f", getLastFps()));
        }

        // Update focus distance and stuff.
        if (textViewCamera != null) {
          final int whiteBalanceMode = result.get(CaptureResult.CONTROL_AWB_MODE);
          final double gbAvailable = getGbAvailable(recordingDir, frameSensorNanos);
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
