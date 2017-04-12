package de.weis.multisensor_grabber;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.location.Location;
import android.location.LocationListener;
import android.os.Bundle;
import android.util.Pair;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.List;
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

  private JSONArray headings = null, accelerations = null, locations = null, frames = null;

  private boolean isRecording = false;
  private final ReadWriteLock recordingStatusLock = new ReentrantReadWriteLock();
  private final Lock gyroLock = recordingStatusLock.readLock();
  private final Lock accelerometerLock = recordingStatusLock.readLock();
  private final Lock locationLock = recordingStatusLock.readLock();
  private final Lock frameCaptureLock = recordingStatusLock.readLock();
  private final Lock recordingStatusChangeLock = recordingStatusLock.writeLock();

  private final File storageDir;    // Parent directory where to write all the recordings.
  private File recordingDir = null;  // Directory where to write the current recording.

  private long prevFrameSystemMs = 0, currentFrameSystemMs = 0;

  public SensorDataSaver(File storageDir) {
    this.storageDir = storageDir;
  }

  public void start() {
    try {
      recordingStatusChangeLock.lock();
      assert !isRecording;
      isRecording = true;

      // Allocate new arrays for all the datastreams. We will accumulate the data in memory and
      // write it out to files in the end (on stop()) to avoid file IO in all the event handlers
      // during recording.
      headings = new JSONArray();
      accelerations = new JSONArray();
      locations = new JSONArray();
      frames = new JSONArray();

      // Make a directory ID for the current recording based on the current timestamp.
      final long sequenceStartMillis = System.currentTimeMillis();
      final DateFormat dateFormat = new SimpleDateFormat("yyyy_MM_dd-HH_mm_ss");
      recordingDir = new File(storageDir, dateFormat.format(new Date(sequenceStartMillis)));
      recordingDir.mkdirs();

    } finally {
      recordingStatusChangeLock.unlock();
    }
  }

  public double getLastFps() {
    final long interFrameMs =
        (prevFrameSystemMs > 0) ? (currentFrameSystemMs - prevFrameSystemMs) : 0;
    return (interFrameMs == 0) ? Double.NaN :
        ((double) TimeUnit.SECONDS.toMillis(1)) / (double) interFrameMs;
  }

  public void stop() {
    try {
      recordingStatusChangeLock.lock();
      assert isRecording;
      isRecording = false;


      final List<Pair<JSONArray, String>> outputDataItems = Arrays
          .asList(new Pair<>(headings, HEADINGS), new Pair<>(accelerations, ACCELERATIONS),
              new Pair<>(locations, LOCATIONS), new Pair<>(frames, FRAMES));
      for (Pair<JSONArray, String> outputData : outputDataItems) {
        final JSONObject outputJson = new JSONObject();
        final String outputName = outputData.second;
        outputJson.put(outputName, outputData.first);
        final Writer outputWriter =
            new BufferedWriter(new FileWriter(new File(recordingDir, outputName + ".json")));
        outputWriter.write(outputJson.toString(2));
        outputWriter.close();
      }

      // Release all the data arrays.
      headings = null;
      accelerations = null;
      locations = null;
      frames = null;

    } catch (JSONException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    } finally {
      recordingStatusChangeLock.unlock();
    }
  }

  // SensorEventListener: gyro and accelerations.

  public void onSensorChanged(SensorEvent event) {
    switch (event.sensor.getType()) {
      case Sensor.TYPE_GYROSCOPE:
        try {
          gyroLock.lock();
          if (isRecording) {
            final JSONObject heading = new JSONObject();
            heading.put("yaw", event.values[0]);
            heading.put("pitch", event.values[1]);
            heading.put("roll", event.values[2]);
            heading.put("time_msec", TimeUnit.NANOSECONDS.toMillis(event.timestamp));
            headings.put(heading);
          }
        } catch (JSONException e) {
        } finally {
          gyroLock.unlock();
        }
        break;
      case Sensor.TYPE_ACCELEROMETER:
        try {
          accelerometerLock.lock();
          if (isRecording) {
            final JSONObject acceleration = new JSONObject();
            acceleration.put("x", event.values[0]);
            acceleration.put("y", event.values[1]);
            acceleration.put("z", event.values[2]);
            acceleration.put("time_msec", TimeUnit.NANOSECONDS.toMillis(event.timestamp));
            accelerations.put(acceleration);
          }
        } catch (JSONException e) {
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
        final JSONObject locationJson = new JSONObject();
        locationJson.put("lat", location.getLatitude());
        locationJson.put("lon", location.getLongitude());
        locationJson.put("accuracy_m", location.getAccuracy());
        locationJson.put("speed_m_s", location.getSpeed());
        locationJson.put("bearing_degrees", location.getBearing());
        locationJson.put("event_time_msec", location.getTime());
        locationJson.put("time_msec", System.currentTimeMillis());
        locations.put(locationJson);
      }
    } catch (JSONException e) {
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
        final JSONObject frameJson = new JSONObject();
        frameJson.put("frame_id", result.getFrameNumber());
        final long frameSensorNanos = result.get(CaptureResult.SENSOR_TIMESTAMP);
        frameJson.put("event_time_nanos", frameSensorNanos);
        frameJson.put("time_msec", System.currentTimeMillis());
        frames.put(frameJson);
        prevFrameSystemMs = currentFrameSystemMs;
        currentFrameSystemMs = TimeUnit.NANOSECONDS.toMillis(frameSensorNanos);
      }
    } catch (JSONException e) {
    } finally {
      frameCaptureLock.unlock();
    }
  }

}
