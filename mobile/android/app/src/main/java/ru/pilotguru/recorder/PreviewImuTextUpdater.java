package ru.pilotguru.recorder;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.support.annotation.NonNull;
import android.widget.TextView;

import java.util.Locale;

public class PreviewImuTextUpdater extends TimeSpacedUpdater implements SensorEventListener {
  private final float rotation[] = {0, 0, 0};
  private final float acceleration[] = {0, 0, 0};
  private final TextView textViewImu;

  public PreviewImuTextUpdater(long minUpdateIntervalNanos, @NonNull TextView textViewImu) {
    super(minUpdateIntervalNanos);
    this.textViewImu = textViewImu;
  }

  @Override
  public void doUpdate(long currentTimeNanos) {
    final String imuText = String.format(Locale.US,
        "IMU: rotation x %.01f  y %.01f  z %.01f acceleration x %.01f  y %.01f  z %.01f",
        rotation[0], rotation[1], rotation[2], acceleration[0], acceleration[1], acceleration[2]);
    textViewImu.setText(imuText);
  }

  @Override
  public void onSensorChanged(SensorEvent event) {
    switch (event.sensor.getType()) {
      case Sensor.TYPE_GYROSCOPE:
        System.arraycopy(event.values, 0, rotation, 0, 3);  // x, y, z
        maybeUpdate(event.timestamp);
        break;
      case Sensor.TYPE_ACCELEROMETER:
        System.arraycopy(event.values, 0, acceleration, 0, 3);  // x, y, z
        maybeUpdate(event.timestamp);
        break;
      default:
        break;
    }
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int accuracy) {
  }
}
