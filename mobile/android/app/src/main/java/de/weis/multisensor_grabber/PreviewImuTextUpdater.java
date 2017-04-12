package de.weis.multisensor_grabber;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.widget.TextView;

import java.util.Locale;

public class PreviewImuTextUpdater implements SensorEventListener {
  private final float heading[] = {0, 0, 0};
  private final float acceleration[] = {0, 0, 0};
  private final TextView textViewImu;

  public PreviewImuTextUpdater(TextView textViewImu) {
    this.textViewImu = textViewImu;
  }

  private void updateText() {
    final String imuText = String
        .format(Locale.US, "IMU:  yaw %.01f  pitch %.01f  roll %.01f  ax %.01f  ay %.01f  az %.01f",
            heading[0], heading[1], heading[2], acceleration[0], acceleration[1], acceleration[2]);
    textViewImu.setText(imuText);
  }

  public void onSensorChanged(SensorEvent event) {
    switch (event.sensor.getType()) {
      case Sensor.TYPE_GYROSCOPE:
        System.arraycopy(event.values, 0, heading, 0, 3);  // yaw, pitch, roll
        updateText();
        break;
      case Sensor.TYPE_ACCELEROMETER:
        System.arraycopy(event.values, 0, acceleration, 0, 3);  // x, y, z
        updateText();
        break;
      default:
        break;
    }
  }

  public void onAccuracyChanged(Sensor sensor, int accuracy) {
  }
}
