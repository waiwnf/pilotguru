package ru.pilotguru.recorder.elm327;

import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.support.annotation.NonNull;
import android.widget.TextView;

import ru.pilotguru.recorder.LastPacketTimestamp;
import ru.pilotguru.recorder.TimeSpacedUpdater;

public class ELM327StatusTextUpdater extends TimeSpacedUpdater implements SensorEventListener {
  private final long elm327TimeoutNanos;
  private final LastPacketTimestamp elm327LastPacketTimestamp;
  private final TextView elm327StatusTextView;

  public ELM327StatusTextUpdater(long updateIntervalNanos,
      long elm327TimeoutNanos,
      @NonNull LastPacketTimestamp elm327LastPacketTimestamp,
      @NonNull TextView elm327StatusTextView) {
    super(updateIntervalNanos);
    this.elm327TimeoutNanos = elm327TimeoutNanos;
    this.elm327LastPacketTimestamp = elm327LastPacketTimestamp;
    this.elm327StatusTextView = elm327StatusTextView;
  }

  @Override
  public void doUpdate(long currentTimeNanos) {
    final long timeSincelastPacketNanos =
        currentTimeNanos - elm327LastPacketTimestamp.getTimeNanos();
    if (timeSincelastPacketNanos < elm327TimeoutNanos) {
      elm327StatusTextView.setText("OBD: ON");
      elm327StatusTextView.setTextColor(Color.GREEN);
    } else {
      elm327StatusTextView.setText("OBD: OFF");
      elm327StatusTextView.setTextColor(Color.RED);
    }
  }

  @Override
  public void onSensorChanged(SensorEvent event) {
    if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
      maybeUpdate(event.timestamp);
    }
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int accuracy) {
  }
}
