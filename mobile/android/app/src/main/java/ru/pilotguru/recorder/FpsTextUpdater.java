package ru.pilotguru.recorder;

import android.support.annotation.NonNull;
import android.widget.TextView;

import java.util.Locale;
import java.util.concurrent.TimeUnit;

class FpsTextUpdater extends TimeSpacedUpdater {
  private final TextView textViewFps;
  private long prevFrameNanos = -1, currentFrameNanos = -1;

  public FpsTextUpdater(long updateIntervalNanos, @NonNull TextView textViewFps) {
    super(updateIntervalNanos);
    this.textViewFps = textViewFps;
  }

  public void setCurrentFrameNanosMaybeUpdate(long currentFrameNanos) {
    this.prevFrameNanos = this.currentFrameNanos;
    this.currentFrameNanos = currentFrameNanos;
    maybeUpdate(currentFrameNanos);
  }

  @Override
  public void doUpdate(long currentTimeNanos) {
    final long interFrameNanos =
        (prevFrameNanos < currentFrameNanos) ? (currentFrameNanos - prevFrameNanos) : 0;
    final double fps = (interFrameNanos == 0) ? Double.NaN :
        ((double) TimeUnit.SECONDS.toNanos(1)) / (double) interFrameNanos;
    textViewFps.setText(String.format(Locale.US, "FPS: %.01f", fps));
  }
}
