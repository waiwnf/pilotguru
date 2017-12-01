package ru.pilotguru.recorder;

public abstract class TimeSpacedUpdater {
  private final long updateIntervalNanos;
  private long lastUpdateNanos = -1;

  public TimeSpacedUpdater(long updateIntervalNanos) {
    this.updateIntervalNanos = updateIntervalNanos;
  }

  public abstract void doUpdate(long currentTimeNanos);

  public void maybeUpdate(long currentTimeNanos) {
    if (currentTimeNanos - lastUpdateNanos >= updateIntervalNanos) {
      doUpdate(currentTimeNanos);
      lastUpdateNanos = currentTimeNanos;
    }
  }
}
