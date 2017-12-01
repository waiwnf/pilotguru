package ru.pilotguru.recorder;

public class LastPacketTimestamp {
  public synchronized long getTimeNanos() {
    return timeNanos;
  }

  public synchronized void setTimeNanos(long timeNanos) {
    this.timeNanos = timeNanos;
  }

  private long timeNanos = -1;
}
