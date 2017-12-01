package ru.pilotguru.recorder.elm327;

import android.support.annotation.NonNull;

import ru.pilotguru.recorder.LastPacketTimestamp;

public class ELM327LastPacketTimestampUpdater implements ELM327Receiver.ELM327Listener {
  private final LastPacketTimestamp lastPacketTimestamp;

  public ELM327LastPacketTimestampUpdater(@NonNull LastPacketTimestamp lastPacketTimestamp) {
    this.lastPacketTimestamp = lastPacketTimestamp;
  }

  @Override
  public void onELM327ResponseReceived(@NonNull ELM327Receiver.TimestampedResponse response) {
    lastPacketTimestamp.setTimeNanos(response.getEndNanos());
  }
}
