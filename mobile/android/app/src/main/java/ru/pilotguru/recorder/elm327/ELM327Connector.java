package ru.pilotguru.recorder.elm327;

import android.support.annotation.NonNull;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

public interface ELM327Connector {
  InputStream getInputStream() throws IOException;
  OutputStream getOutputStream() throws IOException;
  void reconnect() throws IOException;
  void close()  throws IOException;
  @NonNull String deviceAddress();
}
