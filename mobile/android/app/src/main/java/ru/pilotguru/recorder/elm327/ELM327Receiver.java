package ru.pilotguru.recorder.elm327;

import android.os.SystemClock;
import android.support.annotation.NonNull;
import android.util.Log;

import java.io.IOException;
import java.io.OutputStream;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public class ELM327Receiver {
  public class TimestampedResponse {
    private final long startNanos, endNanos;
    private final String text;

    public TimestampedResponse(long startNanos, long endNanos, String text) {
      this.startNanos = startNanos;
      this.endNanos = endNanos;
      this.text = text;
    }

    public long getStartNanos() {
      return startNanos;
    }

    public long getEndNanos() {
      return endNanos;
    }

    public String getText() {
      return text;
    }

    @Override
    public String toString() {
      return "TimestampedResponse{" +
          "startNanos=" + startNanos +
          ", endNanos=" + endNanos +
          ", text='" + text + '\'' +
          '}';
    }
  }

  public interface ELM327Listener {
    void onELM327ResponseReceived(@NonNull TimestampedResponse response);
  }

  private static final String ELM327_RESPONSE_OK = "OK>";
  private static final String ELM327_RESET_PROMPT = "ELM327 v1.5>";

  private final ELM327Connector elm327Connector;
  private final Integer canIdFilter, canIdMask;
  private final Set<ELM327Listener> subscribers = new HashSet<>();
  private final ReadWriteLock subscribersLock = new ReentrantReadWriteLock();
  private final Lock subscribersReadLock = subscribersLock.readLock();
  private final Lock subscribersWriteLock = subscribersLock.writeLock();

  private boolean isMonitoring = false, shouldMonitor = false;

  public ELM327Receiver(
      @NonNull ELM327Connector elm327Connector, Integer canIdFilter, Integer canIdMask) {
    this.elm327Connector = elm327Connector;
    this.canIdFilter = canIdFilter;
    this.canIdMask = canIdMask;
  }

  public boolean init() throws IOException {
    sendCommand("AT Z");  // Reset all.
    final String resetResponse = getResponse(false /* isCrEarlyBreak */).getText();
    if (!resetResponse.endsWith(ELM327_RESET_PROMPT)) {
      Log.w(
          "PilotGuruELM327",
          "ELM327 init response does not end in expected [" + ELM327_RESET_PROMPT +
              "]. Actual value: [" + resetResponse + "].");
      return false;
    }

    sendCommand("AT E0");  // Echo off.
    if (!checkResponse(getResponse(false /* isCrEarlyBreak */).getText(), "AT E0OK>")) {
      return false;
    }

    sendCommand("AT SP 6");  // Protocol to 500 kbit/s 11-bit CAN.
    if (!checkResponse(getResponse(false /* isCrEarlyBreak */).getText(), ELM327_RESPONSE_OK)) {
      return false;
    }

    sendCommand("AT DP");  // Query protocol to make sure it matches.
    if (!checkResponse(
        getResponse(false /* isCrEarlyBreak */).getText(),
        "ISO 15765-4 (CAN 11/500)>")) {
      return false;
    }

    sendCommand("AT H1");  // Headers on. Reqiured to see CAN frame IDs.
    if (!checkResponse(getResponse(false /* isCrEarlyBreak */).getText(), ELM327_RESPONSE_OK)) {
      return false;
    }

    // CAN auto-format off. Disables "DATA ERROR" messages for received CAN frames with fewer than
    // 8 bytes.
    sendCommand("AT CAF0");
    if (!checkResponse(getResponse(false /* isCrEarlyBreak */).getText(), ELM327_RESPONSE_OK)) {
      return false;
    }

    if (canIdFilter != null && canIdMask != null) {
      sendCommand(String.format("AT CF %3X", canIdFilter));  // CAN ID filter.
      if (!checkResponse(getResponse(false /* isCrEarlyBreak */).getText(), ELM327_RESPONSE_OK)) {
        return false;
      }

      sendCommand(String.format("AT CM %3X", canIdMask));  // CAN ID filter importance mask.
      if (!checkResponse(getResponse(false /* isCrEarlyBreak */).getText(), ELM327_RESPONSE_OK)) {
        return false;
      }
    }

    return true;
  }

  private void sendCommand(@NonNull String command) throws IOException {
    final OutputStream outputStream = elm327Connector.getOutputStream();
    outputStream.write(command.getBytes());
    outputStream.write('\r');
    outputStream.flush();
  }

  @NonNull
  private TimestampedResponse getResponse(boolean isCrEarlyBreak) throws IOException {
    StringBuilder responseBuilder = new StringBuilder();
    long startNanos = -1;
    while (true) {
      final int readInt = elm327Connector.getInputStream().read();
      if (responseBuilder.length() == 0) {
        startNanos = SystemClock.elapsedRealtimeNanos();
      }
      if (readInt == 0) {
        // ELM327 may spontaneously insert nulls into responses, they should be ignored per
        // datasheet.
        continue;
      } else if (readInt < 0) {
        Log.w("PilotGuru", "Unexpected end of OBDII response stream");
        throw new IOException("Unexpected end of OBDII response stream");
      } else if (readInt == 0x0d) {
        // Encountered CR.
        if (isCrEarlyBreak) {
          // If early breaks on CR are requested (i.e. if only the first line of the response is
          // needed, stop reading the buffer.
          break;
        } else {
          // CRs should be ignored when the whole response up to the next response prompt is
          // requested.
          continue;
        }
      } else {
        final char readChar = (char) readInt;
        responseBuilder.append(readChar);
        if (readChar == '>') {
          break;
        }
      }
    }
    final long endNanos = SystemClock.elapsedRealtimeNanos();
    return new TimestampedResponse(startNanos, endNanos, responseBuilder.toString());
  }

  public void monitorBlocking(int totalLinesRequested) {
    synchronized (this) {
      if (shouldMonitor) {
        Log.e("PilotGuruELM327", "Monitoring already requested.");
        return;
      }
      shouldMonitor = true;
    }
    doMonitor(totalLinesRequested);
    stopMonitoring();
  }

  public void startMonitoringAsync(final int totalLinesRequested) {
    synchronized (this) {
      if (shouldMonitor) {
        Log.e("PilotGuruELM327", "Monitoring already requested.");
        return;
      }
      shouldMonitor = true;
      new Thread(new Runnable() {
        @Override
        public void run() {
          doMonitor(totalLinesRequested);
        }
      }).start();
      while (!isMonitoring) {
        try {
          wait();
        } catch (InterruptedException e) {
        }
      }
    }
  }

  private void doMonitor(int totalLinesRequested) {
    synchronized (this) {
      if (isMonitoring) {
        Log.e("PilotGuruELM327", "doMonitor() is already running.");
        return;
      } else if (!shouldMonitor) {
        Log.e("PilotGuruELM327", "doMonitor() called, but monitoring not previously requested.");
        return;
      } else {
        isMonitoring = true;
        notifyAll();
      }
    }

    int validLinesSaved = 0;
    final int maxReconnectAttempts = 10;

    boolean isConnected = true;
    boolean isInitialized = false;
    TimestampedResponse previousLine = null;
    while (
        (totalLinesRequested < 0 || validLinesSaved < totalLinesRequested) && shouldMonitor) {
      try {
        if (!isConnected) {
          // IO with ELMS327 broke up. Make at most maxReconnectAttempts consecutive reconnect
          // attempts; bail out if they all fail.
          int reconnectAttempt = 0;
          for (; reconnectAttempt < maxReconnectAttempts; ++reconnectAttempt) {
            try {
              elm327Connector.reconnect();
              // If we are here, then reconnect() did not throw and was successful. Exit the reconnect
              // attempts loop.
              break;
            } catch (IOException reconnectException) {
              // Reconnect failed. Do nothing to proceed to the next attempt in the loop.
            }
          }
          if (reconnectAttempt >= maxReconnectAttempts) {
            // We have spent maxReconnectAttempts reconnects unsuccessfully, stop trying to talk to
            // ELM327 altogether.
            break;  // while (validLinesSaved < totalLinesRequested)
          } else {
            // Fewer than max reconnect attempts were used, so the last one was successful. Reset the
            // received data before coming back to the main monitoring loop.
            previousLine = null;
            isInitialized = false;
          }
        }

        if (!isInitialized) {
          isInitialized = init();
          if (!isInitialized) {
            // ELM327 device init failed, but not because of IO exception. Most likely the AT
            // commands protocol has changed, so there is no point in retrying and we give up.
            break;
          }
        }

        // Having a previous line is an indicator of running monitoring session. If there is no
        // previous line, we need to send the "monitor all" AT command.
        if (previousLine == null) {
          sendCommand("AT MA");
        }

        final TimestampedResponse currentLine = getResponse(true /* isCrEarlyBreak */);
        if (currentLine.getText().endsWith(">")) {
          // Current line ends in a prompt. This can only be because the ELM327 stopped the "monitor
          // all" session, so the previous line may have been not a valid CAN frame (truncated or
          // containing also ELM327 error message). Hence we discard the previous line too.
          previousLine = null;
          continue;
        } else if (currentLine.getText().contains("BUFFER")) {
          // "BUFFER FULL" is a frequent error message before the "monitor all" session is aborted
          // by ELM327. Empty the buffer (read up everything until the prompt sign) and reset.
          getResponse(false  /* isCrEarlyBreak */);
          previousLine = null;
          continue;
        } else {
          // The current line is not a prompt and does not appear to be an error message, so the
          // previous line, if there was one, must be a valid CAN frame.
          if (previousLine != null) {
            subscribersReadLock.lock();
            for (ELM327Listener subscriber : subscribers) {
              subscriber.onELM327ResponseReceived(previousLine);
            }
            subscribersReadLock.unlock();
            ++validLinesSaved;
          }
          previousLine = currentLine;
        }
      } catch (IOException e) {
        isConnected = false;
      }
    }

    synchronized (this) {
      isMonitoring = false;
      notifyAll();
    }
  }

  public void stopMonitoring() {
    synchronized (this) {
      try {
        shouldMonitor = false;
        elm327Connector.close();
        while (isMonitoring) {
          wait();
        }
      } catch (IOException | InterruptedException e) {
      }
    }

  }

  public void addSubscriber(@NonNull ELM327Listener subscriber) {
    subscribersWriteLock.lock();
    subscribers.add(subscriber);
    subscribersWriteLock.unlock();
  }

  public @NonNull String deviceAddress() {
    return elm327Connector.deviceAddress();
  }

  private static boolean checkResponse(@NonNull String actual, @NonNull String expected) {
    final boolean responseMatches = actual.equals(expected);
    if (!responseMatches) {
      Log.w(
          "PilotGuruELM327",
          "ELM327 response mismatch. Expected [" + expected + "], but got [" + actual + "].");
    }
    return responseMatches;
  }
}
