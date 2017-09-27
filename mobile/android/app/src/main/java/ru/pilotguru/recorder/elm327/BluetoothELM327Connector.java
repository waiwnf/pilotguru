package ru.pilotguru.recorder.elm327;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.support.annotation.NonNull;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;

public class BluetoothELM327Connector implements ELM327Connector {
  private BluetoothDevice device;
  private final UUID uuid;
  private BluetoothSocket socket;

  public BluetoothELM327Connector(@NonNull String deviceAddress) throws IOException {
    Log.w("PilotGuruELM327", "BluetoothELM327Connector constructor");
    BluetoothAdapter btAdapter = BluetoothAdapter.getDefaultAdapter();
    device = btAdapter.getRemoteDevice(deviceAddress);
    uuid = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    socket = device.createInsecureRfcommSocketToServiceRecord(uuid);
    socket.connect();
  }

  @Override
  synchronized public InputStream getInputStream() throws IOException {
    return socket.getInputStream();
  }

  @Override
  synchronized public OutputStream getOutputStream() throws IOException {
    return socket.getOutputStream();
  }

  @Override
  synchronized public void reconnect() throws IOException {
    socket.close();
    socket = device.createInsecureRfcommSocketToServiceRecord(uuid);
    socket.connect();
  }

  @Override
  synchronized public void close() throws IOException {
    socket.close();
  }

  @Override
  public @NonNull String deviceAddress() {
    return device.getAddress();
  }
}
