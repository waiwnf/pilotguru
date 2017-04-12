package de.weis.multisensor_grabber;

import android.location.Location;
import android.location.LocationListener;
import android.os.Bundle;
import android.widget.TextView;

import java.util.Locale;

public class PreviewCoordinatesTextUpdater implements LocationListener {
  private final TextView textViewCoords;

  public PreviewCoordinatesTextUpdater(TextView textViewCoords) {
    this.textViewCoords = textViewCoords;
  }

  public void onLocationChanged(Location location) {
    final String coordinatesText = String
        .format(Locale.US, "Coordinates:  lat %.03f  lon %.03f  accuracy %.01f",
            location.getLatitude(), location.getLongitude(), location.getAccuracy());
    textViewCoords.setText(coordinatesText);
  }

  public void onProviderDisabled(String provider) {
  }

  public void onProviderEnabled(String provider) {
  }

  public void onStatusChanged(String provider, int status, Bundle extras) {
  }
}
