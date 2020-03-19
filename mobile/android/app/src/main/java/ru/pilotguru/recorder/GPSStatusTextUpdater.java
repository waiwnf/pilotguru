package ru.pilotguru.recorder;

import android.location.GnssStatus;
import android.support.annotation.NonNull;
import android.widget.TextView;

public class GPSStatusTextUpdater extends GnssStatus.Callback {
    final @NonNull TextView textViewGpsStatus;

    GPSStatusTextUpdater(@NonNull TextView textViewGpsStatus) {
        this.textViewGpsStatus = textViewGpsStatus;
    }

    public void onSatelliteStatusChanged (GnssStatus status) {
        int satsInFix = 0;
        for (int i=0; i<status.getSatelliteCount(); ++i) {
            if (status.usedInFix(i)) {
                satsInFix++;
            }
        }
        textViewGpsStatus.setText("GPS status: " + satsInFix + " sats active");
    }

}
