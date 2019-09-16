package ru.pilotguru.recorder;

import android.hardware.camera2.CaptureResult;

import org.junit.Assert;
import org.junit.Test;

public class CameraInfoIntervalUpdaterTest {
    @Test
    public void getFocalTextLength_nullAfMode() {
        Assert.assertEquals("Unknown: 1.0", CameraInfoIntervalUpdater.getFocalLengthText(null, 1.0f));
    }

    @Test
    public void getFocalTextLength_AfModeOff() {
        Assert.assertEquals("Fixed: 1.0", CameraInfoIntervalUpdater.getFocalLengthText(CaptureResult.CONTROL_AF_MODE_OFF, 1.0f));
    }

    @Test
    public void getFocalTextLength_AfModeOffDistanceNull() {
        Assert.assertEquals("Fixed: NA", CameraInfoIntervalUpdater.getFocalLengthText(CaptureResult.CONTROL_AF_MODE_OFF, null));
    }

    @Test
    public void getFocalTextLength_AfModeOn() {
        Assert.assertEquals("Auto: 3.5", CameraInfoIntervalUpdater.getFocalLengthText(CaptureResult.CONTROL_AF_MODE_CONTINUOUS_VIDEO, 3.5f));
    }

}
