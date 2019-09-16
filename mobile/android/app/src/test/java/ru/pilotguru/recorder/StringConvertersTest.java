package ru.pilotguru.recorder;

import org.junit.Assert;
import org.junit.Test;

import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_DAYLIGHT;

public class StringConvertersTest {
    @Test
    public void wbStringNull() {
        Assert.assertEquals("N/A", StringConverters.whiteBalanceModeToString(null));
    }

    @Test
    public void wbStringDaylight() {
        Assert.assertEquals("Daylight", StringConverters.whiteBalanceModeToString(CONTROL_AWB_MODE_DAYLIGHT));
    }

    @Test
    public void wbStringDayUnknown() {
        Assert.assertEquals("N/A: 1024", StringConverters.whiteBalanceModeToString(1024));
    }
}
