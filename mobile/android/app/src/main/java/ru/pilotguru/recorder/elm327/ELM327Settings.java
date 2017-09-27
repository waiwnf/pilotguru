package ru.pilotguru.recorder.elm327;

import android.content.SharedPreferences;

import static ru.pilotguru.recorder.SettingsConstants.PREF_CAN_ID_FILTER;
import static ru.pilotguru.recorder.SettingsConstants.PREF_CAN_ID_MASK;
import static ru.pilotguru.recorder.SettingsConstants.PREF_ELM327_DEVICES;

public class ELM327Settings {
  public String getElm327DeviceName() {
    return elm327DeviceName;
  }

  public Integer getCanIdFilter() {
    return canIdFilter;
  }

  public Integer getCanIdMask() {
    return canIdMask;
  }

  private final String elm327DeviceName;
  private final Integer canIdFilter;
  private final Integer canIdMask;

  public ELM327Settings(SharedPreferences prefs) {
    elm327DeviceName = prefs.getString(PREF_ELM327_DEVICES, "");
    canIdFilter = getCanIdMask(
        !elm327DeviceName.isEmpty(), prefs.getString(PREF_CAN_ID_FILTER, ""));
    canIdMask= getCanIdMask(!elm327DeviceName.isEmpty(), prefs.getString(PREF_CAN_ID_MASK, ""));
  }

  private static Integer getCanIdMask(boolean elm327DeviceNonEmpty, String maskString) {
    if (!elm327DeviceNonEmpty || maskString.isEmpty()) {
      return null;
    } else {
      try {
        final int result = Integer.decode(maskString);
        if (result < 0 || result >= (1 << 12) ) {
          // The mask must fit into 11 bits of the CAN frame ID.
          return null;
        } else {
          return result;
        }
      } catch (NumberFormatException e) {
        return null;
      }
    }
  }

  @Override
  public String toString() {
    return "ELM327Settings{" +
        "elm327DeviceName='" + elm327DeviceName + '\'' +
        ", canIdFilter=" + canIdFilter +
        ", canIdMask=" + canIdMask +
        '}';
  }
}
