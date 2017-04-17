package de.weis.multisensor_grabber;

import android.content.Context;
import android.content.SharedPreferences;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.media.CamcorderProfile;
import android.os.Bundle;
import android.preference.CheckBoxPreference;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceFragment;
import android.preference.PreferenceManager;
import android.util.Pair;
import android.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static android.hardware.camera2.CameraCharacteristics.CONTROL_AWB_AVAILABLE_MODES;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_AUTO;

import static de.weis.multisensor_grabber.SettingsConstants.PREF_FIXED_FOCUS_DIST;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_FIXED_ISO;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_FOCUS_DIST;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_ISO;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_WHITE_BALANCE;

/**
 * Created by weis on 27.12.16.
 */

public class SettingsFragment extends PreferenceFragment implements
    SharedPreferences.OnSharedPreferenceChangeListener {
  CameraCharacteristics characteristics = null;

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    // Load the preferences from an XML resource
    addPreferencesFromResource(R.xml.preferences);

    getPreferenceScreen().getSharedPreferences().registerOnSharedPreferenceChangeListener(this);

    final CameraManager manager =
        (CameraManager) getActivity().getSystemService(Context.CAMERA_SERVICE);
    try {
      final String cameraId = manager.getCameraIdList()[0];
      characteristics = manager.getCameraCharacteristics(cameraId);
    } catch (CameraAccessException e) {
      e.printStackTrace();
    }

    final CheckBoxPreference prefFixedFocus =
        (CheckBoxPreference) findPreference(PREF_FIXED_FOCUS_DIST);
    prefFixedFocus.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
      @Override
      public boolean onPreferenceChange(Preference preference, Object newValue) {
        populateFocusDistance(newValue);
        return true;
      }
    });

    final CheckBoxPreference prefFixedIso = (CheckBoxPreference) findPreference(PREF_FIXED_ISO);
    prefFixedIso.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
      @Override
      public boolean onPreferenceChange(Preference preference, Object newValue) {
        populateIsoList(newValue);
        return true;
      }
    });

    populateFocusDistance(null);
    populateResolutionList((ListPreference) findPreference(SettingsConstants.PREF_RESOLUTIONS));
    populateWhitebalanceList();
    populateIsoList(null);

    // initial summary setting
    final SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(getActivity());
    findPreference(PREF_FOCUS_DIST).setSummary(prefs.getString(PREF_FOCUS_DIST, ""));
    findPreference(PREF_WHITE_BALANCE).setSummary(StringConverters
        .whiteBalanceModeToString(Integer.parseInt(prefs.getString(PREF_WHITE_BALANCE, "-1"))));
  }

  public void populateFocusDistance(Object val) {
    final CheckBoxPreference prefFixedFocusDist =
        (CheckBoxPreference) findPreference(PREF_FIXED_FOCUS_DIST);
    final Preference prefFocusDist = findPreference(PREF_FOCUS_DIST);
    final boolean isChecked = (val == null) ? prefFixedFocusDist.isChecked() : val.equals(true);
    prefFocusDist.setEnabled(isChecked);
  }

  public void populateIsoList(Object val) {
    final ListPreference prefIso = (ListPreference) findPreference(PREF_ISO);
    CheckBoxPreference pref_fix_iso = (CheckBoxPreference) findPreference(PREF_FIXED_ISO);

    try {
      //FIXME: find a way to check if manual_sensor is in capabilities instead of catching this
      Range<Integer> sensitivityRange =
          characteristics.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
      final int maxIsoSensitivity = sensitivityRange.getUpper();//10000
      final int minIsoSensitivity = sensitivityRange.getLower();//100
      final List<String> isoSensitivities = new ArrayList<String>();
      for (int i = minIsoSensitivity; i < maxIsoSensitivity; i += 50) {
        isoSensitivities.add(Integer.toString(i));
      }
      final CharSequence[] isoSensitivitiesArray = isoSensitivities.toArray(new CharSequence[0]);

      prefIso.setEnabled(true);
      prefIso.setEntries(isoSensitivitiesArray);
      prefIso.setDefaultValue(isoSensitivitiesArray[0]);
      prefIso.setEntryValues(isoSensitivitiesArray);
    } catch (Exception e) {
      pref_fix_iso.setEnabled(false);
      prefIso.setSummary("Not available on device");
      prefIso.setEnabled(false);
      prefIso.setDefaultValue("-1");
    }
  }

  public void populateWhitebalanceList() {
    int[] awbAvailableModes = characteristics.get(CONTROL_AWB_AVAILABLE_MODES);

    // List dialog to select resolution
    final CharSequence[] entries = new CharSequence[awbAvailableModes.length];
    final CharSequence[] entryValues = new CharSequence[awbAvailableModes.length];
    for (int awbModeIndex = 0; awbModeIndex < awbAvailableModes.length; ++awbModeIndex) {
      final int awbMode = awbAvailableModes[awbModeIndex];
      entries[awbModeIndex] = StringConverters.whiteBalanceModeToString(awbMode);
      entryValues[awbModeIndex] = Integer.toString(awbMode);
    }

    final ListPreference prefWhiteBalance = (ListPreference) findPreference(PREF_WHITE_BALANCE);
    prefWhiteBalance.setEntries(entries);
    prefWhiteBalance.setDefaultValue(Integer.toString(CONTROL_AWB_MODE_AUTO));
    prefWhiteBalance.setEntryValues(entryValues);
  }

  public static void populateResolutionList(ListPreference prefResolutions) {
    // All the possible profiles that we may be interested in.
    final List<Pair<Integer, String>> possibleProfiles = Arrays
        .asList(new Pair<>(CamcorderProfile.QUALITY_480P, "480p"),
            new Pair<>(CamcorderProfile.QUALITY_720P, "720p"),
            new Pair<>(CamcorderProfile.QUALITY_1080P, "1080p"),
            new Pair<>(CamcorderProfile.QUALITY_2160P, "2160p"),
            new Pair<>(CamcorderProfile.QUALITY_CIF, "CIF (352 x 288)"),
            new Pair<>(CamcorderProfile.QUALITY_QVGA, "QVGA (320x240)"),
            new Pair<>(CamcorderProfile.QUALITY_QCIF, "QCIF (176 x 144)"));

    final List<CharSequence> supportedProfileIds = new ArrayList<>();
    final List<CharSequence> supportedProfileLabels = new ArrayList<>();
    for (Pair<Integer, String> profile : possibleProfiles) {
      if (CamcorderProfile.hasProfile(profile.first)) {
        supportedProfileIds.add(Integer.toString(profile.first));
        supportedProfileLabels.add(profile.second);
      }
    }

    if (supportedProfileIds.isEmpty()) {
      throw new AssertionError("Could not find a supported video recording profile.");
    }

    prefResolutions.setEntries(supportedProfileLabels.toArray(new CharSequence[0]));
    prefResolutions.setEntryValues(supportedProfileIds.toArray(new CharSequence[0]));
    prefResolutions.setDefaultValue(supportedProfileIds.get(0));
  }

  @Override
  public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
    updatePreference(findPreference(key), key);
  }

  private void updatePreference(Preference preference, String key) {
    if (preference == null) {
      return;
    }

    if (preference instanceof ListPreference) {
      final ListPreference listPreference = (ListPreference) preference;
      listPreference.setSummary(listPreference.getEntry());
    } else if (preference instanceof CheckBoxPreference) {
      return;
    } else {
      final SharedPreferences sharedPrefs = getPreferenceManager().getSharedPreferences();
      preference.setSummary(sharedPrefs.getString(key, "Default"));
    }
  }
}
