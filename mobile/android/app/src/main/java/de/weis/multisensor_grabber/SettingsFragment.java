package de.weis.multisensor_grabber;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.net.Uri;
import android.os.Bundle;
import android.preference.CheckBoxPreference;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceFragment;
import android.preference.PreferenceManager;
import android.util.Range;
import android.util.Size;

import java.util.ArrayList;
import java.util.List;

import static android.hardware.camera2.CameraCharacteristics.CONTROL_AE_AVAILABLE_MODES;
import static android.hardware.camera2.CameraCharacteristics.CONTROL_AWB_AVAILABLE_MODES;
import static android.hardware.camera2.CameraMetadata.CONTROL_AE_MODE_OFF;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_AUTO;

import static de.weis.multisensor_grabber.SettingsConstants.PREF_DIR;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_EXPOSURE;
import static de.weis.multisensor_grabber.SettingsConstants.PREF_FIXED_EXPOSURE;
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
  SharedPreferences prefs;

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    // Load the preferences from an XML resource
    addPreferencesFromResource(R.xml.preferences);

    getPreferenceScreen().getSharedPreferences().registerOnSharedPreferenceChangeListener(this);

    prefs = PreferenceManager.getDefaultSharedPreferences(getActivity());

    findPreference(PREF_DIR)
        .setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
          @Override
          public boolean onPreferenceClick(Preference preference) {
            Intent intent = new Intent(Intent.ACTION_VIEW);
            Uri selected = Uri.parse(prefs.getString(PREF_DIR, ""));
            intent.setDataAndType(selected, "resource/folder");

            if (intent.resolveActivityInfo(getActivity().getPackageManager(), 0) != null) {
              startActivity(intent);
            }
            return true;
          }
        });

    final CameraManager manager =
        (CameraManager) getContext().getSystemService(Context.CAMERA_SERVICE);
    try {
      final String cameraId = manager.getCameraIdList()[0];
      characteristics = manager.getCameraCharacteristics(cameraId);
    } catch (CameraAccessException e) {
      e.printStackTrace();
    }

    final CheckBoxPreference prefFixedExposure =
        (CheckBoxPreference) findPreference(PREF_FIXED_EXPOSURE);
    prefFixedExposure.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
      @Override
      public boolean onPreferenceChange(Preference preference, Object newValue) {
        populateExposureList(newValue);
        return true;
      }
    });

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
    populateExposureList(null);
    populateResolutionList();
    populateWhitebalanceList();
    populateIsoList(null);

    // initial summary setting
    findPreference(PREF_DIR).setSummary(prefs.getString(PREF_DIR, ""));
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

  public void populateExposureList(Object val) {
    CheckBoxPreference prefFixedExposure = (CheckBoxPreference) findPreference(PREF_FIXED_EXPOSURE);
    final Preference exposurePreference = (Preference) findPreference(PREF_EXPOSURE);

    boolean autoExposureOffSupported = false;
    for (Integer mode : characteristics.get(CONTROL_AE_AVAILABLE_MODES)) {
      if (mode == CONTROL_AE_MODE_OFF) {
        autoExposureOffSupported = true;
        break;
      }
    }

    if (!autoExposureOffSupported) {
      prefFixedExposure.setChecked(false);
      prefFixedExposure.setEnabled(false);
      exposurePreference.setEnabled(false);
      exposurePreference.setSummary("Not supported by device");
    } else {
      final boolean isChecked = (val == null) ? prefFixedExposure.isChecked() : val.equals(true);
      exposurePreference.setEnabled(isChecked);
      if (isChecked) {
        exposurePreference.setSummary("Enabled");
        findPreference(PREF_EXPOSURE).setSummary(prefs.getString(PREF_EXPOSURE, ""));
      } else {
        exposurePreference.setSummary("Fixed exposure not enabled");
      }
    }
  }

  public void populateResolutionList() {
    if (characteristics == null) {
      return;
    }

    final Size[] sizes = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
        .getOutputSizes(ImageFormat.JPEG);

    // List dialog to select resolution
    final CharSequence[] entries = new CharSequence[sizes.length];
    final CharSequence[] entryValues = new CharSequence[sizes.length];
    for (int sizeId = 0; sizeId < sizes.length; ++sizeId) {
      final Size size = sizes[sizeId];
      entries[sizeId] = size.toString();
      entryValues[sizeId] = Integer.toString(sizeId);
    }

    final ListPreference prefResolutions =
        (ListPreference) findPreference(SettingsConstants.PREF_RESOLUTIONS);
    prefResolutions.setEntries(entries);
    prefResolutions.setDefaultValue("0");
    prefResolutions.setEntryValues(entryValues);
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
