package de.weis.multisensor_grabber;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.ImageFormat;
import android.hardware.Camera;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.net.Uri;
import android.os.Bundle;
import android.preference.CheckBoxPreference;
import android.preference.EditTextPreference;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceFragment;
import android.preference.PreferenceManager;
import android.util.Log;
import android.util.Range;
import android.util.Size;

import java.util.ArrayList;
import java.util.List;

import static android.hardware.camera2.CameraCharacteristics.CONTROL_AE_AVAILABLE_MODES;
import static android.hardware.camera2.CameraCharacteristics.CONTROL_AWB_AVAILABLE_MODES;
import static android.hardware.camera2.CameraMetadata.CONTROL_AE_MODE_OFF;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_AUTO;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_CLOUDY_DAYLIGHT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_DAYLIGHT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_FLUORESCENT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_INCANDESCENT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_OFF;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_SHADE;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_TWILIGHT;
import static android.hardware.camera2.CameraMetadata.CONTROL_AWB_MODE_WARM_FLUORESCENT;
import static android.hardware.camera2.CameraMetadata.REQUEST_AVAILABLE_CAPABILITIES_MANUAL_SENSOR;

/**
 * Created by weis on 27.12.16.
 */

public class SettingsFragment extends PreferenceFragment
    implements SharedPreferences.OnSharedPreferenceChangeListener {
  android.hardware.camera2.CameraManager manager;
  CameraCharacteristics characteristics = null;
  SharedPreferences prefs;

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);

    // Load the preferences from an XML resource
    addPreferencesFromResource(R.xml.preferences);

    getPreferenceScreen().getSharedPreferences().registerOnSharedPreferenceChangeListener(this);

    prefs = PreferenceManager.getDefaultSharedPreferences(getActivity());

    findPreference("pref_dir").setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
      @Override
      public boolean onPreferenceClick(Preference preference) {
        Intent intent = new Intent(Intent.ACTION_VIEW);
        Uri selected = Uri.parse(prefs.getString("pref_dir", ""));
        intent.setDataAndType(selected, "resource/folder");

        if (intent.resolveActivityInfo(getActivity().getPackageManager(), 0) != null) {
          startActivity(intent);
        }
        return true;
      }
    });

    manager = (CameraManager) getContext().getSystemService(Context.CAMERA_SERVICE);
    String cameraId = null;
    try {
      cameraId = manager.getCameraIdList()[0];
      characteristics = manager.getCameraCharacteristics(cameraId);
    } catch (CameraAccessException e) {
      e.printStackTrace();
    }

    final CheckBoxPreference pref_fix_exp = (CheckBoxPreference) findPreference("pref_fix_exp");
    pref_fix_exp.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
      @Override
      public boolean onPreferenceChange(Preference preference, Object newValue) {
        populate_exposure_list(newValue);
        return true;
      }
    });

    final CheckBoxPreference pref_fix_foc = (CheckBoxPreference) findPreference("pref_fix_foc");
    pref_fix_foc.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
      @Override
      public boolean onPreferenceChange(Preference preference, Object newValue) {
        populate_focus_dist(newValue);
        return true;
      }
    });

    final CheckBoxPreference pref_fix_iso = (CheckBoxPreference) findPreference("pref_fix_iso");
    pref_fix_foc.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
      @Override
      public boolean onPreferenceChange(Preference preference, Object newValue) {
        populate_iso_list(newValue);
        return true;
      }
    });


    populate_focus_dist(null);
    populate_exposure_list(null);
    populate_resolution_list();
    populate_whitebalance_list();
    populate_iso_list(null);

    // initial summary setting
    findPreference("pref_dir").setSummary(prefs.getString("pref_dir", ""));
    findPreference("pref_focus_dist").setSummary(prefs.getString("pref_focus_dist", ""));
    findPreference("pref_wb").setSummary(wb2string(Integer.parseInt(prefs.getString("pref_wb", "-1"))));
    //findPreference("pref_iso").setSummary(Integer.parseInt(prefs.getString("pref_iso", "-1")));
  }

  public void populate_focus_dist(Object val) {
    CheckBoxPreference pref_fix_foc = (CheckBoxPreference) findPreference("pref_fix_foc");
    final Preference fp = findPreference("pref_focus_dist");

    boolean isChecked;

    if (val == null) {
      isChecked = pref_fix_foc.isChecked();
    } else {
      isChecked = val.equals(true);
    }

    if (isChecked) {
      fp.setEnabled(true);
    } else {
      fp.setEnabled(false);
    }
  }

  public void populate_iso_list(Object val) {
    final ListPreference lp = (ListPreference) findPreference("pref_iso");

    CheckBoxPreference pref_fix_iso = (CheckBoxPreference) findPreference("pref_fix_iso");

    boolean isChecked;

    if (val == null) {
      isChecked = pref_fix_iso.isChecked();
    } else {
      isChecked = val.equals(true);
    }

    int max1;
    int min1;

    try {
      //FIXME: find a way to check if manual_sensor is in capabilities instead of catching this
      Range<Integer> range2 = characteristics.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE);
      max1 = range2.getUpper();//10000
      min1 = range2.getLower();//100
    } catch (Exception e) {
      pref_fix_iso.setEnabled(false);
      lp.setSummary("Not available on device");
      lp.setEnabled(false);
      lp.setDefaultValue("-1");
      return;
    }
    // List dialog to select resolution
    List<String> itemslist = new ArrayList<String>();
    List<String> valueslist = new ArrayList<String>();

    int i = min1;
    while (true) {
      itemslist.add("" + i);
      valueslist.add("" + i);
      i += 50;
      if (i >= max1) break;
    }

    final CharSequence[] entries = itemslist.toArray(new CharSequence[itemslist.size()]);
    final CharSequence[] values = valueslist.toArray(new CharSequence[valueslist.size()]);

    lp.setEnabled(true);
    lp.setEntries(entries);
    lp.setDefaultValue("" + min1);
    lp.setEntryValues(values);
  }

  // FIXME: double use in MainActivity
  public String wb2string(int wb) {
    if (wb == CONTROL_AWB_MODE_CLOUDY_DAYLIGHT) return "Cloudy daylight";
    if (wb == CONTROL_AWB_MODE_DAYLIGHT) return "Daylight";
    if (wb == CONTROL_AWB_MODE_FLUORESCENT) return "Fluorescent";
    if (wb == CONTROL_AWB_MODE_INCANDESCENT) return "Incandescent";
    if (wb == CONTROL_AWB_MODE_SHADE) return "Shade";
    if (wb == CONTROL_AWB_MODE_TWILIGHT) return "Twilight";
    if (wb == CONTROL_AWB_MODE_WARM_FLUORESCENT) return "Warm Fluorescent";
    if (wb == CONTROL_AWB_MODE_OFF) return "Off";
    if (wb == CONTROL_AWB_MODE_AUTO) return "Auto";
    if (wb == -1) return "Not available";
    return "N/A: " + wb;
  }

  public void populate_whitebalance_list() {
    int[] tmp = characteristics.get(CONTROL_AWB_AVAILABLE_MODES);

    // List dialog to select resolution
    List<String> itemslist = new ArrayList<String>();
    List<String> valueslist = new ArrayList<String>();

    for (int wbval : tmp) {
      itemslist.add(wb2string(wbval));
      valueslist.add("" + wbval);
    }
    final CharSequence[] entries = itemslist.toArray(new CharSequence[itemslist.size()]);
    final CharSequence[] values = valueslist.toArray(new CharSequence[valueslist.size()]);

    final ListPreference lp = (ListPreference) findPreference("pref_wb");
    lp.setEntries(entries);
    lp.setDefaultValue("" + CONTROL_AWB_MODE_AUTO);
    lp.setEntryValues(values);
  }

  public void populate_exposure_list(Object val) {
    CheckBoxPreference pref_fix_exp = (CheckBoxPreference) findPreference("pref_fix_exp");
    final Preference ep = (Preference) findPreference("pref_exposure");

    boolean ae_off_supported = false;
    for (Integer mykey : characteristics.get(CONTROL_AE_AVAILABLE_MODES)) {
      if (mykey == CONTROL_AE_MODE_OFF) {
        ae_off_supported = true;
      }
    }

    if (!ae_off_supported) {
      pref_fix_exp.setChecked(false);
      pref_fix_exp.setEnabled(false);
      ep.setEnabled(false);
      ep.setSummary("Not supported by device");
      return;
    }

    boolean isChecked;

    if (val == null) {
      isChecked = pref_fix_exp.isChecked();
    } else {
      isChecked = val.equals(true);
    }
    if (isChecked) {
      ep.setEnabled(true);
      ep.setSummary("Enabled");
      findPreference("pref_exposure").setSummary(prefs.getString("pref_exposure", ""));
    } else {
      ep.setEnabled(false);
      ep.setSummary("Fixed exposure not enabled");
    }
  }

  public void populate_resolution_list() {

    Size[] sizes = null;

    if (characteristics != null) {
      sizes = characteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP).getOutputSizes(ImageFormat.JPEG);
    }

    // List dialog to select resolution
    List<String> itemslist = new ArrayList<String>();
    List<String> valueslist = new ArrayList<String>();

    int cnt = 0;
    for (Size size : sizes) {
      itemslist.add(size.getWidth() + "x" + size.getHeight());
      valueslist.add("" + cnt);
      cnt += 1;
    }
    final CharSequence[] entries = itemslist.toArray(new CharSequence[itemslist.size()]);
    final CharSequence[] values = valueslist.toArray(new CharSequence[valueslist.size()]);

    final ListPreference lp = (ListPreference) findPreference("pref_resolutions");
    lp.setEntries(entries);
    lp.setDefaultValue("0");
    lp.setEntryValues(values);
  }

  @Override
  public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
    updatePreference(findPreference(key), key);
  }

  private void updatePreference(Preference preference, String key) {
    if (preference == null) return;
    if (preference instanceof ListPreference) {
      ListPreference listPreference = (ListPreference) preference;
      listPreference.setSummary(listPreference.getEntry());
      return;
    }
    if (preference instanceof CheckBoxPreference) {
      return;
    }
    SharedPreferences sharedPrefs = getPreferenceManager().getSharedPreferences();
    preference.setSummary(sharedPrefs.getString(key, "Default"));
  }
}
