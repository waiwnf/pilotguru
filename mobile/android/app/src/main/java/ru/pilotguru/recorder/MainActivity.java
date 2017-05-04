package ru.pilotguru.recorder;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Matrix;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.location.Criteria;
import android.location.LocationListener;
import android.location.LocationManager;
import android.media.CamcorderProfile;
import android.os.Bundle;
import android.os.Environment;
import android.preference.PreferenceManager;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.Surface;
import android.view.TextureView;
import android.view.TextureView.SurfaceTextureListener;
import android.view.View;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;

public class MainActivity extends Activity {
  // UI - text fields.
  private TextView textViewBattery;
  private TextView textViewCoords;
  private TextView textViewFps;
  private TextView textViewImu;
  private TextView textViewCamera;
  // UI - video preview panel.
  private TextureView textureView;
  // UI - buttons.
  private ImageButton takePictureButton, settingsButton;

  private static final int REQUEST_ALL_PERMISSIONS = 200;
  private static final String[] necessaryPermissions =
      {Manifest.permission.CAMERA, Manifest.permission.WRITE_EXTERNAL_STORAGE,
          Manifest.permission.ACCESS_FINE_LOCATION};

  // Status bits for camera connections.
  private boolean isPreviewTextureAvailable = false;
  private boolean isCameraOpened = false;

  private CameraDevice cameraDevice;
  private CameraCaptureSession cameraCaptureSession;
  private SensorAndVideoRecorder recorder = new SensorAndVideoRecorder(this,
      new File(Environment.getExternalStorageDirectory(), "PilotGuru"));

  private final SurfaceTextureListener textureStatusListener = new SurfaceTextureListener() {
    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
      setPreviewTextureAvailable(true);
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
      setPreviewTextureAvailable(false);
      return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
    }
  };

  private final CameraDevice.StateCallback cameraStatePreviewCallback =
      new CameraDevice.StateCallback() {
        @Override
        public void onOpened(@NonNull CameraDevice camera) {
          cameraDevice = camera;
          createCameraSession(camera, Arrays.<Surface>asList(), null);
          takePictureButton.setOnClickListener(recordButtonListener);
          isCameraOpened = true;
        }

        @Override
        public void onDisconnected(@NonNull CameraDevice camera) {
          cameraDevice = camera;
          closeCamera();
        }

        @Override
        public void onError(@NonNull CameraDevice camera, int error) {
          cameraDevice = camera;
          isCameraOpened = false;
        }
      };

  private CamcorderProfile effectiveCamcorderProfile() {
    final CaptureSettings captureSettings =
        new CaptureSettings(PreferenceManager.getDefaultSharedPreferences(this));
    return captureSettings.getSupportedQualityProfile();
  }

  private void maybeStopRecording() {
    if (recorder.isRecording()) {
      try {
        // Stop recording the video and sensors.
        cameraCaptureSession.stopRepeating();
        cameraCaptureSession.close();
        recorder.stop(this);

        // Update buttons status.
        takePictureButton.setImageResource(R.mipmap.icon_rec);
        settingsButton.setEnabled(true);
        Toast.makeText(getApplicationContext(), "Stopped", Toast.LENGTH_LONG).show();
      } catch (CameraAccessException e) {
        Errors.dieOnException(MainActivity.this, e, "Camera access exception, exiting.");
      }
    }
  }

  private final View.OnClickListener recordButtonListener = new View.OnClickListener() {
    @Override
    public void onClick(View v) {
      if (recorder.isRecording()) {
        maybeStopRecording();
        // Recreate the preview-only capture session.
        createCameraSession(cameraDevice, Arrays.<Surface>asList(), null);
      } else {
        // Update buttons status.
        Toast.makeText(getApplicationContext(), "Started", Toast.LENGTH_LONG).show();
        takePictureButton.setImageResource(R.mipmap.icon_rec_on);
        settingsButton.setEnabled(false);

        try {
          // Stop preview-only capture session, replace it with video recording session.
          cameraCaptureSession.stopRepeating();
          cameraCaptureSession.close();
          final int displayRotationEnum = getWindowManager().getDefaultDisplay().getRotation();
          final Surface videoRecorderSurface = recorder
              .start(effectiveCamcorderProfile(), textViewFps, textViewCamera, displayRotationEnum,
                  getCameraCharacteristics());
          createCameraSession(cameraDevice, Arrays.asList(videoRecorderSurface),
              recorder.getSensorDataSaver());
        } catch (CameraAccessException e) {
          Errors.dieOnException(MainActivity.this, e, "Camer access exception, exiting.");
        } catch (IOException e) {
          Errors.dieOnException(MainActivity.this, e, "IO exception, exiting.");
        }
      }
    }
  };

  private final View.OnClickListener settingsButtonListener = new View.OnClickListener() {
    @Override
    public void onClick(View v) {
      Intent intent = new Intent(MainActivity.this, SettingsActivity.class);
      // on android 6.0, camera needs to be closed before starting this new intent
      startActivity(intent);
    }
  };

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    PreferenceManager.setDefaultValues(this, R.xml.preferences, false /* readAgain */);

    setContentView(R.layout.activity_main);

    textViewBattery = (TextView) findViewById(R.id.textview_battery);
    textViewCoords = (TextView) findViewById(R.id.textview_coords);
    textViewImu = (TextView) findViewById(R.id.textview_imu);
    textViewFps = (TextView) findViewById(R.id.textview_fps);
    textViewCamera = (TextView) findViewById(R.id.textview_camera);

    textureView = (TextureView) findViewById(R.id.texture);
    if (textureView == null) {
      throw new AssertionError("Preview texture not found in resources.");
    }

    takePictureButton = (ImageButton) findViewById(R.id.btn_takepicture);
    if (takePictureButton == null) {
      throw new AssertionError("Take picture button not found in resources.");
    }

    settingsButton = (ImageButton) findViewById(R.id.btn_settings);
    if (settingsButton == null) {
      throw new AssertionError("Settings button not found in resources.");
    }
  }

  @Override
  protected void onStart() {
    super.onStart();
    textureView.setSurfaceTextureListener(textureStatusListener);

    // Check which of the necessary permissions we do not have yet.
    final List<String> lackingPermissions = new LinkedList<>();
    for (final String permission : necessaryPermissions) {
      if (ContextCompat.checkSelfPermission(this, permission) !=
          PackageManager.PERMISSION_GRANTED) {
        lackingPermissions.add(permission);
      }
    }

    if (!lackingPermissions.isEmpty()) {
      ActivityCompat.requestPermissions(this, lackingPermissions.toArray(new String[0]),
          REQUEST_ALL_PERMISSIONS);
      // The rest is in onRequestPermissionsResult().
    } else {
      // All permissions have been granted already.
      maybeConnectAllSensors();
    }

  }

  @Override
  protected void onResume() {
    maybeConnectCamera();
    super.onResume();
  }

  @Override
  protected void onStop() {
    maybeStopRecording();
    if (cameraDevice != null) {
      closeCamera();
    }
    super.onStop();
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                         @NonNull int[] grantResults) {
    if (requestCode == REQUEST_ALL_PERMISSIONS) {
      for (int i = 0; i < permissions.length; ++i) {
        if (grantResults[i] != PackageManager.PERMISSION_GRANTED) {
          // close the app
          final String errorMessage = String
              .format(Locale.US, "Sorry, you can't use this app without granting permission %s",
                  permissions[i]);
          Toast.makeText(this, errorMessage, Toast.LENGTH_LONG).show();
          finish();
          return;
        }
      }
    }

    maybeConnectAllSensors();
  }

  private synchronized void setPreviewTextureAvailable(boolean isAvailable) {
    isPreviewTextureAvailable = isAvailable;
    if (isAvailable) {
      maybeConnectAllSensors();
    }
  }

  private synchronized void maybeConnectAllSensors() {
    if (isCameraOpened || !isPreviewTextureAvailable || !isHaveAllPermissions()) {
      return;
    }

    subscribeToLocationUpdates(recorder.getSensorDataSaver(), 20 /* minTimeMsec */);
    subscribeToImuUpdates(recorder.getSensorDataSaver(), SensorManager.SENSOR_DELAY_FASTEST);
    subscribeToLocationUpdates(new PreviewCoordinatesTextUpdater(textViewCoords), 100 /* minTimeMsec */);
    subscribeToImuUpdates(new PreviewImuTextUpdater(textViewImu, 500 /* minUpdateIntervalMillis */),
        SensorManager.SENSOR_DELAY_NORMAL);

    maybeConnectCamera();
  }

  private synchronized void maybeConnectCamera() {
    if (isCameraOpened || !isPreviewTextureAvailable || !isHaveAllPermissions()) {
      return;
    }

    try {
      final CameraManager cameraManager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
      final String cameraId = cameraManager.getCameraIdList()[0];
      cameraManager.openCamera(cameraId, cameraStatePreviewCallback, null);
      settingsButton.setOnClickListener(settingsButtonListener);
      isCameraOpened = true;
    } catch (CameraAccessException e) {
      Errors.dieOnException(this, e, "Camera access error occured, exiting.");
    }
  }

  private synchronized void closeCamera() {
    maybeStopRecording();
    takePictureButton.setOnClickListener(null);
    cameraDevice.close();
    isCameraOpened = false;
  }

  private synchronized boolean isHaveAllPermissions() {
    for (final String permission : necessaryPermissions) {
      if (ContextCompat.checkSelfPermission(this, permission) !=
          PackageManager.PERMISSION_GRANTED) {
        return false;
      }
    }
    return true;
  }

  protected void createCameraSession(@NonNull CameraDevice camera,
                                     @NonNull Collection<Surface> nonPreviewSurfaces,
                                     final @Nullable CameraCaptureSession.CaptureCallback captureListener) {
    final SurfaceTexture texture = textureView.getSurfaceTexture();
    if (texture == null) {
      throw new AssertionError("Preview surface texture is null");
    }

    try {
      final CamcorderProfile camcorderProfile = effectiveCamcorderProfile();
      texture.setDefaultBufferSize(camcorderProfile.videoFrameWidth,
          camcorderProfile.videoFrameHeight);
      configureTransform(camcorderProfile, textureView.getWidth(), textureView.getHeight());
      // Non-preview surfaces are those used for video recording. When the video is not recorded,
      // the non-preview surfaces collection may be empty.
      final List<Surface> surfaces = new LinkedList(nonPreviewSurfaces);
      surfaces.add(new Surface(texture));  // Preview surface is always used.
      final CaptureSettings captureSettings =
          new CaptureSettings(PreferenceManager.getDefaultSharedPreferences(this));
      final CaptureRequest.Builder captureRequestBuilder = captureSettings
          .makeCaptureRequestBuilder(camera, surfaces,
              getWindowManager().getDefaultDisplay().getRotation());
      final CameraCaptureSession.StateCallback captureSessionStateCallback =
          new CameraCaptureSession.StateCallback() {
            @Override
            public void onConfigured(@NonNull CameraCaptureSession session) {
              try {
                cameraCaptureSession = session;
                session.setRepeatingRequest(captureRequestBuilder.build(), captureListener, null);
              } catch (CameraAccessException e) {
                Errors
                    .dieOnException(MainActivity.this, e, "Camera access error occured, exiting.");
              }
            }

            @Override
            public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) {
              Toast.makeText(MainActivity.this, "Configuration change", Toast.LENGTH_SHORT).show();
            }
          };
      camera.createCaptureSession(surfaces, captureSessionStateCallback, null);
    } catch (CameraAccessException e) {
      Errors.dieOnException(this, e, "Camera access error occured, exiting.");
    }
  }

  private CameraCharacteristics getCameraCharacteristics() throws CameraAccessException {
    final String cameraId = cameraDevice.getId();
    final CameraManager cameraManager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
    return cameraManager.getCameraCharacteristics(cameraId);
  }

  private void configureTransform(CamcorderProfile camcorderProfile, int viewWidth,
                                  int viewHeight) {
    final int rotation = getWindowManager().getDefaultDisplay().getRotation();
    final Matrix transform = new Matrix();
    final RectF viewRect = new RectF(0, 0, viewWidth, viewHeight);
    final RectF bufferRect =
        new RectF(0, 0, camcorderProfile.videoFrameHeight, camcorderProfile.videoFrameWidth);
    if (Surface.ROTATION_90 == rotation || Surface.ROTATION_270 == rotation) {
      bufferRect.offset(viewRect.centerX() - bufferRect.centerX(),
          viewRect.centerY() - bufferRect.centerY());
      transform.setRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.FILL);
      final float scale = Math.max((float) viewHeight / camcorderProfile.videoFrameHeight,
          (float) viewWidth / camcorderProfile.videoFrameWidth);
      transform.postScale(scale, scale, viewRect.centerX(), viewRect.centerY());
      transform.postRotate(90 * (rotation - 2), viewRect.centerX(), viewRect.centerY());
    } else if (Surface.ROTATION_180 == rotation) {
      transform.postRotate(180, viewRect.centerX(), viewRect.centerY());
    }
    textureView.setTransform(transform);
  }

  private void subscribeToLocationUpdates(LocationListener listener, long minTimeMsec) {
    final LocationManager locationManager = (LocationManager) getSystemService(LOCATION_SERVICE);
    final String bestProvider = locationManager.getBestProvider(new Criteria(), false);
    locationManager.requestLocationUpdates(bestProvider, minTimeMsec, 0.01f, listener);
  }

  private void subscribeToImuUpdates(SensorEventListener listener, int delay) {
    final SensorManager sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
    sm.registerListener(listener, sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE), delay);
    sm.registerListener(listener, sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER), delay);
  }
}
