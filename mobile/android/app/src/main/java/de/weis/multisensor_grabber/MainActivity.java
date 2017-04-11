package de.weis.multisensor_grabber;

import android.Manifest;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.TotalCaptureResult;
import android.location.Criteria;
import android.location.LocationManager;
import android.media.MediaRecorder;
import android.os.BatteryManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.StatFs;
import android.preference.PreferenceManager;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.os.Bundle;
import android.util.Size;
import android.util.SparseIntArray;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.widget.ImageButton;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

public class MainActivity extends Activity {
  private ImageButton takePictureButton;
  private TextureView textureView;
  private static final SparseIntArray ORIENTATIONS = new SparseIntArray();

  static {
    ORIENTATIONS.append(Surface.ROTATION_0, 90);
    ORIENTATIONS.append(Surface.ROTATION_90, 0);
    ORIENTATIONS.append(Surface.ROTATION_180, 270);
    ORIENTATIONS.append(Surface.ROTATION_270, 180);
  }

  protected CameraDevice cameraDevice;
  protected CameraCaptureSession cameraCaptureSessions;
  private static final int REQUEST_CAMERA_PERMISSION = 200;
  private Handler mBackgroundHandler;
  private HandlerThread mBackgroundThread;
  File storageDir;

  CameraManager cameraManager;
  CameraCharacteristics cameraCharacteristics;
  CameraCaptureSession cameraCaptureSession;
  CaptureRequest.Builder captureRequestBuilder; // needs to be global b/c preview-setup

  protected boolean isRecording = false;
  Size imageSize = new Size(640, 480);
  CaptureSettings captureSettings;
  SensorDataSaver sensorDataSaver;

  private Handler sysHandler = new Handler();
  private TextView textviewBattery;
  TextView textviewCoords;
  TextView textviewFps;
  TextView textviewImu;
  TextView textviewCamera;
  ImageButton settingsButton;
  MediaRecorder mediaRecorder = new MediaRecorder();

  public static float GbAvailable(File f) {
    final StatFs stat = new StatFs(f.getPath());
    final long bytesAvailable = stat.getBlockSizeLong() * stat.getAvailableBlocksLong();
    return (float) bytesAvailable / (1024.f * 1024.f * 1024.f);
  }

  private Runnable grab_system_data = new Runnable() {
    @Override
    public void run() {
      textviewBattery.setText(String.format(Locale.US, "BAT: %.01f%%", getBatteryPercent()));
      // FIXME: if we have gps-permission, but gps is off, this fails!
      try {
        final LocationManager locationManager =
            (LocationManager) getSystemService(LOCATION_SERVICE);
        final String bestProvider = locationManager.getBestProvider(new Criteria(), false);
        locationManager.requestLocationUpdates(bestProvider, 1, 0.01f, sensorDataSaver);
        // TODO add updates for the text view
      } catch (Exception e) {
      }

      final String camString = String
          .format(Locale.US, "EXP: %s, FOC: %s, ISO: %s, WB: %s, Free space: %.02f Gb",
              captureSettings.getExposureText(), captureSettings.getFocalLengthText(),
              captureSettings.getIsoValueText(),
              StringConverters.whiteBalanceModeToString(captureSettings.getWhiteBalanceMode()),
              GbAvailable(storageDir));
      textviewCamera.setText(camString);

      sysHandler.postDelayed(grab_system_data, 500);
    }
  };

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    textviewBattery = (TextView) findViewById(R.id.textview_battery);
    textviewCoords = (TextView) findViewById(R.id.textview_coords);
    textviewImu = (TextView) findViewById(R.id.textview_imu);
    textviewFps = (TextView) findViewById(R.id.textview_fps);
    textviewCamera = (TextView) findViewById(R.id.textview_camera);

    textureView = (TextureView) findViewById(R.id.texture);
    assert textureView != null;
    textureView.setSurfaceTextureListener(textureListener);

    takePictureButton = (ImageButton) findViewById(R.id.btn_takepicture);
    assert takePictureButton != null;

    try {
      storageDir = getExternalFilesDirs(null)[1];
    } catch (Exception e) {
      storageDir = getExternalFilesDirs(null)[0];
    }
    sensorDataSaver = new SensorDataSaver(storageDir);

    takePictureButton.setOnClickListener(new View.OnClickListener() {
      @Override
      public void onClick(View v) {
        if (isRecording) {
          try {
            cameraCaptureSession.stopRepeating();
            cameraCaptureSession.close();
            createCameraPreview();
            isRecording = false;
            sensorDataSaver.stop();
            mediaRecorder.stop();
            mediaRecorder.reset();
            takePictureButton.setImageResource(R.mipmap.icon_rec);
            settingsButton.setEnabled(true);
            Toast.makeText(getApplicationContext(), "Stopped", Toast.LENGTH_LONG).show();
          } catch (CameraAccessException e) {
            e.printStackTrace();
          }
        } else {
          Toast.makeText(getApplicationContext(), "Started", Toast.LENGTH_LONG).show();
          takePictureButton.setImageResource(R.mipmap.icon_rec_on);
          settingsButton.setEnabled(false);
          isRecording = true;
          takePicture();
        }
      }
    });

    settingsButton = (ImageButton) findViewById(R.id.btn_settings);
    settingsButton.setOnClickListener(new android.view.View.OnClickListener() {
      @Override
      public void onClick(View v) {
        Intent intent = new Intent(MainActivity.this, SettingsActivity.class);
        // on android 6.0, camera needs to be closed before starting this new intent

        startActivity(intent);
      }
    });

    cameraManager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);

  }

  TextureView.SurfaceTextureListener textureListener = new TextureView.SurfaceTextureListener() {
    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
      //open your camera here
      openCamera();
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
      // Transform you image captured size according to the surface width and height
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
      return false;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
    }
  };

  private final CameraDevice.StateCallback stateCallback = new CameraDevice.StateCallback() {
    @Override
    public void onOpened(@NonNull CameraDevice camera) {
      //Log.e(TAG, "onOpened");
      cameraDevice = camera;
      try {
        cameraCharacteristics = cameraManager.getCameraCharacteristics(cameraDevice.getId());

        // read preferences
        final SharedPreferences prefs =
            PreferenceManager.getDefaultSharedPreferences(MainActivity.this);

        // ------------ resolution and setup reader and output surfaces
        final String selected_res = prefs.getString("pref_resolutions", "");
        // YUV is way faster
        final int imageFormat = ImageFormat.YUV_420_888;
        if (!selected_res.equals("")) {
          // FIXME: expose to preferences!
          // FIXME: max framerate with JPEG is 150ms == ca. 6.6 fps on GS5
          // GS6: 36-40ms
          final Size[] sizes =
              cameraCharacteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP)
                  .getOutputSizes(imageFormat);
          imageSize = sizes[Integer.parseInt(selected_res)];
        }

      } catch (CameraAccessException e) {
        e.printStackTrace();
      }
      createCameraPreview();
    }

    @Override
    public void onDisconnected(@NonNull CameraDevice camera) {
      cameraDevice.close();
    }

    @Override
    public void onError(@NonNull CameraDevice camera, int error) {
      cameraDevice.close();
      cameraDevice = null;
    }
  };

  final CameraCaptureSession.CaptureCallback previewCallbackListener =
      new CameraCaptureSession.CaptureCallback() {
        @Override
        public void onCaptureCompleted(CameraCaptureSession session, CaptureRequest request,
                                       TotalCaptureResult result) {
          super.onCaptureCompleted(session, request, result);
        }
      };

  protected void startBackgroundThread() {
    mBackgroundThread = new HandlerThread("Camera Background");
    mBackgroundThread.start();
    mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
  }

  protected void stopBackgroundThread() {
    mBackgroundThread.quitSafely();
    try {
      mBackgroundThread.join();
      mBackgroundThread = null;
      mBackgroundHandler = null;
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  protected void takePicture() {
    if (null == cameraDevice) {
      return;
    }

    try {
      sensorDataSaver.start();

      mediaRecorder.reset();
      mediaRecorder.setVideoSource(MediaRecorder.VideoSource.SURFACE);
      mediaRecorder.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4);
      mediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.H264);
      // Set the same bitrate per pixel as the reference 720x480 @ 6Mbps video.
      final int videoBitrate =
          (int) ((6e+6 * imageSize.getWidth() * imageSize.getHeight()) / (720 * 480));
      mediaRecorder.setVideoEncodingBitRate(videoBitrate);
      mediaRecorder.setOutputFile((new File(storageDir, "video.mp4")).getAbsolutePath());
      mediaRecorder.setVideoSize(imageSize.getWidth(), imageSize.getHeight());
      mediaRecorder.setVideoFrameRate(30);
      mediaRecorder.prepare();
      mediaRecorder.start();

      final SurfaceTexture texture = textureView.getSurfaceTexture();
      assert texture != null;
      texture.setDefaultBufferSize(imageSize.getWidth(), imageSize.getHeight());
      configureTransform(textureView.getWidth(), textureView.getHeight());
      final List<Surface> surfaces =
          Arrays.asList(new Surface(texture), mediaRecorder.getSurface());
      captureRequestBuilder = getCaptureBuilder(surfaces);

      cameraDevice.createCaptureSession(surfaces, new CameraCaptureSession.StateCallback() {
        @Override
        public void onConfigured(@NonNull CameraCaptureSession session) {
          try {
            cameraCaptureSession = session;
            // use the same captureRequest builder as for the preview,
            // this has already been built from user preferences!
            session.setRepeatingRequest(captureRequestBuilder.build(), sensorDataSaver,
                mBackgroundHandler);
          } catch (CameraAccessException e) {
            e.printStackTrace();
          }
        }

        @Override
        public void onConfigureFailed(@NonNull CameraCaptureSession session) {
        }
      }, mBackgroundHandler);
    } catch (IOException e) {
    } catch (CameraAccessException e) {
    }
  }

  CaptureRequest.Builder getCaptureBuilder(Iterable<Surface> outputSurfaces) {
    try {
      captureSettings =
          new CaptureSettings(PreferenceManager.getDefaultSharedPreferences(MainActivity.this));
      final CaptureRequest.Builder captureBuilder =
          captureSettings.makeCaptureRequestBuilder(cameraDevice, outputSurfaces);

      // Orientation
      int rotation = getWindowManager().getDefaultDisplay().getRotation();
      captureBuilder.set(CaptureRequest.JPEG_ORIENTATION, ORIENTATIONS.get(rotation));
      return captureBuilder;
    } catch (CameraAccessException e) {
      e.printStackTrace();
      return null;
    }
  }

  protected void createCameraPreview() {
    try {
      SurfaceTexture texture = textureView.getSurfaceTexture();
      assert texture != null;

      texture.setDefaultBufferSize(imageSize.getWidth(), imageSize.getHeight());

      //Log.d("textureView", "========================= calling configureTransform");
      configureTransform(textureView.getWidth(), textureView.getHeight());

      // for the preview, we only want the preview-surface as output
      List<Surface> surfaces = Arrays.asList(new Surface(texture));
      captureRequestBuilder = getCaptureBuilder(Arrays.asList(new Surface(texture)));
      cameraDevice.createCaptureSession(surfaces, new CameraCaptureSession.StateCallback() {
        @Override
        public void onConfigured(@NonNull CameraCaptureSession cameraCaptureSession) {
          //The camera is already closed
          if (null == cameraDevice) {
            return;
          }
          // When the session is ready, we start displaying the preview.
          cameraCaptureSessions = cameraCaptureSession;
          updatePreview();
        }

        @Override
        public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) {
          Toast.makeText(MainActivity.this, "Configuration change", Toast.LENGTH_SHORT).show();
        }
      }, null);
    } catch (CameraAccessException e) {
      e.printStackTrace();
    }
  }

  private boolean isPermissionGranted(String permissionName) {
    return ActivityCompat.checkSelfPermission(this, permissionName) ==
        PackageManager.PERMISSION_GRANTED;
  }

  private void openCamera() {
    try {
      final String cameraId = cameraManager.getCameraIdList()[0];

      // Add permission for camera and let user grant the permission
      if (!isPermissionGranted(Manifest.permission.CAMERA) ||
          !isPermissionGranted(Manifest.permission.WRITE_EXTERNAL_STORAGE) ||
          !isPermissionGranted(Manifest.permission.ACCESS_FINE_LOCATION)) {
        ActivityCompat.requestPermissions(MainActivity.this,
            new String[]{Manifest.permission.CAMERA, Manifest.permission.WRITE_EXTERNAL_STORAGE,
                Manifest.permission.ACCESS_FINE_LOCATION}, REQUEST_CAMERA_PERMISSION);
      } else {
        cameraManager.openCamera(cameraId, stateCallback, null);
        final LocationManager locationManager =
            (LocationManager) getSystemService(LOCATION_SERVICE);
        final String bestProvider = locationManager.getBestProvider(new Criteria(), false);
        locationManager.requestLocationUpdates(bestProvider, 1, 0.01f, sensorDataSaver);
        // TODO add updates for the text view

        SensorManager sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        sm.registerListener(sensorDataSaver, sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
            SensorManager.SENSOR_DELAY_GAME);
        sm.registerListener(sensorDataSaver, sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
            SensorManager.SENSOR_DELAY_GAME);

        sysHandler.postDelayed(grab_system_data, 1);
      }
    } catch (CameraAccessException e) {
      e.printStackTrace();
    }
  }

  protected void updatePreview() {
    try {
      cameraCaptureSessions
          .setRepeatingRequest(captureRequestBuilder.build(), previewCallbackListener,
              mBackgroundHandler);
    } catch (CameraAccessException e) {
      e.printStackTrace();
    }
  }

  private void configureTransform(int viewWidth, int viewHeight) {
    if (null == textureView) {
      return;
    }

    int rotation = getWindowManager().getDefaultDisplay().getRotation();
    Matrix matrix = new Matrix();
    RectF viewRect = new RectF(0, 0, viewWidth, viewHeight);
    RectF bufferRect = new RectF(0, 0, imageSize.getHeight(), imageSize.getWidth());
    float centerX = viewRect.centerX();
    float centerY = viewRect.centerY();
    if (Surface.ROTATION_90 == rotation || Surface.ROTATION_270 == rotation) {
      bufferRect.offset(centerX - bufferRect.centerX(), centerY - bufferRect.centerY());
      matrix.setRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.FILL);
      final float scale = Math.max((float) viewHeight / imageSize.getHeight(),
          (float) viewWidth / imageSize.getWidth());
      matrix.postScale(scale, scale, centerX, centerY);
      matrix.postRotate(90 * (rotation - 2), centerX, centerY);
    } else if (Surface.ROTATION_180 == rotation) {
      matrix.postRotate(180, centerX, centerY);
    }
    textureView.setTransform(matrix);
  }

  private void closeCamera() {
    if (null != cameraDevice) {
      cameraDevice.close();
      cameraDevice = null;
    }
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                         @NonNull int[] grantResults) {
    if (requestCode == REQUEST_CAMERA_PERMISSION) {
      if (grantResults[0] == PackageManager.PERMISSION_DENIED) {
        // close the app
        Toast.makeText(MainActivity.this,
            "Sorry!!!, you can't use this app without granting permission", Toast.LENGTH_LONG)
            .show();
        finish();
      }
    }
  }

  @Override
  protected void onResume() {
    super.onResume();
    startBackgroundThread();

    try {
      storageDir = getExternalFilesDirs(null)[1];
    } catch (Exception e) {
      storageDir = getExternalFilesDirs(null)[0];
    }

    /* put it in the prefs so the user can find the files later */
    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(MainActivity.this);
    try {
      prefs.edit().putString("pref_dir", storageDir.toString()).apply();
    } catch (Exception e) {
      Toast.makeText(this, "Setting external directory failed", Toast.LENGTH_LONG);
    }

    if (textureView.isAvailable()) {
      openCamera();
    } else {
      textureView.setSurfaceTextureListener(textureListener);
    }
  }

  @Override
  protected void onPause() {
    isRecording = false;
    takePictureButton.setImageResource(R.mipmap.icon_rec);
    settingsButton.setEnabled(true);
    closeCamera();
    stopBackgroundThread();
    super.onPause();
  }

  public float getBatteryPercent() {
    final int INVALID_VALUE = -1;
    final Intent batteryIntent =
        registerReceiver(null, new IntentFilter(Intent.ACTION_BATTERY_CHANGED));
    final int level = batteryIntent.getIntExtra(BatteryManager.EXTRA_LEVEL, INVALID_VALUE);
    final int scale = batteryIntent.getIntExtra(BatteryManager.EXTRA_SCALE, INVALID_VALUE);

    // Error checking that probably isn't needed but I added just in case.
    if (level == INVALID_VALUE || scale == INVALID_VALUE) {
      return 50.0f;
    }

    return ((float) level / (float) scale) * 100.0f;
  }
}
