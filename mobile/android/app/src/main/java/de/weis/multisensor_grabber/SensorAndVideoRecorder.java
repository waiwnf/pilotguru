package de.weis.multisensor_grabber;

import android.app.Activity;
import android.content.Context;
import android.media.CamcorderProfile;
import android.media.MediaRecorder;
import android.media.MediaScannerConnection;
import android.support.annotation.NonNull;
import android.view.Surface;
import android.widget.TextView;

import java.io.File;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

public class SensorAndVideoRecorder {
  private final File storageDir;
  private final MediaRecorder videoRecorder = new MediaRecorder();
  private final SensorDataSaver sensorRecorder;
  private File videoFile;

  public SensorAndVideoRecorder(@NonNull Activity parentActivity, @NonNull File storageDir) {
    this.storageDir = storageDir;
    this.sensorRecorder = new SensorDataSaver(parentActivity);
  }

  public Surface start(@NonNull CamcorderProfile profile, TextView textViewFps,
                       TextView textViewCamera, int displayRotationEnum /* Display.getRotation() */,
                       int sensorOrientationDegrees /* CameraCharacteristics.SENSOR_ORIENTATION */)
      throws IOException {
    if (sensorRecorder.isRecording()) {
      throw new AssertionError(
          "Attempted to start recording, but recording is already in progress");
    }

    final long sequenceStartMillis = System.currentTimeMillis();
    final DateFormat dateFormat = new SimpleDateFormat("yyyy_MM_dd-HH_mm_ss");
    final File recordingDir =
        new File(storageDir, dateFormat.format(new Date(sequenceStartMillis)));
    recordingDir.mkdirs();
    sensorRecorder.start(recordingDir, textViewFps, textViewCamera);

    videoFile = new File(recordingDir, "video.mp4");
    videoRecorder.reset();
    videoRecorder.setVideoSource(MediaRecorder.VideoSource.SURFACE);
    videoRecorder.setOutputFormat(profile.fileFormat);
    videoRecorder.setVideoFrameRate(profile.videoFrameRate);
    videoRecorder.setVideoSize(profile.videoFrameWidth, profile.videoFrameHeight);
    videoRecorder.setVideoEncodingBitRate(profile.videoBitRate);
    videoRecorder.setVideoEncoder(profile.videoCodec);
    videoRecorder.setOutputFile(videoFile.getAbsolutePath());
    videoRecorder.setOrientationHint(sensorOrientationDegrees - displayRotationEnum * 90);
    videoRecorder.prepare();
    videoRecorder.start();

    return videoRecorder.getSurface();
  }

  public void stop(@NonNull Context context) {
    if (!sensorRecorder.isRecording()) {
      throw new AssertionError("Attempted to stop recording, but no recording is in progress");
    }
    sensorRecorder.stop(context);
    videoRecorder.stop();
    videoRecorder.reset();
    MediaScannerConnection.scanFile(context, new String[]{videoFile.toString()}, null, null);
  }

  public boolean isRecording() {
    return sensorRecorder.isRecording();
  }

  public SensorDataSaver getSensorDataSaver() {
    return sensorRecorder;
  }
}
