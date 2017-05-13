# Pilotguru Recorder

Available for free [on Play Store](https://play.google.com/store/apps/details?id=ru.pilotguru.recorder).

![Screenshot](https://raw.githubusercontent.com/waiwnf/pilotguru/master/mobile/android/img/readme/pilotguru-screenshot.png)

This is an Android app to record video and sensor ([GPS](https://developer.android.com/guide/topics/location/strategies.html), [accelerometer and gyroscope](https://developer.android.com/guide/topics/sensors/sensors_motion.html)) data. Every video frame and sensor reading gets a timestamp (written to a separate JSON file for every sensor) so that all the data can be then aligned in time for post-processing.

The goal here is to record raw data for training a car autopilot from demonstration. See the [pilotguru project](https://github.com/waiwnf/pilotguru) site for details on the postprocessing of the raw sensor data and (forthcoming) autopilot model training.

Pilotguru Recorder is based on [Multi-Sensor Grabber](https://github.com/TobiasWeis/android-multisensorgrabber-2) by [Tobias Weis](http://www.ccc.cs.uni-frankfurt.de/people/tobias-weis/), though the sensor handling logic has been significantly reworked. The main changes were to switch from saving raw frames to encoding a video file, and to use Android sensor timing annotations for better accuracy.

## Output format

Every recording goes to a separate directory named `PilotGuru/YYYY_MM_DD-HH_MM_SS` on the device, where recording start determines date/time. The folder contains avideo file, and several JSON files with timestamped sensor data.

**Timestamps caveat:** Android has several [clock mechanisms](https://developer.android.com/reference/android/os/SystemClock.html). Most important are `System.nanoTime()` (which is paused when the phone is in deep sleep) and `SystemClock.elapsedRealtimeNanos()` (which is always running). Though the `SensorEvent` API does not specify the base time for the [sensor event timestamps](https://developer.android.com/reference/android/hardware/SensorEvent.html#timestamp), in the lower level Android [HAL interface docs](https://source.android.com/devices/sensors/hal-interface#sensors_event_t) we find that the sensor event timestamps should be synchronized with `elapsedRealtimeNanos()`. With camera frame capture events, there is not such a guarantee. There is a [special field](https://developer.android.com/reference/android/hardware/camera2/CaptureResult.html#SENSOR_TIMESTAMP) indicating whether the frame timestamp is synchronized with `elapsedRealtimeNanos()`, and if it is not, there are no guarantees on the camera timer base in the docs at all. In practice though, on both of the devices I tested on (Nexus 5 and Nexus 5X), the frame capture timestamps appear to come from `System.nanoTime()`. So for `UNKNOWN` camera timer source the app uses the difference between `System.nanoTime()` and `SystemClock.elapsedRealtimeNanos()` to shif the frame capture timestamps to the same base as all the other sensor timestamps. If your phone uses some other camera timer base, timestamps compatibility **will break**. I do not know how often this is an issue in practice, so do let me know if you come across this problem on your device.

The recording folder contains the following files:

* `video.mp4` - this is the video stream.
* `frames.json` - video frame capture timestamps in microseconds. Example:

      {
        "frames": [
          {
            "frame_id": 0,
            "time_usec": 82019466630
          },
          {
            "frame_id": 1,
            "time_usec": 82019499823
          },
          ...
        ]
      }


* `accelerations.json` - timestamped [raw accelerometer readings](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-accel), along the three axis of the [device coordinate system](https://developer.android.com/guide/topics/sensors/sensors_overview.html#sensors-coords).

  **IMPORTANT:** Accelerometer data is **not** normalized for gravity and **not** calibrated. I have found built-in Android calibrations to be hopelessly inaccurate for long-range (minutes) motion integration, and the documentation explicitly says the linear acceleration sensor [has to be calibrated manually](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-linear) by laying the phone still and measuring the bias. Instead, we do gravity subtraction and calibration are done during postprocessing, leveraging GPS data.
  
  Example:

      {
        "accelerations": [
          {
            "x": 9.21,
            "y": 0.24,
            "z": 1.86,
            "time_usec": 82018925111
          },
          {
            "x": 9.42,
            "y": 0.29,
            "z": 1.84,
            "time_usec": 82018927651
          },
          ...
        ]
      }

* `rotations.json` - [angular velocities](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-gyro) around the three [device axes]((https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-accel)). 

Example:

      {
        "rotations": [
          {
            "x": -0.03,
            "y": -0.45,
            "z": -0.12,
            "time_usec": 82018925111
          },
          {
            "x": -0.007,
            "y": -0.43,
            "z": -0.11,
            "time_usec": 82018927651
          },
          ...
        ]
      }

* `locations.json` - [GPS locations](https://developer.android.com/guide/topics/location/strategies.html#Updates) (decimal latitude and longtitude), [location accuracies](https://developer.android.com/reference/android/location/Location.html#getAccuracy) in meters and [derived velocities](https://developer.android.com/reference/android/location/Location.html#getSpeed()). Example:

      {
        "locations": [
          {
            "lat": 56.29856204,
            "lon": 38.1406519,
            "accuracy_m": 23.0,
            "speed_m_s": 0.09,
            "time_usec": 82019490183
          },
          ...
        ]
      }
