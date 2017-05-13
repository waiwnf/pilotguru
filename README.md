# Pilotguru

* [What?](#what)
* [Current status](#current-status)
* [Requirements](#requirements)
* [Recording data](#recording-data)
* [Postprocessing - motion inference from data](#postprocessing)
  + [Installation](#installation)
  + [Vehicle velocity inference from GPS and IMU data](#velocity-from-imu)
  + [Steering angle inference from video](#steering-from-video)
    - [Calibrate the camera](#calibrate)
    - [Process the video](#steering-process-video)
  + [Visualize results](#visualize-results)
* [Processing detals and output data format](#processing-details)
  + [Forward velocity](#forward-velocity-details)
    - [IMU processing challenges](#imu-processing-challenges)
    - [Calibration problem formulation](#calibration-problem-formulation)
    - [Local calibration](#local-calibration)
    - [Inferred velocity data format](#inferred-velocity-data-format)
  + [Steering angle](#steering-details)

## What? <a name="what"/>

Pilotguru is a project aiming to make it possible, with just a smartphone and zero hardware work,
to gather all the necessary data to train your own computer vision based car autopilot/ADAS:
1. With [**pilotguru recorder**](https://github.com/waiwnf/pilotguru/tree/master/mobile/android), use a smartphone as a dashcam to record a ride (video + GPS data + accelerometer + gyroscope).
2. With **pilotguru**, annotate frames of the recording with angular velocity (for steering) and 
   forward speed (for acceleration and braking).
    * Repeat 1-2 until you have enough data.
3. Train a machine learning model to control a car based on your garthered data.
   Pick an [existing good one](https://github.com/udacity/self-driving-car/tree/master/steering-models/community-models)
   or build your own.
4. If you are *really* feeling adventurous, intergrate your model with 
   [openpilot](https://github.com/commaai/openpilot).
   You will have to have a compatible car and get some hardware to
   get it to actually control the car. Make sure to comply with the local laws,
   you assume all the responsibility for consequences,
   this software comes with no guarantees, yada yada yada.
5. Self-driving cars solved! Enjoy ;)

## Current status <a name="current-status"/>

* Android app to record timestamped video, GPS, accelerometer and gyroscope data - [[Play Store](https://play.google.com/store/apps/details?id=ru.pilotguru.recorder)], [[source code](https://github.com/waiwnf/pilotguru/tree/master/mobile/android)].
* Forward velocity from GPS and inertial measurements accelerometer/gyroscope - [[how to run](#velocity-from-imu)], [demo].
* Steering angle data from just the video - [[how to run](#steering-from-video)] , [[demo](https://youtu.be/gMXn0IMcX-k)].

## Requirements <a name="requirements"/>

Data recording: Android phone with Marshmallow (6.0) or more recent version.

Data processing: Linux, [docker](https://www.docker.com/). Everything runs in a docker image, 
the necessary dependencies will be pulled when you build the image.

## Recording data <a name="recording-data"/>

1. Install [pilotguru recorder](https://play.google.com/store/apps/details?id=ru.pilotguru.recorder).
2. Get a windshield/dash phone mount for your car. We will use the phone motion to capture the motion of the car, 
   so the less flex and wobble in the mount, the better. Long-armed mounts 
   [like this](http://www.dhresource.com/260x260s/f2-albu-g2-M01-27-15-rBVaGlVIh1iAEctPAACSjAj2Xrk779.jpg/universal-360-degree-rotating-long-arm-car.jpg)
   are probaby not very good for our use.
   Something [like this](http://www.mobilecityonline.com/images/products/26582_L.jpg) would be better.
   Make sure the phone camera has a good view out of the windshield.
4. If you decide to tweak the app settings, do not try to go for the maximum resolution the phone can handle - the video processing logic we will need later typically struggles with the noise from high-res videos anyway. Something like 720x480 resolution should work fine.
5. After you have secured the phone in the windshield mount, press the *record* button and go for a ride. Remember to **not** adjust the phone in the mount while data is recorded. We assume the motion of the phone to match the motion of the car, so moving the phone relative to the car will make the data useless.
6. Once you stop the recording, the results are saved to a new directory on the phone, named like `PilotGuru/2017_04_21-14_05_32` (coresponding to the time of recording start).

    The directory will contain (see [here](https://github.com/waiwnf/pilotguru/tree/master/mobile/android#output-format) for the details on data formats)
    * `video.mp4` - the video stream.
    * `frames.json` - video frames timestamps.
    * `accelerations.json` - raw accelerometer readings.
    * `accelerations.json` - raw gyroscope readings.
    * `locations.json` - GPS data (locations and velocities).

    Copy the whole directory to your computer for postprocessing.

## Postprocessing - motion inference from data <a name="postprocessing"/>

### Installation <a name="installation"/>

1. Clone this repo.

    ```
    git clone https://github.com/waiwnf/pilotguru.git
    cd pilotguru
    ```

2. Build the docker image (this will take a while).

    ```
    ./build-docker.sh
    ```

3. Download extra data for the SLAM (simultaneous localization and mapping) library.
We rely on [ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2) by [Raul Mur-Artal](https://github.com/raulmur).

    ```
    ./fetch-vocabulary.sh
    ```

4. Build everything (inside a docker container).

    ```
    ./run-docker.sh
    mkdir build
    cd build
    cmake ..
    make -j
    ```

Everything below should be run from inside the docker container
(i.e. after executing ./run-docker.sh from pilotguru root directory).

### Vehicle velocity inference from GPS and IMU data <a name="velocity-from-imu"/>

Velocity data comes from two separate sources with very distinct characteristics:
* GPS readings.
    * Pros: Distinct measurements have *independent errors*. This is an extremely valuable property, it means the errors *do not compound* over time.
    * Cons: Relatively coarse-grained, both in time (typically comes at around 1 update per second) and space (can be off by a few meters). May not be able to capture well important events like starting hard braking when an obstacle suddenly appears.
* Inertial measurement unit (IMU) readings: accelerometer and gyroscope.
    * Pros: Can be very fine-grained (e.g. up to 500Hz measurement rate on a Nexus 5X), so very promising for precise annotation on video frames level.
    * Cons: Inertial measurements use the immediate previous state (pose + velocity) of the device as a reference point. To get the device trajectory one needs to *integrate* the accelerations over time, so the measurement *errors accumulate over time*: acceleration measurement error on step 1 "pollutes" velocity estimates on each subsequent step. Even worse is the travel distance computation, where we need to integrate accelerations twice and the errors grow quadratically with time.

We will fuse the two data sources to combine the advantages and cancel out the drawbacks. GPS data will provide "anchor points" and let us eliminate most of the compounding errors from integrating the IMU data.

The fusion is done by `fit_motion` binary with JSON files produced by pilotguru recorder as inputs:
```
    ./fit_motion \
        --rotations_json /path/to/rotations.json \
        --accelerations_json /path/to/accelerations.json \
        --locations_json /path/to/locations.json \
        --out_json /path/to/output/velocities.json
```
The result is a file (`velocities.json`) with inferred vehicle velocities for every IMU measurement. [See here](#forward-velocity-details) for computation details.

### Steering angle inference from video <a name="steering-from-video"/>

Orientation data from GPS is harder to deal with, so instead of using GPS+IMU as we did for forward velocity, we will infer the steering angle (more precisely, angular velocity) directly from video.

#### Calibrate the camera <a name="calibrate"/>

Every camera has some lens distortion. [Calibration](https://www.mathworks.com/help/vision/ug/camera-calibration.html) is a process of estimating the distortion parameters,
so that we can warp the captured image and remove most of the distortion.

1. Print out a calibration board. Examples:
    [A4](https://github.com/artoolkit/artoolkit5/blob/master/doc/patterns/Calibration%20chessboard%20(A4).pdf),
    [US Letter](https://github.com/artoolkit/artoolkit5/blob/master/doc/patterns/Calibration%20chessboard%20(US%20Letter).pdf),
    [A3](https://github.com/zawsx/3D-vr-DepthKit/blob/master/chessboard_a3.pdf).  Larger is better. Makes sure to disable "fit to page" and other autoscaling options when you print.
2. Stick the printout somewhere so that it is flat.
3. Take a video of the printout using [pilotguru recorder](https://play.google.com/store/apps/details?id=ru.pilotguru.recorder) **with the same camera settings as you will use in the dashcam mode** in your car. Make sure to capture the calibration pattern at different angles, distances, and in different parts of the frame.
4. Grab the video from the phone, to, say, `calibration-video.mp4` on your computer.
5. Run the calibration code (substitute your board parameters for `X`, `Y` and `Z`). Do not forget 
   that board side width and height must be the number of *inner corners* on your calibration board, which is one 
   fewer than the number of *squares* of that side.
   ```
   ./calibrate \
     --board_side_width X \
     --board_side_height Y \
     --square_size Z \
     --input /path/to/calibration-video.mp4 \
     --skip_frames 10 \
     --out_file /path/to/output/calibration.yml
   ```
   Press `g` once the video shows up on the screen. In the lower right corner is the number of frames where 
   the calibration pattern was successfully detected. Make sure there was enough of those.
   If not, take a longer video and repeat.

6. Done! Don't lose `calibration.yml`, you will need it later.

#### Process the video <a name="steering-process-video"/>

1. Process the video to extract the camera motion:
    ```
    mkdir /opt/pilotguru/data/ride-trajectories
    ./optical_trajectories \
        --vocabulary_file /opt/pilotguru/data/orb-vocabulary/ORBvoc.txt \
        --camera_settings /path/to/output/calibration.yml \
        --out_dir /path/to/output/ \
        --in_video /path/to/video.mp4
    ```
    This will run the optical SLAM logic on the video, extract camera motion and write it as JSON files to
    `/path/to/output/`. The SLAM system may need some time to pick up localization,
    and also may lose tracking in the middle of the video, so there may be multiple non-overlapping tracked
    segments, with a separate JSON file written for each. Those JSON files should be treated as completely
    independent (i.e. there are no relations between world coordinate systems in different JSON files).
    
    If the SLAM system fails to start tracking at all, it may be that your camera calibration results are off.
    Try to re-calibrate a few times to see how much variance there is. Too high a resolution may also be a problem.
    
    The SLAM system estimates the full 3D trajectory of the camera (both translation and rotation), but the 
    translation information tends to be less reliable, so we only use rotation results and rely on GPS+IMU for translations/velocity.

    [See here](#steering-details) for more details on the inference steps.

2. Raw SLAM trajectory estimates tend to be quite noisy. Apply Gaussian smoothing across frames to the
    rotation values to remove the high-frequency noise:
    ```
    ./smooth_heading_directions \
    --trajectory_in_file /path/to/output/trajectory-0.json \
    --trajectory_out_file /path/to/output/trajectory-0-smoothed.json \
    --sigma 5
    ```
    Larger sigma leads to more smoothing. Try out a couple of values to see what is the smallest one that
    still removes most of the jitter.

### Visualize results <a name="visualize-results"/>

Render a video with steering and velocity infdormation overlayed with the original input video:
```
./render_turning \
    --in_video /path/to/video.mp4 \
    --trajectory_json /path/to/output/trajectory-0-smoothed.json \
    --steering_wheel /opt/pilotguru/img/steering_wheel_image.jpg \
    --out_video /path/to/output/ride-0-smoothed.mp4
```
This will produce a video [like this](https://youtu.be/gMXn0IMcX-k) with a rotating steering wheel 
tiled next to the original ride recording.
The steering wheel rotation should generally match the car steering.
    
**Caveat**: the steering wheel rotation in the rendering is somewhat misleading: it is proportional
to the *angular velocity* of the car, so for turns at slow speed the rendered steering wheel will 
rotate less than the real one in the car, and vice versa for high speed.

## Processing detals and output data format <a name="processing-details"/>

### Forward velocity <a name="forward-velocity-details"/>

We aim to combine GPS data (coarse grained, but no systematic drift over time) with inertial measurements unit data (fine-grained, but prone to systematic drift) to combine the advantages of the two data sources and get fine-grained precise velocity data.

Before diving into details, I would like to remark that GPS data by itself is actually quite precise, at least with a decent view of clear sky. Here is an example track:

![GPS-only velocity track](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/gps-only-velocity-example.png)

There are a couple of areas where we can count on improving the accuracy somewhat:
* Velocity jumps between the 30 second and 40 second marks are suspicious.
* GPS-derived velocity by its nature lags behind the true velocity changes, since we need to wait until the next measurement to compute the estimated velocity over the ~1 second interval. So during significant acceleration or braking there may be significant difference between the GPS reported velocity (averaged over the past 1 second) and the true instantaneous velocity.

After incorporating the IMU data, we indeed see improvements on both counts. Noisy jumps are smoothed out and sustained sharp velocity changes are registered earlier:

![IMU+GPS velocity track](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/imu-gps-velocity-example.png)

#### IMU processing challenges <a name="imu-processing-challenges"/>

Some of the accelerometer processing challenges can be seen by simply looking at the raw accelerometer sensor readings from a phone lying flat on the table:

![Raw accelerometer readings per axis](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/acceleration-still-raw-per-axis.png)

One can note that
* Gravity is not subtracted away by the sensor automatically. Theoretically Android has logic to do that [on the OS level](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-linear), but it requires extra calibration by the user (which I am trying to avoid in the overall process if possible) and did not seem to be precise enough in my experience.
* Overall acceleration magnitude is quite far from 1g = 9.81 m/s^2. This suggests additional bias in the sensor.

Significant sensor bias is confirmed by comparing the readings from the phone laying screen up and screen down:

![Overall measured acceleration screen up and down comparison](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/acceleration-magnitude-diff-screen-up-down.png)

#### Calibration problem formulation <a name="calibration-problem-formulation"/>

Formalizing the above section, define the calibration problem for the IMU unit to find the optimal
* `g`: accelerometer bias in the fixed reference frame (this is roughly gravity).
* `h`: accelerometer bias in the device-local reference frame (this is the bias introduced by the sensor itself).
* `v0`: Initial velocity (3D) at the beginning of the recording sequence to support recordings that start when the car is already in motion.

The optimization objective is helpfully provided by the coarse grained "mostly correct" GPS data: for every interval between two neighbnoring GPS measurements we will aim to match the travelled distance of the calibrated IMU trajectory with the distance from GPS data. Using L2 loss function, we get

![\min_{\vec{g}, \vec{h}, \vec{v}_0} \sum_{i=1}^n \left( \left \| \sum_{k=k_i}^{k_{i+1}} (\tau_k - \tau_{k-1} )\vec{v}_k^{IMU} \right \| - (t_i - t_{i-1})v^{GPS}_i \right )^2](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/calibration-objective-velocities.gif)

where
* `i` is the index of a GPS measurement.
* `k_i  ... k_(i+1)` are the range of indices for IMU measurements that fall in time between GPS measurements `i` and `i+1`.
* ![\tau_j](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/tau-j.gif) is the time of the `j`-th IMU measurement.

Next, unroll the integrated IMU velocity in terms of measured raw accelerations and calibration parameters. Denote ![\Delta \tau_j \equiv \tau_j - \tau_{j-1}](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/delta-tau-j-def.gif) and ![\Delta t_i \equiv t_i - t_{i-1}](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/delta-t-i-def.gif).

![\vec{v}_k^{IMU} = \vec{v}_0 + \sum_{j=1}^k \vec{a}_j \Delta \tau_j = \vec{v}_0 + \sum_{j=1}^k \left(R_j \cdot (\vec{a}_j^{RAW} + \vec{h}) + \vec{g} \right) \Delta \tau_j](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/calibration-imu-velocity-from-accelerations.gif)

where `R_j` is the rotation matrix from the device-local reference frame to the fixed reference time at time step `j`. The rotation matrix can itself be computed by integrating the rotational velocities measured by the gyroscope. Denoting `W` to be the derivative of the rotation matrix coming from the gyroscope, in the first approximation we have

![R_j = R_{j-1} \cdot (I + W_j \Delta \tau_j) = \prod_{m=1}^j (I + W_m \Delta \tau_m)](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/rotation-from-gyro.gif)

In reality, this linear approximation is not used. The gyroscope does not report the derivative `W` directly. Instead it reports angular velocities around the 3 device axes, from which (and time duration) it is possible to [compute the rotation matrix exactly](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-gyro). It is interesting to look at the first-order approximation though to highlight the linear time dependence.

Summing everything up, we arrive at the final calibration objective:

<a name="calibration-problem-formulation-objective"/>
![\min_{\vec{g}, \vec{h}, \vec{v}_0} \sum_{i=1}^n \left( \left \| \sum_{k=k_i}^{k_{i+1}} \Delta \tau_k \left( \vec{v}_0 + (\tau_k - \tau_0)\vec{g} + \sum_{j=1}^k \Delta \tau_j R_j \cdot (\vec{a}_j^{RAW} + \vec{h}) \right ) \right \| - v^{GPS}_i \Delta t_i \right )^2](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/calibration-objective-final.gif)

Despite looking a bit hairy, it is actually a nice quadratic objective with derivatives that are straightforward to compute analytically. It can thus be optimized efficiently with L-BFGS, which is a standard limited-memory second order optimization algorithm, to find the optimal calibration parameters `g`, `h` and `v0`.

#### Local calibration <a name="local-calibration"/>

... aka error accumulation strikes back.

Though the [IMU calibration objective](#calibration-problem-formulation-objective) captures several important sources of trajectory integration errors, there are also aspects that it ignores. For example both unavoidable white noise in measurements and systematic gyroscope measurement bias are completely ignored. IMU integration errors compound at quadratic rate (as a function of time). Hence for long (more than a couple of minutes) recordings, even after calibration the reconstructed IMU velocities may be totally off, such as in this example:

![Error accumulation - calibration not enough for long time series](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/imu-gps-velocity-long-calibration-fail.png)

On the other hand, for shorter durations (~ 1 minute), calibrated IMU data yields a well matching result:

![Calibration success for the short fragment of the time series](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/imu-gps-velocity-short-calibration-success.png)

That calibration works well on shorter time ranges suggests an easy workaround. Instead of trying to capture all possible sources of integration error and fit the corresponding calibration parameters in the model, we slide a window of moderate duration (40 seconds by default) over the recorded time series, and each time calibrate the IMU only within that sliding window. The resulting estimated velocity for each timestamp is the average of values *for that timestamp* over all sliding windows containing that time. The result is a trajectory that at each point closely follows the dynamic measured by the IMU and at the same time "sheds" the accumulating integration error by re-anchoring itself to the GPS measurements:

![Final result - long-range time series calibrated with overlapping sliding windows](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/imu-gps-velocity-sliding-window-calibrated-final.png)

#### Inferred velocity data format <a name="inferred-velocity-data-format"/>

Output data is a JSON file with timestamped absolute velocity magnitudes (no direction information) in m/s:
```
{
  "velocities": [
    {
      "speed_m_s": 0.031
      "time_usec": 567751960167
    },
    {
      "speed_m_s": 0.030
      "time_usec": 567751962707
    },
    ...
  ]
}
```

### Steering angle <a name="steering-details"/>

Steps that happen when `optical_trajectories` runs to extract the steering angle from video, and output JSON data format.

1. [SLAM (simultaneous localization and mapping) library](https://github.com/raulmur/ORB_SLAM2) infers camera motion from video, annotating every successfully tracked frame with the 6 degrees of freedom camera pose (translation + heading direction). In the output JSON file, this data is stored as `pose` field for every frame, along with the frame number in the original video (`frame_id`) and time offset from the start in microseconds (`time_usec`):
    ```
    "frame_id": 1172,
    "time_usec": 39062900,
    "pose": {
      "rotation": {
        "w": 0.895,
        "x": 0.0067,
        "y": -0.437,
        "z": 0.086
      },
      "translation": [
        -0.102,
        0.009,
        0.050
      ]
    },
    ```
    Rotations are repsresented as [unit quaternions](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation).
    
    The reference coordinatre frame is chose by the SLAM library arbitrarily, so there is no inherent notion of what the horizontal (road) plane is or what are left/right turn directions. Which is why we need the next steps..
    
2. The horizontal (i.e. road surface) plane is inferred (approximately) using [principal components analysis](https://en.wikipedia.org/wiki/Principal_component_analysis). As long as the car does not drive just in a straight line, or goes on a mountain [switchback road](https://bikealps.files.wordpress.com/2011/08/dolomites-20110815-dsc_0039.jpg), the distance traveled in any horizontal direction will be much larger than in the vertical direction. So we simply take the top two PCA components as the basis vectors of the horizontal plane. The basis vectors are stored in every JSON file as the `plane` field:
    ```
    "plane": [
      [
        0.914,
        0.097,
        0.393
      ],
      [
        -0.404,
        0.168,
        0.899
      ]
    ],
    ```

3. 3D rotation directions from SLAM are projected onto the 2D inferred horizontal plane. Result are stored for every frame as `(x,y)` unit vectors in the reference frame of the plane basis vectors:
    ```
    "planar_direction": [
      0.324955405433993,
      0.93074165275327
    ],
    ```

4. Finally, from the horizontal directions the relative horizontal rotation between the two frames is computed. We use the direction of the cross product to distinguish left and right turns:
    ```
    v = cross(prev_frame_flat_direction, current_frame_flat_direction)
    alpha = acos(dot(prev_frame_flat_direction, current_frame_flat_direction)) * sign(v_z)
    ```
    The results are stored as `turn_angle` field for every frame (in radians):
    ```
    "turn_angle": -0.0054
    ```

5. All of the above steps are prone to high-fequency noise, showing up in the SLAM camera trajectory and propagating to the rotation magnitudes. To reduce the noise, `smooth_heading_directions` binary applies [Gaussian blur](https://en.wikipedia.org/wiki/Gaussian_blur) to the 3D camera rotations and propagates the results downstream.
