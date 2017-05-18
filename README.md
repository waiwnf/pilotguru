# Pilotguru

* [What?](#what)
* [Current status](#current-status)
* [Requirements](#requirements)
* [Recording data](#recording-data)
* [Postprocessing - motion inference from data](#postprocessing)
  + [Installation](#installation)
  + [Vehicle motion inference from GPS and IMU data](#velocity-from-imu)
  + [Visualize results](#visualize-results)
* [Processing detals and output data format](#processing-details)
  + [Forward velocity](#forward-velocity-details)
    - [IMU processing challenges](#imu-processing-challenges)
    - [Calibration problem formulation](#calibration-problem-formulation)
    - [Local calibration](#local-calibration)
    - [Inferred velocity data format](#inferred-velocity-data-format)
  + [Steering angle](#steering-details)
    - [Computation details](#steering-computation-details")
    - [Steering angular velocity data format](#steering-data-format)

## What? <a name="what"/>

Pilotguru is a project aiming to make it possible, with just a smartphone and zero hardware work,
to gather all the necessary data to train your own computer vision based car autopilot/ADAS:
1. With [**pilotguru recorder**](https://github.com/waiwnf/pilotguru/tree/master/mobile/android), use a smartphone as a dashcam to record a ride (video + GPS data + accelerometer + gyroscope).
2. With **pilotguru**, annotate frames of the recording with horizontal angular velocity (for steering) and 
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

* Android app to record timestamped video, GPS, accelerometer and gyroscope data: [[Play Store](https://play.google.com/store/apps/details?id=ru.pilotguru.recorder)], [[source code](https://github.com/waiwnf/pilotguru/tree/master/mobile/android)].
* Forward velocity and steering angular velocity inferred from inertial measurements (accelerometer + gyroscope) and GPS data: [[how to run](#velocity-from-imu)], [[demo](https://youtu.be/HvfqpzvW2E8)].

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
3. After you have secured the phone in the windshield mount, press the *record* button and go for a ride. Remember to **not** adjust the phone in the mount while data is recorded. We assume the motion of the phone to match the motion of the car, so moving the phone relative to the car will make the data useless.
4. Once you stop the recording, the results are saved to a new directory on the phone, named like `PilotGuru/2017_04_21-14_05_32` (coresponding to the time of recording start).

    The directory will contain (see [here](https://github.com/waiwnf/pilotguru/tree/master/mobile/android#output-format) for the details on data formats)
    * `video.mp4` - the video stream.
    * `frames.json` - video frames timestamps.
    * `accelerations.json` - raw accelerometer readings.
    * `rotations.json` - raw gyroscope readings.
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
    ./build-docker.sh main
    ```
3. Build everything (inside a docker container).
    ```
    ./run-docker.sh
    mkdir build
    cd build
    cmake ..
    make -j
    ```

Everything below should be run from inside the docker container
(i.e. after executing ./run-docker.sh from pilotguru root directory).

### Vehicle motion inference from GPS and IMU data <a name="velocity-from-imu"/>

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
        --velocities_out_json /path/to/output/velocities.json \
        --steering_out_json /path/to/output/steering.json
```
The outputs are two files:
* `velocities.json` with inferred vehicle velocities for every IMU measurement. [See here](#forward-velocity-details) for computation details.
* `steering.json` with angular velocities in the inferred horizontal (i.e. road surface) plane, corresponding to car steering. [See here](#steering-details) for computation details.

### Visualize results <a name="visualize-results"/>

Render a video with steering and velocity infdormation overlayed with the original input video:
```
./render_motion \
    --in_video /path/to/video.mp4 \
    --frames_json /path/to/frames.json \
    --velocities_json /path/to/output/velocities.json \
    --steering_json /path/to/output/steering.json \
    --steering_wheel /opt/pilotguru/img/steering_wheel_image.jpg \
    --out_video /path/to/output/rendered-motion.mp4
```
This will produce a video [like this](https://youtu.be/HvfqpzvW2E8) with a rotating steering wheel and a digital speedometer tiled next to the original ride recording.
The steering wheel rotation should closely match the car steering.
    
**Caveat**: the steering wheel rotation in the rendering is somewhat misleading: it is proportional
to the *angular velocity* of the car, so for turns at slow speed the rendered steering wheel will 
rotate less than the real one in the car, and vice versa for high speed.

## Processing detals and output data format <a name="processing-details"/>

### Forward velocity <a name="forward-velocity-details"/>

We aim to combine GPS data (coarse grained, but no systematic drift over time) with inertial measurements unit data (fine-grained, but prone to systematic drift) to combine the advantages of the two data sources and get fine-grained precise velocity data.

Before diving into details, notice that GPS data by itself is actually quite precise, at least with a decent view of clear sky. Here is an example track:

![GPS-only velocity track](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/gps-only-velocity-example.png)

There are a couple of areas where we can count on improving the accuracy somewhat:
* Velocity jumps between the 30 second and 40 second marks are suspicious.
* GPS-derived velocity by its nature lags behind the true velocity changes, since we need to wait until the next measurement to compute the estimated velocity over the ~1 second interval. So during significant acceleration or braking there may be substantial difference between the GPS reported velocity (averaged over the past 1 second) and the true instantaneous velocity.

After incorporating the IMU data, we indeed see improvements on both counts. Noisy jumps are smoothed out and sustained sharp velocity changes are registered earlier:

![IMU+GPS velocity track](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/imu-gps-velocity-example.png)

#### IMU processing challenges <a name="imu-processing-challenges"/>

Some of the accelerometer processing challenges can be seen by simply looking at the raw accelerometer sensor readings from a phone lying flat on the table:

![Raw accelerometer readings per axis](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/acceleration-still-raw-per-axis.png)

One can note that
* Gravity is not subtracted away by the sensor automatically. Theoretically Android has logic to do that [on the OS level](https://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-linear), but it requires extra calibration by the user (which I am trying to avoid in the overall process if at all possible) and did not seem to be precise enough in my experience.
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

Despite looking a bit hairy, it is actually a nice quadratic objective with derivatives that are straightforward to compute analytically. It can thus be optimized efficiently with [L-BFGS](https://en.wikipedia.org/wiki/Limited-memory_BFGS), which is a standard limited-memory second order optimization algorithm, to find the optimal calibration parameters `g`, `h` and `v0`.

#### Local calibration <a name="local-calibration"/>

... aka error accumulation strikes back.

Though the [IMU calibration objective](#calibration-problem-formulation-objective) captures several important sources of trajectory integration errors, there are also aspects that it ignores. For example both unavoidable white noise in measurements and systematic gyroscope measurement bias are completely ignored. IMU integration errors compound at quadratic rate (as a function of time). Hence for long (more than a couple of minutes) recordings, even after calibration the reconstructed IMU velocities may be totally off, such as in this example (the IMU+GPS velocity is the best match with respect to the [calibration objective](#calibration-problem-formulation-objective)):

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

#### Computation details <a name="steering-computation-details"/>

Unlike accelerometer, gyroscope data does not suffer from noticeable systematic bias on our timescales (tens of minutes). The main problem with the raw data is that 
* There is no notion of the road surface plane.
* The vehicle can rotate around all 3 possible axes: in addition to vertical axis (yaw, steering), rotation is possible around both the lateral axis (pitch, e.g. when transitioning from uphill to downhill road, or coming across a speed bump) and around longitudinal axis (roll, e.g. when driving into a pothole on one side of the car).

![pitch-roll-yaw](https://raw.githubusercontent.com/waiwnf/pilotguru/master/img/readme/pitch-roll-yaw.png)

So the goal with rotation data is to infer the vertical axis in the vehicle reference frame. Then projecting instantaneous rotational velocity (in 3D) on the vertical axis yields the steering related rotation. Fortunately, the Earth locally is *almost* flat, so both pitch and roll rotations magnitude will be much smaller than steering-related yaw rotation. So we can well approximate the vertical axis as the one *around which most of the rotation happens*.

Mathematically, a convenient way to capture the main principal rotation axis is to employ a [quaternion rotation representation](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation). Rotation around an axis with direction `(ax,ay,az)` (normalized to unit length) through an angle `a` can be represented as a quaternion (4D vector):
```
q == (
  qx = ax * sin(a/2),
  qy = ay * sin(a/2),
  qz = az * sin(a/2),
  qw = sqrt(1 - x^2 - y^2 - z^2)
)
```
One can see that the first 3 components of the rotation quaternion capture both the direction and magnitude of rotation. We can then take all the recorded rotations (recorded in the moving device reference frame), integrated over intervals on the order of 1 second to reduce noise, ans apply [principal components analysis](https://en.wikipedia.org/wiki/Principal_component_analysis) to the first 3 quaternion components `(qx, qy, qz)`. The first principal component of the result will be the axis (in the moving device reference frame) that captures most of the rotation overall. This first principal component is assumed to be the vertical axis of the vehicle. We then simply project all the rotations onto that axis to get the steering (yaw) rotation component.

#### Steering angular velocity data format <a name="steering-data-format"/>

Output data is a JSON file with timestamped angular velocity around the inferred vehicle vertical axis:
```
{
  "steering": [
    {
      "angular_velocity": 0.02
      "time_usec": 567751960134
    },
    {
      "angular_velocity": -0.01
      "time_usec": 567751962709
    },
    ...
  ]
}