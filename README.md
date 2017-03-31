# Pilotguru

**! Work in progress !**<br/>
(you have been warned)

* [What?](#what)
* [Current status](#current-status)
* [How to use](#how-to-use)
  + [Requirements](#requirements)
  + [Installation](#installation)
  + [Calibrate your smartphone camera](#calibrate)
  + [Steering angle from video](#steering-from-video)
    - [Steering angle inference - details](#steering-details)

## What? <a name="what"/>

Pilotguru is a project aiming to make it possible, with just a smartphone and zero hardware work,
to gather all the necessary data to train your own computer vision based car autopilot/ADAS:
1. Use a smartphone as a dashcam to record a ride (video + GPS data + accelerometer).
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

* Steering angle data from just the video - [available](https://youtu.be/gMXn0IMcX-k).
* Forward velocity from video/GPS/accelerometer - under development, nothing to show yet.

## How to use <a name="how-to-use"/>

Pretty easy actually, see below.

### Requirements <a name="requirements"/>

Linux, [docker](https://www.docker.com/). Everything runs in a docker image, 
the necessary dependencies will be pulled when you build the image.

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

### Calibrate the camera <a name="calibrate"/>

Every camera has some lens distortion. [Calibration](https://www.mathworks.com/help/vision/ug/camera-calibration.html) is a process of estimating the distortion parameters,
so that we can warp the captured image and remove most of the distortion.

1. Print out a calibration board. Examples:
    [A4](https://github.com/artoolkit/artoolkit5/blob/master/doc/patterns/Calibration%20chessboard%20(A4).pdf),
    [US Letter](https://github.com/artoolkit/artoolkit5/blob/master/doc/patterns/Calibration%20chessboard%20(US%20Letter).pdf),
    [A3](https://github.com/zawsx/3D-vr-DepthKit/blob/master/chessboard_a3.pdf).  Larger is better. Makes sure to disable "fit to page" and other autoscaling options when you print.
2. Stick the printout somewhere so that it is flat.
3. Take a video of the printout **with the same camera settings as you will use in the dashcam mode** in your car.
   Do not try to go for the maximum resolution the phone can handle - the SLAM system typically struggles with the
   noise from high-res videos anyway. Something like 720x480 resolution should work fine.
   Make sure to capture the calibration pattern at different angles, distances, and in different parts of the frame.
4. Grab the video from the phone, to, say, `pilotguru/data/calibration-video.mp4`.
5. Run the calibration code (substitute your board parameters for `X`, `Y` and `Z`). Do not forget 
   that board side width and height must be the numbr of *inner corners* on your calibration board, which is one 
   fewer than the number of *squares* of that side.
   ```
   ./calibrate \
     --board_side_width X \
     --board_side_height Y \
     --square_size Z \
     --input /opt/pilotguru/data/calibration-video.mp4 \
     --skip_frames 10 \
     --out_file /opt/pilotguru/data/calibration.yml
   ```
   Press `g` once the video shows up on the screen. In the lower right corner is the number of frames where 
   the calibration pattern was successfully detected. Make sure there was enough of those.
   If not, take a longer video and repeat.

6. Done! Don't lose `calibration.yml`, you will need it later.

### Steering angle from video <a name="steering-from-video"/>

1. Get a windshield/dash phone mount for your car. We will use the phone motion to capture the motion of the car, 
   so the less flex and wobble in the mount, the better. Long-armed mounts 
   [like this](http://www.dhresource.com/260x260s/f2-albu-g2-M01-27-15-rBVaGlVIh1iAEctPAACSjAj2Xrk779.jpg/universal-360-degree-rotating-long-arm-car.jpg)
   are probaby not very good for our use.
   Something (like this)[http://www.mobilecityonline.com/images/products/26582_L.jpg] would be better.
   Make sure the phone camera has a good view out of the windshield. 
2. Go for a ride! Set the phone to record a video. Remember to **use the same settings as you were using for camera calibration**.
3. Back at your computer, grab the video from the phone, call it `pilotguru/data/ride.mp4`.
4. Process the video to extract the camera motion:
    ```
    mkdir /opt/pilotguru/data/ride-trajectories
    ./optical_trajectories \
        --vocabulary_file /opt/pilotguru/data/orb-vocabulary/ORBvoc.txt \
        --camera_settings /opt/pilotguru/data/calibration.yml \
        --out_dir /opt/pilotguru/data/ride-trajectories \
        --in_video /opt/pilotguru/data/ride.mp4
    ```
    This will run the optical SLAM logic on the video, extract camera motion and write it as JSON files to
    `/opt/pilotguru/data/ride-trajectories`. The SLAM system may needs some time to pick up localization,
    and also may lose tracking in the middle of the video, so there may be multiple non-overlapping tracked
    segments, with  a separate JSON file written for each. Those JSON files should be treated as completely
    independent (i.e. there are no relations between world coordinate systems in different JSON files).
    
    If the SLAM system fails to start tracking at all, it may be that your camera calibration results are off.
    Try to re-calibrate a few times to see how much variance there is. Too high a resolution may also be a problem.
    
    The SLAM system estimates the full 3D trajectory of the camera (both translation and rotation), but the 
    translation information tends to be less reliable, so we will ignore it for now.
5. Raw SLAM trajectory estimates tend to be quite noisy. Apply Gaussian smoothing across frames to the
    rotation values to remove the high-frequency noise:
    ```
    ./smooth_heading_directions \
    --trajectory_in_file /opt/pilotguru/data/ride-trajectories/trajectory-0.json \
    --trajectory_out_file /opt/pilotguru/data/ride-trajectories/trajectory-0-smoothed.json \
    --sigma 5
    ```
    Larger sigma leads to more smoothing. Try out a couple of values to see what is the smallest one that
    still removes most of the jitter.
6. Visualize the results.
    ```
    ./render_turning \
        --in_video /opt/pilotguru/data/ride.mp4 \
        --trajectory_json /opt/pilotguru/data/ride-trajectories/trajectory-0-smoothed.json \
        --steering_wheel /opt/pilotguru/img/steering_wheel_image.jpg \
        --out_video /opt/pilotguru/data/ride-0-smoothed.mp4
    ```
    This will produce a video [like this](https://youtu.be/gMXn0IMcX-k) with a rotating steering wheel 
    tiled next to the original ride recording.
    The steering wheel rotation should generally match the car steering.
    
    **Caveat**: the steering wheel rotation in the rendering is somewhat misleading: it is proportional
    to the *angular velocity* of the car, so for turns at slow speed the rendered steering wheel will 
    rotate less than the real one in the car, and vice versa for high speed.

#### Steering angle inference - details <a name="steering-details"/>

Steps that happen to extract the steering angle from video, and output JSON data format.

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
    
2. Infer (approximately) the horizontal (i.e. road surface) plane using [principal components analysis](https://en.wikipedia.org/wiki/Principal_component_analysis). As long as the car does not drive just in a straight line, or goes on a mountain [switchback road](https://bikealps.files.wordpress.com/2011/08/dolomites-20110815-dsc_0039.jpg), the distance traveled in any horizontal direction will be much larger than in the vertical direction. So we simply take the top two PCA components as the basis vectors of the horizontal plane. The basis vectors are stored in every JSON file as the `plane` field:
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

3. Project the 3D rotation onto the 2D inferred horizontal plane. Stored for every frame as `(x,y)` unit vectors in the reference frame of the plane basis vectors:
    ```
    "planar_direction": [
      0.324955405433993,
      0.93074165275327
    ],
    ```

4. Finally, from the horizontal directions we can compute the relative horizontal rotation between the two frames. Use the direction of the cross product to distinguish left and right turns:
    ```
    v = cross(prev_frame_flat_direction, current_frame_flat_direction)
    alpha = acos(dot(prev_frame_flat_direction, current_frame_flat_direction)) * sign(v_z)
    ```
    The results are stored as `turn_angle` field for every frame (in radians):
    ```
    "turn_angle": -0.0054
    ```

5. All of the above steps are prone to high-fequency noise, showing up in the SLAM camera trajectory and propagating to the rotation magnitudes. To reduce the noise, `smooth_heading_directions` binary applies [Gaussian blur](https://en.wikipedia.org/wiki/Gaussian_blur) to the 3D camera rotations and propagates the results downstream.
