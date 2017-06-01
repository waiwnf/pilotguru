# From 
# - a video file
# - frame timestamps (frames.json from pilotguru recorder)
# - timestamped steering (angular velocities) json (produced by fit_motion)
# produces pairs of files (frame image, angular velocity) for individual frames
# in Numpy format.
# Results are intended to be easily readable by training pipelines (pytorch 
# etc).

import argparse
import json
import os
import re
import subprocess

import av
import scipy.misc
import numpy as np

def VideoOrientationDegrees(filename):
  '''Infers video rotation by parsing  avprobe output.'''
  avprobe_outcome = subprocess.run(['avprobe', filename], stderr=subprocess.PIPE)
  for line in avprobe_outcome.stderr.decode('utf-8').split('\n'):
    match_result = re.match('^\s*rotate\s*:\s*([0-9]+)\s*$', line)
    if match_result is not None:
      return int(match_result.group(1))
  return 0  # No rotation information is in the file, hence zero rotation.

def Video90RotationTimes(rotation_degrees):
  '''Converts rotation in degrees to number of 90-degree rotations.'''
  assert rotation_degrees % 90 == 0
  return (rotation_degrees % 360) / 90

def RawVideoFrameGenerator(filename):
  container = av.open(filename)
  for packet in container.demux():
    for frame in packet.decode():
      if isinstance(frame, av.video.frame.VideoFrame):
        yield frame

def OutFileName(out_dir, frame_id, data_id):
  return os.path.join(args.out_dir, 'frame-%06d-%s.npy') % (frame_id, data_id)

def Crop(img, top, bottom, left, right):
  """Crops given number of pixels from the edges of the image in HWC format."""

  assert left >= 0
  assert right >= 0
  assert top >= 0
  assert bottom >= 0
  assert (top + bottom) < img.shape[0]
  assert (left + right) < img.shape[1]

  return img[top:(img.shape[0] - bottom), left:(img.shape[1] - right), ...]

def MaybeResize(img, height, width):
  if height <= 0 and width <= 0:
    return img
  else:
    effective_height = height if height > 0 else img.shape[0]
    effective_width = width if width > 0 else img.shape[1]
    return scipy.misc.imresize(img, (effective_height, effective_width))

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument(
    '--in_video', required=True,
    help='MPEG video file from Pilotguru Recorder.')
  parser.add_argument(
    '--in_frames_json', required=True,
    help='Video frames timestamps file from Pilotguru Recorder.')
  parser.add_argument(
    '--in_steering_json', required=True,
    help='Timestamped steering angular velocities JSON, produced by ' +
      'fit_motion.')
  parser.add_argument(
    '--binary_dir', required=True,
    help='Directory where the compiled pilotguru C++ binaries are located.')
  parser.add_argument(
    '--out_dir', required=True,
    help='Output directory to write data files to.')
  parser.add_argument(
    '--frames_step', type=int, default=10,
    help='Only writes out data for every --frames_step\'s frame. ' +
      'Used to reduce the effective frame rate and have fewer very similar ' +
      'redundant examples.')
  
  # Crop settings
  parser.add_argument('--crop_top', type=int, default=0)
  parser.add_argument('--crop_bottom', type=int, default=0)
  parser.add_argument('--crop_left', type=int, default=0)
  parser.add_argument('--crop_right', type=int, default=0)

  # Post-crop resize settings.
  parser.add_argument('--target_height', type=int, default=-1)
  parser.add_argument('--target_width', type=int, default=-1)

  args = parser.parse_args()

  # Compute per-frame steering angular velocity annotations from the raw 
  # angular velocity time series.
  annotate_frames_bin = os.path.join(args.binary_dir, 'annotate_frames')
  steering_frames_json_name = os.path.join(args.out_dir, 'steering.json')
  subprocess.call(
    [annotate_frames_bin,
      '--frames_json', args.in_frames_json,
      '--in_json', args.in_steering_json,
      '--json_root_element_name', 'steering',
      '--json_value_name', 'angular_velocity',
      '--out_json', steering_frames_json_name])
  
  with open(steering_frames_json_name) as f:
    steering_frames_json = json.load(f)

  # Open the video file for reading the frames.
  rotation_times = Video90RotationTimes(VideoOrientationDegrees(args.in_video))
  frames_generator = RawVideoFrameGenerator(args.in_video)
  current_raw_frame = next(frames_generator)

  # Id of previous frame for which the data was written out.
  prev_frame_id = None
  for frame in steering_frames_json['steering']:
    frame_id = frame['frame_id']
    if prev_frame_id is None or (frame_id - prev_frame_id) >= args.frames_step:
      prev_frame_id = frame_id
      angular_velocity = frame['angular_velocity']
  
      # Skip video frames until we get to the requested frame id.
      while current_raw_frame.index < frame_id:
        current_raw_frame = next(frames_generator)
      assert current_raw_frame.index == frame_id

      # Raw image is in HWC order.
      frame_image_raw = np.asarray(current_raw_frame.to_image())
      # Rotate to allow for matrix orirentation in the camera.
      frame_image_rotated = np.rot90(frame_image_raw, k=rotation_times)
      # Crop to the ROI for the vision model.
      frame_image_cropped = Crop(
          frame_image_rotated,
          args.crop_top, args.crop_bottom, args.crop_left, args.crop_right)
      frame_image_resized = MaybeResize(
          frame_image_cropped, args.target_height, args.target_width)

      # Transpose to CHW for pytorch.
      frame_image = np.transpose(frame_image_resized, (2,0,1))

      # Write out numpy array for the training example input.
      image_out_name = OutFileName(args.out_dir, frame_id, 'img')
      np.save(image_out_name, frame_image)
      # Write out the PNG picture too for checking the inputs by hand.
      scipy.misc.imsave(image_out_name + '.png', frame_image_resized)
      # Steering angular velocity is the label.
      angular_out_name = OutFileName(args.out_dir, frame_id, 'angular')
      np.save(angular_out_name, np.array([angular_velocity], dtype=np.float32))