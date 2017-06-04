# From 
# - a video file
# - frame timestamps (frames.json from pilotguru recorder)
# - timestamped steering (angular velocities) json (produced by fit_motion)
# produces pairs of files (frame image, angular velocity) for individual frames
# in Numpy format.
# Results are intended to be easily readable by training pipelines (pytorch 
# etc).

import argparse
import collections
import json
import os
import subprocess

import scipy.misc
import numpy as np

import image_helpers

FrameData = collections.namedtuple(
    'FrameData', 'frame_id angular_velocity speed_m_s')

def FillFrameData(steering, velocity):
  assert velocity is not None or steering is not None
  if velocity is not None and steering is not None:
    assert velocity['frame_id'] == steering['frame_id']
  frame_id = (
      velocity['frame_id'] if velocity is not None else steering['frame_id'])
  s = velocity['speed_m_s'] if velocity is not None else None
  av = steering['angular_velocity'] if steering is not None else None
  return FrameData(frame_id, av, s)

def JoinFrameData(steering, velocities):
  steering_idx = 0
  velocities_idx = 0
  result = []
  while steering_idx < len(steering) or velocities_idx < len(velocities):
    v = None if velocities_idx >= len(velocities) else (
        velocities[velocities_idx])
    s = None if steering_idx >= len(steering) else steering[steering_idx]
    if v is None:
      result.append(FillFrameData(s, None))
      steering_idx += 1
    elif s is None:
      result.append(FillFrameData(None, v))
      velocities_idx += 1
    elif s['frame_id'] < v['frame_id']:
      result.append(FillFrameData(s, None))
      steering_idx += 1
    elif s['frame_id'] > v['frame_id']:
      result.append(FillFrameData(None, v))
      velocities_idx += 1
    else:
      result.append(FillFrameData(s, v))
      steering_idx += 1
      velocities_idx += 1
  return result

def OutFileName(out_dir, frame_id, data_id):
  return os.path.join(args.out_dir, 'frame-%06d-%s.npy') % (frame_id, data_id)

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
    '--in_velocities_json', required=True,
    help='Timestamped absolute forward velocities JSON, produced by ' +
      'fit_motion.')
  parser.add_argument(
    '--min_forward_velocity_m_s', type=float, default=0.0,
    help='Timestamped absolute forward velocities JSON, produced by ' +
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

  # Per-frame forward velocities annotations.
  velocities_frames_json_name = os.path.join(args.out_dir, 'velocities.json')
  subprocess.call(
    [annotate_frames_bin,
      '--frames_json', args.in_frames_json,
      '--in_json', args.in_velocities_json,
      '--json_root_element_name', 'velocities',
      '--json_value_name', 'speed_m_s',
      '--out_json', velocities_frames_json_name])
  
  with open(steering_frames_json_name) as f:
    steering_frames_json = json.load(f)
  with open(velocities_frames_json_name) as f:
    velocities_frames_json = json.load(f)
  frames_data = JoinFrameData(
      steering_frames_json['steering'], velocities_frames_json['velocities'])

  # Open the video file for reading the frames.
  frames_generator = image_helpers.VideoFrameGenerator(args.in_video)
  raw_frame, frame_index = next(frames_generator)

  # Id of previous frame for which the data was written out.
  prev_frame_id = None
  for frame_data in frames_data:
    # Skip if no angular velocity data
    if frame_data.angular_velocity is None:
      continue
    # Skip if no forward velocity data or forward velocity is too low.
    if (frame_data.speed_m_s is None or
        frame_data.speed_m_s < args.min_forward_velocity_m_s):
      continue
    
    frame_id = frame_data.frame_id
    # Skip if too few frames were seen since the last saved output.
    if (prev_frame_id is not None and 
        (frame_id - prev_frame_id) < args.frames_step):
      continue
   
    prev_frame_id = frame_id

    # Skip video frames until we get to the requested frame id.
    while frame_index < frame_id:
      raw_frame, frame_index = next(frames_generator)
    assert frame_index == frame_id

    # Crop to the ROI for the vision model.
    frame_image_cropped = image_helpers.CropHWC(
        raw_frame,
        args.crop_top, args.crop_bottom, args.crop_left, args.crop_right)
    frame_image_resized = image_helpers.MaybeResizeHWC(
        frame_image_cropped, args.target_height, args.target_width)

    # Transpose to CHW for pytorch.
    frame_image_chw = np.transpose(frame_image_resized, (2,0,1))

    # Write out numpy array for the training example input.
    image_out_name = OutFileName(args.out_dir, frame_id, 'img')
    np.save(image_out_name, frame_image_chw)
    # Write out the PNG picture too for checking the inputs by hand.
    scipy.misc.imsave(image_out_name + '.png', frame_image_resized)
    # Steering angular velocity is the label.
    angular_out_name = OutFileName(args.out_dir, frame_id, 'angular')
    np.save(
        angular_out_name,
        np.array([frame_data.angular_velocity],
        dtype=np.float32))