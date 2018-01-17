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
import skvideo.io
import numpy as np

import image_helpers
import io_helpers

_FRAME_ID = 'frame_id'
_ANGULAR_VELOCITY = 'angular_velocity'
_SPEED_M_S = 'speed_m_s'
_STEERING = 'steering'
_STEERING_GENERIC_VALUE = 'steering_generic_value'
_STEERING_ANGLE_DEGREES = 'steering_angle_degrees'
_VELOCITIES = 'velocities'

_CROP_TOP = 'crop_top'
_CROP_BOTTOM = 'crop_bottom'
_CROP_LEFT = 'crop_left'
_CROP_RIGHT = 'crop_right'

_IMU = 'imu'
_CAN = 'can'

# Multipliers to bring steering data inferred from IMU and obtained directly
# from the CAN bus to uniform units to be used as labels for the trained models.
#
# The generic steering unit is 90 degrees of steering wheel rotation.
# Empirically, we have
# [degrees of steering wheel rotation] ~= 2500 * [inverse steering radius],
# hence the multipliers.
_CAN_DEGREES_TO_STEERING_UNITS = 1.0 / 90.0
_INVERSE_RADIUS_METERS_TO_STEERING_UNITS = 28.0

# JSON field names to look at for the steering data, depending on the data 
# source.
_STEERING_VALUE_BY_SOURCE = {
  _IMU: _ANGULAR_VELOCITY,
  _CAN: _STEERING_ANGLE_DEGREES
}

# Temporal smoothing settings for the steering data.
# IMU-derived data is rather noisy, so a little temporal smoothing is useful.
# Data coming from the CAN bus is good enough (noise within 0.5 steering wheel 
# turn degree), so extra smoothing is not necessary.
_SMOOTHING_SETTINGS_BY_STEERING_SOURCE = {
  _IMU: ['--smoothing_sigma', '0.1'],
  _CAN: []
}

# Internal data structure to hold per-frame steering and velocity vales.
# The steering value is interpreted differently by the helpers, depending on the
# specified steering data source (CAN bus or IMU).
FrameData = collections.namedtuple(
    'FrameData', [_FRAME_ID, _STEERING_GENERIC_VALUE, _SPEED_M_S])

def FillFrameData(steering, velocity, steering_source):
  assert velocity is not None or steering is not None
  if velocity is not None and steering is not None:
    assert velocity[_FRAME_ID] == steering[_FRAME_ID]
  frame_id = (
      velocity[_FRAME_ID] if velocity is not None else steering[_FRAME_ID])
  
  velocity_value = None
  if velocity is not None:
    velocity_value = velocity[_SPEED_M_S]

  steering_value = None
  if steering is not None:
    steering_value = steering[_STEERING_VALUE_BY_SOURCE[steering_source]]

  return FrameData(frame_id, steering_value, velocity_value)

def JoinFrameData(steering, velocities, steering_source):
  steering_idx = 0
  velocities_idx = 0
  result = []
  while steering_idx < len(steering) or velocities_idx < len(velocities):
    v = None if velocities_idx >= len(velocities) else (
        velocities[velocities_idx])
    s = None if steering_idx >= len(steering) else steering[steering_idx]
    if v is not None and s is not None:
      if s[_FRAME_ID] < v[_FRAME_ID]:
        v = None
      elif s[_FRAME_ID] > v[_FRAME_ID]:
        s = None
    
    result.append(FillFrameData(s, v, steering_source))
    
    if s is not None:
      steering_idx += 1
    if v is not None:
      velocities_idx += 1

  return result

def FrameToModelInput(
    raw_frame,
    crop_top, crop_bottom, crop_left, crop_right,
    target_height, target_width, convert_to_grayscale,
    convert_to_yuv):
  # Grayscale and YUV outputs are mutually exclusive.
  # TODO: make an output enum instead (RGB, YUV, GRAY).
  assert not convert_to_grayscale or not convert_to_yuv
  # Crop to the ROI for the vision model.
  frame_image_cropped = image_helpers.CropHWC(
      raw_frame, crop_top, crop_bottom, crop_left, crop_right)
  frame_image_resized = image_helpers.MaybeResizeHWC(
      frame_image_cropped, target_height, target_width)
  # Optionally convert to grayscale.
  if convert_to_grayscale:
    rgb_to_gray_weights = np.array(
      [0.2989, 0.5870, 0.1140], dtype=np.float64).reshape([1,1,3])
    frame_image_resized = np.sum(
        frame_image_resized.astype(np.float64) * rgb_to_gray_weights,
        axis=2, keepdims=True).astype(np.uint8)
  if convert_to_yuv:
    frame_image_resized = image_helpers.RgbToYuv(frame_image_resized)
  # Transpose to CHW for pytorch.
  frame_image_chw = np.transpose(frame_image_resized, (2,0,1))
  return frame_image_chw, frame_image_resized

def OutFileName(out_dir, frame_id, data_id):
  return os.path.join(args.out_dir, 'frame-%06d-%s') % (frame_id, data_id)

def LabelDataWithLookaheads(raw_labels, write_indices, lookaheads):
  assert len(raw_labels.shape) == 2
  assert raw_labels.shape[1] == 1
  result = np.zeros(shape=[len(write_indices), len(lookaheads)])
  total_raw_history_size = raw_labels.shape[0]
  for w, write_index in enumerate(write_indices):
    result[w,:] = raw_labels[
      [(write_index + lookahead) % total_raw_history_size
            for lookahead in lookaheads], 0]
  return result

def AnnotateFramesSteering(
    annotate_frames_bin,
    frames_json,
    in_steering_json,
    out_steering_json,
    steering_source):
  subprocess.call(
    [annotate_frames_bin,
      '--frames_json', frames_json,
      '--in_json', in_steering_json,
      '--json_root_element_name', _STEERING,
      '--json_value_name', _STEERING_VALUE_BY_SOURCE[steering_source],
      '--out_json', steering_frames_json_name] + 
      _SMOOTHING_SETTINGS_BY_STEERING_SOURCE[steering_source])

# Interpret the steering data as either angular velocity from IMU or steering
# wheel turn angle from the CAN bus and apply the appropriate multiplier to
# bring the data to the uniform scale.
def RawSteeringDataToSteeringLabels(raw_steering, velocities, steering_source):
  if steering_source == _CAN:
    return raw_steering * _CAN_DEGREES_TO_STEERING_UNITS
  elif steering_source == _IMU:
    inverse_radius = raw_steering / (velocities + 1.0)
    return inverse_radius * _INVERSE_RADIUS_METERS_TO_STEERING_UNITS
  else:
    assert False
    return None

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
  parser.add_argument('--steering_source', default=_CAN,
      help='Steering data source. Supported values: ' + 
            _CAN + ' and ' + _IMU + '.')
  parser.add_argument(
    '--in_velocities_json', required=True,
    help='Timestamped absolute forward velocities JSON, produced by ' +
      'fit_motion.')
  parser.add_argument(
    '--in_forward_axis_json', required=True,
    help='Vehicle forward motion axis direction in smartphone-local ' + 
      'reference frame, produced by fit_motion.')
  parser.add_argument('--crop_settings_json', required=True)
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
  parser.add_argument(
    '--frames_history_length', type=int, default=1,
    help='Number of sequential frames (spaced by --frames_history_step) to ' +
      'use per single training example.')
  parser.add_argument(
    '--frames_history_step', type=int, default=1,
    help='Spacing of sequential frames within an individual example (with a ' +
      'total of --frames_history_length frames per example)')
  parser.add_argument(
    '--label_lookahead_frames', default='0',
    help='Comma-separated list of lookaheads (in terms of number of frames) '
    'to use for the label annotations.')
  parser.add_argument(
    '--exclude_frames_json', default='',
    help='Optional JSON file with ranges of frame IDs to exclude. Format is ' +
    '\'exclude\' key mapped a list of 2-element lists [start_id, end_id].')
  parser.add_argument('--convert_to_grayscale', type=bool, default=False)
  parser.add_argument('--convert_to_yuv', type=bool, default=False)
  
  # Post-crop resize settings.
  parser.add_argument('--target_height', type=int, default=-1)
  parser.add_argument('--target_width', type=int, default=-1)

  parser.add_argument('--save_png_every', type=int, default=100)

  args = parser.parse_args()

  forward_axis = io_helpers.LoadForwardAxis(args.in_forward_axis_json)
  with open(args.crop_settings_json) as crop_settings_file:
    crop_settings_json = json.load(crop_settings_file)
  crop_settings = crop_settings_json['crop_settings']
  crop_top = crop_settings[_CROP_TOP] if _CROP_TOP in crop_settings else 0
  crop_bottom = (
      crop_settings[_CROP_BOTTOM] if _CROP_BOTTOM in crop_settings else 0)
  crop_left = crop_settings[_CROP_LEFT] if _CROP_LEFT in crop_settings else 0
  crop_right = crop_settings[_CROP_RIGHT] if _CROP_RIGHT in crop_settings else 0

  out_color_channels = 1 if args.convert_to_grayscale else 3
  
  annotate_frames_bin = os.path.join(args.binary_dir, 'annotate_frames')

  # Compute per-frame steering angular velocity annotations from the raw 
  # angular velocity time series.
  steering_frames_json_name = os.path.join(args.out_dir, 'steering_frames.json')
  AnnotateFramesSteering(
      annotate_frames_bin,
      args.in_frames_json,
      args.in_steering_json,
      steering_frames_json_name,
      args.steering_source)

  # Per-frame forward velocities annotations.
  velocities_frames_json_name = os.path.join(
      args.out_dir, 'velocities_frames.json')
  subprocess.call(
    [annotate_frames_bin,
      '--frames_json', args.in_frames_json,
      '--in_json', args.in_velocities_json,
      '--json_root_element_name', _VELOCITIES,
      '--json_value_name', _SPEED_M_S,
      '--out_json', velocities_frames_json_name])
  
  label_lookahead_frames = [
      int(x) for x in args.label_lookahead_frames.split(',')]
  label_lookahead_frames.sort()
  # No negative lookaheads for simplicity.
  assert min(label_lookahead_frames) >= 0
  max_lookahead = max(label_lookahead_frames)

  exclude_frames = set()
  if args.exclude_frames_json != '':
    with open(args.exclude_frames_json) as exclude_frames_file:
      exclude_frames_json = json.load(exclude_frames_file)
    for exclude_range in exclude_frames_json['exclude']:
      assert len(exclude_range) == 2
      exclude_frames.update(range(exclude_range[0], exclude_range[1] + 1))

  raw_history_size = (
      (args.frames_history_length - 1) * args.frames_history_step
          + 1 + max_lookahead)
  raw_frames_history = np.zeros(
      (raw_history_size, out_color_channels, args.target_height, args.target_width),
      dtype=np.uint8)
  raw_steering_history = np.zeros((raw_history_size, 1), dtype=np.float32)
  raw_velocity_history = np.zeros((raw_history_size, 1), dtype=np.float32)
  remaining_unfilled_history = raw_history_size

  with open(steering_frames_json_name) as f:
    steering_frames_json = json.load(f)
  with open(velocities_frames_json_name) as f:
    velocities_frames_json = json.load(f)
  frames_data = JoinFrameData(
      steering_frames_json[_STEERING],
      velocities_frames_json[_VELOCITIES],
      args.steering_source)

  # Open the video file for reading the frames.
  frames_generator = skvideo.io.vreader(args.in_video)

  # Id of previous frame for which the data was written out.
  prev_saved_frame_id = None
  prev_seen_frame_data_id = None
  total_samples_written = 0
  frame_index = 0
  for frame_data in frames_data:
    # Skip if no steering data
    if frame_data.steering_generic_value is None:
      remaining_unfilled_history = raw_history_size
      continue
    # Skip if no forward velocity data or forward velocity is too low.
    if (frame_data.speed_m_s is None or
        frame_data.speed_m_s < args.min_forward_velocity_m_s):
      remaining_unfilled_history = raw_history_size
      continue
    
    frame_id = frame_data.frame_id
    # Skip the blacklisted frames.
    if frame_id in exclude_frames:
      remaining_unfilled_history = raw_history_size
      continue

    # Invalidate history if there were skipped frames in frame data.
    if (prev_seen_frame_data_id is not None and 
        frame_id != prev_seen_frame_data_id + 1):
      remaining_unfilled_history = raw_history_size

    prev_seen_frame_data_id = frame_id

    # Skip video frames until we get to the requested frame id.
    raw_frame = next(frames_generator)
    frame_index += 1
    while frame_index < frame_id:
      raw_frame = next(frames_generator)
      frame_index += 1
    assert frame_index == frame_id
    history_index = frame_index % raw_history_size
    frame_chw, frame_hwc = FrameToModelInput(
        raw_frame,
        crop_top, crop_bottom, crop_left, crop_right,
        args.target_height, args.target_width, args.convert_to_grayscale,
        args.convert_to_yuv)
    raw_frames_history[history_index, ...] = frame_chw
    raw_steering_history[history_index, 0] = frame_data.steering_generic_value
    raw_velocity_history[history_index, 0] = frame_data.speed_m_s
    remaining_unfilled_history = max(0, remaining_unfilled_history - 1)

    if remaining_unfilled_history > 0:
      continue

    # Skip if too few frames were seen since the last saved output.
    if (prev_saved_frame_id is not None and 
        (frame_id - prev_saved_frame_id) < args.frames_step):
      continue

    # Have all the necessary history filled in and long enough since the
    # previous write. Write a new example ending in this frame.
    prev_saved_frame_id = frame_id
    write_indices = [
        (history_index - max_lookahead - x * args.frames_history_step) % raw_history_size
        for x in range(args.frames_history_length)]
    # Reverse the indices to get the time-natural ordering.
    write_indices.reverse()

    out_frame_id = frame_id - max_lookahead

    # Write out the PNG picture too for checking the inputs by hand.
    if total_samples_written % args.save_png_every == 0:
      image_out_name = OutFileName(args.out_dir, out_frame_id, 'img')
      scipy.misc.imsave(image_out_name + '.png', np.squeeze(frame_hwc))

    # Raw steering and velocity data.
    raw_steering_data = LabelDataWithLookaheads(
        raw_steering_history, write_indices, label_lookahead_frames)
    velocities_data = LabelDataWithLookaheads(
        raw_velocity_history, write_indices, label_lookahead_frames)
    # Steering data normalized to the uniform (CAN, IMU) label space.
    steering_labels_data = RawSteeringDataToSteeringLabels(
        raw_steering_data, velocities_data, args.steering_source)
    
    frame_img = np.squeeze(raw_frames_history[write_indices, ...], axis=0)
    steering = np.squeeze(steering_labels_data, axis=0)

    sample_out_name = OutFileName(args.out_dir, out_frame_id, 'data')
    np.savez_compressed(
      sample_out_name,
      frame_img=frame_img,
      steering=steering.astype(np.float32),
      forward_axis=forward_axis)
    total_samples_written += 1