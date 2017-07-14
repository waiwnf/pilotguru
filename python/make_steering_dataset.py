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
    'FrameData', ['frame_id', 'angular_velocity', 'speed_m_s'])

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
  return os.path.join(args.out_dir, 'frame-%06d-%s.npy') % (frame_id, data_id)

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
  
  # Crop settings
  parser.add_argument('--crop_top', type=int, default=0)
  parser.add_argument('--crop_bottom', type=int, default=0)
  parser.add_argument('--crop_left', type=int, default=0)
  parser.add_argument('--crop_right', type=int, default=0)

  # Post-crop resize settings.
  parser.add_argument('--target_height', type=int, default=-1)
  parser.add_argument('--target_width', type=int, default=-1)

  args = parser.parse_args()

  out_color_channels = 1 if args.convert_to_grayscale else 3
  
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
      '--smoothing_sigma', '0.1',
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
      steering_frames_json['steering'], velocities_frames_json['velocities'])

  # Open the video file for reading the frames.
  frames_generator = image_helpers.VideoFrameGenerator(args.in_video)

  # Id of previous frame for which the data was written out.
  prev_saved_frame_id = None
  prev_seen_frame_data_id = None
  for frame_data in frames_data:
    # Skip if no angular velocity data
    if frame_data.angular_velocity is None:
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
    raw_frame, frame_index = next(frames_generator)
    while frame_index < frame_id:
      raw_frame, frame_index = next(frames_generator)
    assert frame_index == frame_id
    history_index = frame_index % raw_history_size
    frame_chw, frame_hwc = FrameToModelInput(
        raw_frame,
        args.crop_top, args.crop_bottom, args.crop_left, args.crop_right,
        args.target_height, args.target_width, args.convert_to_grayscale,
        args.convert_to_yuv)
    raw_frames_history[history_index, ...] = frame_chw
    raw_steering_history[history_index, 0] = frame_data.angular_velocity
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
    # Write out numpy array for the training example input.
    image_out_name = OutFileName(args.out_dir, out_frame_id, 'img')
    np.save(image_out_name, raw_frames_history[write_indices, ...])
    # Write out the PNG picture too for checking the inputs by hand.
    scipy.misc.imsave(image_out_name + '.png', np.squeeze(frame_hwc))
    # Steering angular velocity is the label.
    angular_out_name = OutFileName(args.out_dir, out_frame_id, 'angular')
    angular_out_data = LabelDataWithLookaheads(
        raw_steering_history, write_indices, label_lookahead_frames)
    np.save(angular_out_name, angular_out_data)
    velocity_out_data = LabelDataWithLookaheads(
        raw_velocity_history, write_indices, label_lookahead_frames)
    # Inverse turn radius.
    inverse_radius_out_name = OutFileName(
        args.out_dir, out_frame_id, 'inverse-radius')
    inverse_radius_data = (
        angular_out_data / (velocity_out_data + 1.0)).astype(np.float32)
    np.save(inverse_radius_out_name, inverse_radius_data)