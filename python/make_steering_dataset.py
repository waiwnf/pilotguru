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
import shutil
import subprocess
import tempfile

import scipy.misc

import numpy as np

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
  args = parser.parse_args()

  tempdir = tempfile.mkdtemp()
  print('Created temporary directory: %s' % (tempdir,))

  # Extract all video frames as PNGs to the temporary directory.
  out_frames_pattern = os.path.join(tempdir, 'frame-%06d.png')
  subprocess.call(['ffmpeg', '-i', args.in_video, out_frames_pattern])

  # Compute per-frame steering angular velocity annotations from the raw 
  # angular velocity time series.
  annotate_frames_bin = os.path.join(args.binary_dir, 'annotate_frames')
  steering_frames_json_name = os.path.join(tempdir, 'steering.json')
  subprocess.call(
    [annotate_frames_bin,
      '--frames_json', args.in_frames_json,
      '--in_json', args.in_steering_json,
      '--json_root_element_name', 'steering',
      '--json_value_name', 'angular_velocity',
      '--out_json', steering_frames_json_name])
  
  with open(steering_frames_json_name) as f:
    steering_frames_json = json.load(f)
  
  # Id of previous frame for which the data was written out.
  prev_frame_id = None
  for frame in steering_frames_json['steering']:
    frame_id = frame['frame_id']
    if prev_frame_id is None or (frame_id - prev_frame_id) >= args.frames_step:
      prev_frame_id = frame_id
      angular_velocity = frame['angular_velocity']
      # FFMpeg frame ids are 1-based, so add 1 to our 0-based frame ids to get 
      # the filename.
      frame_image_name = out_frames_pattern % (frame_id + 1)

      # Raw image is in HWC order.
      frame_image_raw = scipy.misc.imread(frame_image_name, mode='RGB')
      # Transpose to CHW for pytorch.
      frame_image = np.transpose(frame_image_raw, (2,0,1))

      image_out_name = OutFileName(args.out_dir, frame_id, 'img')
      np.save(image_out_name, frame_image)
      angular_out_name = OutFileName(args.out_dir, frame_id, 'angular')
      np.save(angular_out_name, np.array([angular_velocity], dtype=np.float32))

  # Remove the intermediate PNGs and jsons.
  shutil.rmtree(tempdir)