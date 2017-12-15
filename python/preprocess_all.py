# Wrapper script for the raw IMU autocalibration and CAN bus data processing.

import argparse
import os
import subprocess

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--binary_dir', required=True)
  parser.add_argument('--in_dir', required=True)
  parser.add_argument('--out_dir')
  parser.add_argument('--process_can_data', type=bool, default=False)
  args = parser.parse_args()

  out_dir = args.out_dir
  if out_dir is None:
    out_dir = os.path.join(args.in_dir, 'postprocessed')

  # Convert raw GPS+IMU data to steering and velocity.
  fit_motion_binary = os.path.join(args.binary_dir, 'fit_motion')
  subprocess.call(
    [fit_motion_binary,
      '--rotations_json', os.path.join(args.in_dir, 'rotations.json'),
      '--accelerations_json', os.path.join(args.in_dir, 'accelerations.json'),
      '--locations_json', os.path.join(args.in_dir, 'locations.json'),
      '--compute_steering_raw_rotations',
      '--compute_velocities_from_imu',
      '--velocities_out_json', os.path.join(out_dir, 'velocities-imu.json'),
      '--steering_out_json_raw', os.path.join(out_dir, 'steering-imu.json')])

  # Convert raw CAN bus data to steering and velocity.
  if args.process_can_data:
    process_can_frames_binary = os.path.join(args.binary_dir, 'process_can_frames')
    subprocess.call(
      [process_can_frames_binary,
        '--can_frames_json', os.path.join(args.in_dir, 'can_frames.json'),
        '--velocities_out_json', os.path.join(out_dir, 'velocities-can.json'),
        '--steering_out_json', os.path.join(out_dir, 'steering-can.json')])

