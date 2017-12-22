# Generates linearly adjusted label augmentation magnitudes to be used as
# --horizontal_label_shift_rate for train.py.

import argparse

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--start_value', type=float, default=0.0)
  parser.add_argument('--end_value', type=float, default=0.0)
  parser.add_argument('--dims', type=int, default=1)
  args = parser.parse_args()

  label_shift_values = [
      str((args.start_value * (args.dims - i) + args.end_value * i) / args.dims)
      for i in range(args.dims)]
  print(','.join(label_shift_values))
