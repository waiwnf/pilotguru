import argparse
import json

import image_helpers
import models

import numpy as np

import torch
from torch.autograd import Variable

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument(
    '--in_video', required=True,
    help='MPEG video file from Pilotguru Recorder.')
  parser.add_argument('--net_name', default=models.NVIDIA_NET_NAME)
  parser.add_argument('--net_out_total_dimensions', type=int, default=1)
  parser.add_argument('--net_out_dimension_to_use', type=int, default=0)
  parser.add_argument(
    '--in_model_weights', required=True, help='Pytorch model weights file')
  parser.add_argument(
    '--out_steering_json', required=True,
    help='JSON file name to write results to.')

  # Crop settings
  parser.add_argument('--crop_top', type=int, default=0)
  parser.add_argument('--crop_bottom', type=int, default=0)
  parser.add_argument('--crop_left', type=int, default=0)
  parser.add_argument('--crop_right', type=int, default=0)

  # Post-crop resize settings.
  parser.add_argument('--target_height', type=int, default=-1)
  parser.add_argument('--target_width', type=int, default=-1)

  args = parser.parse_args()

  # Init model and load weights.
  net = models.MakeNetwork(
      args.net_name,
      in_shape=[3, args.target_height, args.target_width],
      out_dims=args.net_out_total_dimensions,
      dropout_prob=0.0)
  net.load_state_dict(torch.load(args.in_model_weights))
  net.eval()
  net.cuda()

  result_data = []
  frames_generator = image_helpers.VideoFrameGenerator(args.in_video)
  for raw_frame, frame_index in frames_generator:
    frame_cropped = image_helpers.CropHWC(
        raw_frame,
        args.crop_top, args.crop_bottom, args.crop_left, args.crop_right)
    frame_resized = image_helpers.MaybeResizeHWC(
        frame_cropped, args.target_height, args.target_width)
    frame_chw = np.transpose(frame_resized, (2,0,1))
    frame_float = frame_chw.astype(np.float32) / 255.0
    # Add a dummy dimension to make it a "batch" of size 1.
    frame_tensor = Variable(
        torch.from_numpy(frame_float[np.newaxis,...])).cuda()
    result_tensor = net(frame_tensor).cpu()
    result_value = result_tensor.data.numpy()[0,args.net_out_dimension_to_use].item()
    result_data.append({'frame_id': frame_index, 'angular_velocity': result_value})
  
  with open(args.out_steering_json, 'w') as out_json:
    json.dump({'steering': result_data}, out_json, indent=2)