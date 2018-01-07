import argparse
import json

import image_helpers
import io_helpers
import models
import training_helpers

import numpy as np

import torch
from torch.autograd import Variable

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument(
    '--in_video', required=True,
    help='MPEG video file from Pilotguru Recorder.')
  parser.add_argument('--forward_axis_json', required=True)
  parser.add_argument('--net_settings_json', required=True)
  parser.add_argument(
    '--in_model_weights', required=True, help='Pytorch model weights file')
  parser.add_argument(
    '--out_steering_json', required=True,
    help='JSON file name to write results to.')
  parser.add_argument('--convert_to_yuv', type=bool, default=False)

  parser.add_argument('--cuda_device_id', type=int, default=0)
  
  # Crop settings
  parser.add_argument('--crop_top', type=int, default=0)
  parser.add_argument('--crop_bottom', type=int, default=0)
  parser.add_argument('--crop_left', type=int, default=0)
  parser.add_argument('--crop_right', type=int, default=0)

  args = parser.parse_args()

  with open(args.net_settings_json, 'r') as f:
    net_settings = json.load(f)
  
  forward_axis = io_helpers.LoadForwardAxis(args.forward_axis_json)
  forward_axis_tensor = Variable(
      torch.from_numpy(forward_axis[np.newaxis, ...])).cuda(args.cuda_device_id)

  # Init model and load weights.
  net = models.MakeNetwork(
      net_settings[training_helpers.NET_NAME],
      in_shape=[
          net_settings[training_helpers.IN_CHANNELS],
          net_settings[training_helpers.TARGET_HEIGHT],
          net_settings[training_helpers.TARGET_WIDTH]],
      head_dims=net_settings[training_helpers.NET_HEAD_DIMS],
      out_dims=net_settings[training_helpers.LABEL_DIMENSIONS],
      dropout_prob=net_settings[training_helpers.DROPOUT_PROB],
      options=net_settings[training_helpers.NET_OPTIONS])
  net.load_state_dict(torch.load(args.in_model_weights))
  net.cuda(args.cuda_device_id)
  net.eval()

  result_data = []
  frames_generator = image_helpers.VideoFrameGenerator(args.in_video)
  for raw_frame, frame_index in frames_generator:
    frame_cropped = image_helpers.CropHWC(
        raw_frame,
        args.crop_top, args.crop_bottom, args.crop_left, args.crop_right)
    frame_resized = image_helpers.MaybeResizeHWC(
        frame_cropped,
        net_settings[training_helpers.TARGET_HEIGHT],
        net_settings[training_helpers.TARGET_WIDTH])
    if args.convert_to_yuv:
      frame_resized = image_helpers.RgbToYuv(frame_resized)
    frame_chw = np.transpose(frame_resized, (2,0,1))
    frame_float = frame_chw.astype(np.float32) / 255.0
    # Add a dummy dimension to make it a "batch" of size 1.
    frame_tensor = Variable(
        torch.from_numpy(frame_float[np.newaxis,...])).cuda(args.cuda_device_id)
    result_tensor = net([frame_tensor, forward_axis_tensor])[0].cpu()
    result_value = result_tensor.data.numpy()[0,0].item()
    result_data.append(
        {'frame_id': frame_index, 'angular_velocity': result_value})
  
  with open(args.out_steering_json, 'w') as out_json:
    # TODO reuse constants
    json.dump({'steering': result_data}, out_json, indent=2)