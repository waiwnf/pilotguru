import argparse
import json

import image_helpers
import io_helpers
import models
import training_helpers

import numpy as np
import skvideo.io
import torch
from torch.autograd import Variable

def UpdateFutureTrajectoryPrediction(previous_prediction, current_update, lr):
  assert len(current_update.shape) == 2
  assert current_update.shape[0] == 1
  assert lr > 0
  assert lr <= 1

  if previous_prediction is None:
    return np.copy(current_update)

  assert previous_prediction.shape == current_update.shape
  result = np.copy(previous_prediction)
  result[0,:-1] = (
    lr * current_update[0,:-1] + (1.0 - lr) * previous_prediction[0,1:])
  result[0, -1] = current_update[0,-1]
  return result

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument(
    '--in_video', required=True,
    help='MPEG video file from Pilotguru Recorder.')
  parser.add_argument('--forward_axis_json', required=True)
  parser.add_argument('--net_settings_json', required=True)
  parser.add_argument(
    '--in_model_weights', required=True,
    help='Pytorch model weights files (comma-separated list).')
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

  parser.add_argument('--trajectory_frame_update_rate', type=float, default=1.0)

  args = parser.parse_args()

  with open(args.net_settings_json, 'r') as f:
    net_settings = json.load(f)
  
  forward_axis = io_helpers.LoadForwardAxis(args.forward_axis_json)
  forward_axis_tensor = Variable(
      torch.from_numpy(forward_axis[np.newaxis, ...])).cuda(args.cuda_device_id)

  # Init model and load weights.
  nets = []
  for weights_filename in args.in_model_weights.split(','):
    net = models.MakeNetwork(
        net_settings[training_helpers.NET_NAME],
        in_shape=[
            net_settings[training_helpers.IN_CHANNELS],
            net_settings[training_helpers.TARGET_HEIGHT],
            net_settings[training_helpers.TARGET_WIDTH]],
        head_dims=net_settings[training_helpers.NET_HEAD_DIMS],
        calibration_bias_dims=net_settings[NET_CALIBRATION_BIAS_DIMS],
        out_dims=net_settings[training_helpers.LABEL_DIMENSIONS],
        dropout_prob=net_settings[training_helpers.DROPOUT_PROB],
        options=net_settings[training_helpers.NET_OPTIONS])
    net.load_state_dict(torch.load(weights_filename))
    net.cuda(args.cuda_device_id)
    net.eval()
    nets.append(net)

  result_data = []
  frames_generator = skvideo.io.vreader(args.in_video)
  trajectory_prediction = None
  for frame_index, raw_frame in enumerate(frames_generator):
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
    result_components = np.array([
        net([frame_tensor, forward_axis_tensor])[0].cpu().data.numpy()
        for net in nets])
    result_averaged = np.mean(result_components, axis=0, keepdims=False)
    trajectory_prediction = UpdateFutureTrajectoryPrediction(
        trajectory_prediction,
        result_averaged,
        args.trajectory_frame_update_rate)
    result_value = trajectory_prediction[0,0].item()
    result_data.append(
        {'frame_id': frame_index, 'steering': result_value})
  
  with open(args.out_steering_json, 'w') as out_json:
    # TODO reuse constants
    json.dump({'steering': result_data}, out_json, indent=2)
