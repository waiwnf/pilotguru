import argparse
import json

import image_helpers
import io_helpers
import prediction_helpers

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
    '--in_model_weights', required=True,
    help='Pytorch model weights files (comma-separated list).')
  parser.add_argument(
    '--out_steering_json', required=True,
    help='JSON file name to write results to.')
  parser.add_argument('--convert_to_yuv', type=bool, default=False)
  parser.add_argument('--cuda_device_id', type=int, default=0)
  parser.add_argument('--trajectory_frame_update_rate', type=float, default=1.0)
  
  prediction_helpers.AddCropArgs(parser)

  args = parser.parse_args()

  with open(args.net_settings_json, 'r') as f:
    net_settings = json.load(f)
  
  forward_axis = io_helpers.LoadForwardAxis(args.forward_axis_json)
  forward_axis_tensor = Variable(
      torch.from_numpy(forward_axis).unsqueeze(0)).cuda(args.cuda_device_id)

  # Init model and load weights.
  nets = prediction_helpers.LoadPredictionModels(
      args.in_model_weights.split(','), net_settings, args.cuda_device_id)

  result_data = []
  frames_generator = image_helpers.VideoFrameGenerator(args.in_video)
  trajectory_prediction = None
  for raw_frame, frame_index in frames_generator:
    frame_variable = prediction_helpers.RawFrameToModelInput(
        raw_frame=raw_frame,
        crop_settings=args,
        net_settings=net_settings,
        convert_to_yuv=args.convert_to_yuv,
        cuda_device_id=args.cuda_device_id)
    prediction_single_frame = prediction_helpers.EvalModelsEnsemble(
        nets, [frame_variable, forward_axis_tensor])
    trajectory_prediction = prediction_helpers.UpdateFutureTrajectoryPrediction(
        trajectory_prediction,
        prediction_single_frame,
        args.trajectory_frame_update_rate)
    result_value = trajectory_prediction[0,0].item()
    result_data.append(
        {'frame_id': frame_index, 'steering': result_value})
  
  with open(args.out_steering_json, 'w') as out_json:
    # TODO reuse constants
    # TODO also write full trajectory predictions
    json.dump({'steering': result_data}, out_json, indent=2)
