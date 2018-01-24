import argparse
import json
import time

import io_helpers
import prediction_helpers

import cv2
import zmq

import torch
from torch.autograd import Variable

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--in_video_device_id', type=int, required=True)
  parser.add_argument('--forward_axis_json', required=True)
  parser.add_argument('--net_settings_json', required=True)
  parser.add_argument(
    '--in_model_weights', required=True,
    help='Pytorch model weights files (comma-separated list).')
  parser.add_argument('--convert_to_yuv', type=bool, default=False)
  parser.add_argument('--cuda_device_id', type=int, default=0)
  parser.add_argument('--trajectory_frame_update_rate', type=float, default=1.0)
  parser.add_argument(
    '--steering_prediction_socket', default='ipc:///tmp/steering-predict')
  
  prediction_helpers.AddCropArgs(parser)

  args = parser.parse_args()

  context = zmq.Context()
  socket = context.socket(zmq.PUB)
  # We want only the most recent value to be published. If the prediction from
  # the previous frame is still not sent by the time the next frame is
  # processed, we want to completely ignore the previous frame prediction and
  # work with the next one.
  socket.setsockopt(zmq.CONFLATE, 1)
  socket.bind(args.steering_prediction_socket)

  with open(args.net_settings_json, 'r') as f:
    net_settings = json.load(f)

  forward_axis = io_helpers.LoadForwardAxis(args.forward_axis_json)
  forward_axis_tensor = Variable(
      torch.from_numpy(forward_axis).unsqueeze(0)).cuda(args.cuda_device_id)

    # Init model and load weights.
  nets = prediction_helpers.LoadPredictionModels(
      args.in_model_weights.split(','), net_settings, args.cuda_device_id)
  
  video_capture = cv2.VideoCapture(args.in_video_device_id)

  trajectory_prediction = None
  print('Started.')
  while True:
    frame_capture_status, raw_frame = video_capture.read()
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
    prediction_dict = {'s': trajectory_prediction[0,0].item()}
    socket.send_json(prediction_dict)

    # cv2.imshow('frame', raw_frame)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #   break

  # When everything done, release the capture
  video_capture.release()
  cv2.destroyAllWindows()