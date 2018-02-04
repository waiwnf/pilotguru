import argparse
import json
import time

import image_helpers
import io_helpers
import prediction_helpers

import cv2
import zmq

import torch
from torch.autograd import Variable

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('--in_video_device_id', type=int, default=None)
  parser.add_argument('--in_video_file', default=None)
  parser.add_argument('--delay_max_fps', type=float, default=-1)
  parser.add_argument('--skip_max_fps', type=float, default=-1)
  parser.add_argument('--forward_axis_json', required=True)
  parser.add_argument('--net_settings_json', required=True)
  parser.add_argument(
    '--in_model_weights', required=True,
    help='Pytorch model weights files (comma-separated list).')
  parser.add_argument('--convert_to_yuv', type=bool, default=False)
  parser.add_argument('--cuda_device_id', type=int, default=0)
  parser.add_argument('--trajectory_frame_update_rate', type=float, default=1.0)
  parser.add_argument(
      '--prediction_units_to_degrees_scale', type=float, default=90.0)
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

  # Initialize the base frame generator, either capturing frames live from a
  # USB camera, or reading from a video file.
  base_frame_generator = None
  video_capture = None
  if args.in_video_device_id is not None:
    assert base_frame_generator is None
    video_capture = cv2.VideoCapture(args.in_video_device_id)
    base_frame_generator = image_helpers.VideoCaptureFrameGenerator(
        video_capture)
  elif args.in_video_file:
    assert base_frame_generator is None
    base_frame_generator = image_helpers.VideoFrameGenerator(args.in_video_file)
  
  # Make sure the frames arrive at roughly the frame rate we need.
  # First add delays between captured frames (needed when reading from a video
  # file to simulate the original camera frame rate).
  # Then add frame skipping. This can be effective both for video files (to e.g.
  # get a 10 FPS subsequence from a 30 FPS video) and live USB camera capture.
  assert base_frame_generator is not None
  frame_generator_delay = image_helpers.VideoFrameGeneratorLimitedFpsDelay(
      base_frame_generator, args.delay_max_fps)
  final_frame_generator = image_helpers.VideoFrameGeneratorLimitedFpsSkip(
      frame_generator_delay, args.skip_max_fps)
  
  trajectory_prediction = None
  print('Live prediction started.')
  for raw_frame, _ in final_frame_generator:
    frame_variable, frame_display = prediction_helpers.RawFrameToModelInput(
        raw_frame=raw_frame,
        crop_settings=args,
        net_settings=net_settings,
        convert_to_yuv=args.convert_to_yuv,
        cuda_device_id=args.cuda_device_id)
    prediction_single_frame = prediction_helpers.EvalModelsEnsemble(
        nets, [frame_variable, forward_axis_tensor])
    # TODO downsample the single frame trajectory prediction in case the live
    # source framerate is different from the framerate on which the model was
    # trained.
    trajectory_prediction = prediction_helpers.UpdateFutureTrajectoryPrediction(
        trajectory_prediction,
        prediction_single_frame,
        args.trajectory_frame_update_rate)
    prediction_degrees = (
        trajectory_prediction[0,0].item() *
        args.prediction_units_to_degrees_scale)
    prediction_dict = {'s': prediction_degrees}
    socket.send_json(prediction_dict)

    cv2.imshow('frame', cv2.cvtColor(frame_display, cv2.COLOR_RGB2BGR))
    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  # When everything done, release the capture.
  if video_capture is not None:
    video_capture.release()
  cv2.destroyAllWindows()